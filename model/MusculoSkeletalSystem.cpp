#include "MusculoSkeletalSystem.h"
#include "DART_helper.h"
#include "../functions/functions.h"
#include "../generator/motionAdjust/muscleCal.h"
#include <tinyxml.h>
#include <algorithm>
#include <fstream>
#include <stdio.h>
using namespace dart::constraint;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace std;
Eigen::Vector3d
getAnchorPointGlobalCoord(const AnchorPoint &ap)
{
	// to keep the length of tendon, we should change here
	return ap.first->getTransform()*ap.second;
}
Eigen::Vector3d
getBlendedWaypointGlobalCoord(const BlendedWaypoint &bwp)
{
	// to keep the length of tendon, we should change here
 	Eigen::Vector3d res = Eigen::Vector3d(0, 0, 0);
	for (int i=0;i<bwp.weights.size();i++){
		res += bwp.weights[i] * (bwp.relatedBodyNodes[i]->getTransform() * bwp.relativeCoords[i]);
	}
	return res;
}
int
Muscle::
getNumForces()
{
	return blendedWaypoints.size();
}
Eigen::MatrixXd
Muscle::
getJacobianTranspose()
{
	const auto& skel = blendedWaypoints[0].ap.first->getSkeleton();
	int dof = skel->getNumDofs();
	Eigen::MatrixXd Jt(dof,3*blendedWaypoints.size());

	Jt.setZero();
	for(int i =0;i<blendedWaypoints.size();i++){
		auto bn = blendedWaypoints[i].ap.first;
		Eigen::Vector3d local_coord = getBlendedWaypointGlobalCoord(blendedWaypoints[i]);
		local_coord = bn->getTransform().inverse()*local_coord;
		Jt.block(0,i*3,dof,3) = skel->getLinearJacobian(bn,local_coord).transpose();
	}
	// std::cout<<Jt<<std::endl;
	return Jt;	
}



std::pair<Eigen::VectorXd,Eigen::VectorXd>
Muscle::
getForceJacobianAndPassive()
{
	compute_f_mt();
	double f_a = f_al(l_m);
	double f_p = f_pl(l_m);

	std::vector<Eigen::Vector3d> point,force_dir;
	for(int i =0;i<blendedWaypoints.size();i++){
		point.push_back(getBlendedWaypointGlobalCoord(blendedWaypoints[i]));
		force_dir.push_back(Eigen::Vector3d::Zero());
	}
	for(int i =0;i<blendedWaypoints.size()-1;i++)
	{
		Eigen::Vector3d dir = point[i+1]-point[i];
		dir.normalize();
		force_dir[i] += dir;
	}
	
	
	for(int i =1;i<blendedWaypoints.size();i++)
	{
		Eigen::Vector3d dir = point[i-1]-point[i];
		dir.normalize();
		force_dir[i] += dir;
	}

	Eigen::VectorXd A(3*blendedWaypoints.size());
	Eigen::VectorXd p(3*blendedWaypoints.size());
	A.setZero();
	p.setZero();

	for(int i =0;i<blendedWaypoints.size();i++)
	{
		A.segment<3>(i*3) = force_dir[i]*f_a;
		p.segment<3>(i*3) = force_dir[i]*f_p;
	}
	return std::make_pair(A,p);
}
void 
MusculoSkeletalSystem::
SetMax_l_m_tilda(double max_val)
{
	for(auto& muscle : mMuscles)
		muscle->max_l_m_tilda = max_val;
}

void 
MusculoSkeletalSystem::
SetMin_l_m_tilda(double min_val)
{
	for(auto& muscle : mMuscles)
		muscle->min_l_m_tilda = min_val;
}

void
Muscle::
initialize()
{
    mPassiveForce = 0;
}

void
Muscle::
transferForce(const double f_mt, Eigen::VectorXd &f)
{
	int n = blendedWaypoints.size();

	for (int i=0;i<n;i++){
		if (i==0){ // origin part
			Eigen::Vector3d v = (getBlendedWaypointGlobalCoord(blendedWaypoints[1])- getBlendedWaypointGlobalCoord(blendedWaypoints[0])).normalized();
			f.block<3,1>(i*3,0) = v * f_mt;
		}else if (i==n-1){ // insertion part
			Eigen::Vector3d v = (getBlendedWaypointGlobalCoord(blendedWaypoints[i - 1])- getBlendedWaypointGlobalCoord(blendedWaypoints[i])).normalized();
			f.block<3,1>(i*3,0) = v * f_mt;
		}else{ // inter points
			Eigen::Vector3d v = (getBlendedWaypointGlobalCoord(blendedWaypoints[i + 1])- getBlendedWaypointGlobalCoord(blendedWaypoints[i])).normalized();
			Eigen::Vector3d u = (getBlendedWaypointGlobalCoord(blendedWaypoints[i - 1])- getBlendedWaypointGlobalCoord(blendedWaypoints[i])).normalized();
			f.block<3,1>(i*3,0) = v * f_mt + u * f_mt;
		}
	}
}

void
Muscle::
setActivationLevel(double a)
{
	activation_level = a;
}

double Muscle::getLength() {
    double len = 0.0;
    for(int i=1; i<blendedWaypoints.size();i++){
        len += (getBlendedWaypointGlobalCoord(blendedWaypoints[i]) - getBlendedWaypointGlobalCoord(blendedWaypoints[i - 1])).norm();
    }
    return len;
}

double Muscle::getLmtilda() {
    return l_m_tilda;
}

double Muscle::getLm() {
    return l_m_tilda * l_m_o;
}

MusculoSkeletalSystem::
MusculoSkeletalSystem()
	:mTendonStiffness(1E6),mMuscleStiffness(1E6),mYoungsModulus(1E6),mPoissonRatio(0.3)
{

}
void
MusculoSkeletalSystem::
addMuscle(
		const std::string &name,
		const std::vector<int> &waypoint_indices,
		const double f_m_o,
		const double l_m_o,
		const double l_t_sl,
		const double pen_angle)
{
	mMuscles.push_back(std::make_shared<Muscle>());
	auto& muscle = mMuscles.back();

	muscle->name = name;
	muscle->waypoint_indices = waypoint_indices;
	muscle->blendedWaypoints.clear();
	for (int idx: waypoint_indices) muscle->blendedWaypoints.emplace_back(mBlendedWaypoints[idx]);

	//cout<<name<<"'s tendon length : "<<muscle->tendon_length<<endl;
	muscle->f_m_o = f_m_o;
	muscle->l_m_o = l_m_o * 1.75 / 168.4; // reference: 168.4 +- 9.3 cm, model: 1.75cm
//    muscle->l_m_o = muscle->getLength() * 0.95;
	muscle->l_t_sl = l_t_sl * 1.75 / 168.4;
	muscle->pen_angle = pen_angle * M_PI / 180.0;

	muscle->activation_level = 0.0;

	muscle->max_l_m_tilda = 2.0;
	muscle->max_l_m_tilda = 0.0;


//	cout << muscle->name << " :\t\t" << muscle->getLength() << " " << muscle->l_m_o << " ; " << muscle->l_t_sl << endl;

	muscle->initialize();
}
void
MusculoSkeletalSystem::
initialize(const dart::simulation::WorldPtr &rigid_world)
{

	mActivationLevels.resize(mMuscles.size());
	mActivationLevels.setZero();

	for (int i = 0; i < mSkeleton->getNumBodyNodes(); i++) {
		BodyNode *body = mSkeleton->getBodyNode(i);
		Eigen::Isometry3d parentJointT = body->getTransform() * body->getParentJoint()->getTransformFromChildBodyNode();

		Eigen::Vector3d jointX, jointY, jointZ;

		jointX = Eigen::Vector3d(1, 0, 0);
		jointY = Eigen::Vector3d(0, 1, 0);
		jointZ = Eigen::Vector3d(0, 0, 1);

		Eigen::Vector3d joint_to_body = body->getTransform().translation() - parentJointT.translation();
		joint_to_body.normalize();

		// find quaternion q s.t. q * jointY * q.inverse() equals to joint_to_body.
		Eigen::Vector3d axis = jointY.cross(joint_to_body);
		double angle = atan2(axis.norm(), jointY.dot(joint_to_body));

		Eigen::AngleAxisd angleAxis(angle, axis.normalized());
		Eigen::Quaterniond q(angleAxis);

		mCoordinateSystemAngleAxis.emplace_back(angleAxis);
	}

	rigid_world->addSkeleton(mSkeleton);
}
void
MusculoSkeletalSystem::
setActivationLevels(const Eigen::VectorXd &a)
{
	mActivationLevels = a;
	for(int i =0;i<mMuscles.size();i++)
		mMuscles[i]->setActivationLevel(a[i]);
}

//void
//MusculoSkeletalSystem::
//applyForcesToSkeletons(const dart::simulation::WorldPtr rigid_world)
//{
//	Eigen::VectorXd f = computeForce(rigid_world);
//	// std::cout<<f.transpose()<<std::endl;
//	int index = 0;
//	for(int i =0;i<mMuscles.size();i++)
//	{
//		auto& muscle = mMuscles[i];
//		auto& way_points = muscle->blendedWaypoints;
//
//		int n = muscle->getNumForces();
//		for (int i=0;i<n;i++)
//		{
//			way_points[i].ap.first->addExtForce(f.block<3,1>((index+i)*3,0),way_points[i].ap.second);
//		}
//		index += n;
//	}
//}
//void
//MusculoSkeletalSystem::
//computeForceDerivative(const dart::simulation::WorldPtr rigid_world,Eigen::SparseMatrix<double>& J)
//{
//	Eigen::VectorXd Ji(getNumMuscleForces()*3);
//	Ji.setZero();
//
//	Ji = computeForce(rigid_world);
//
//
//
//	std::vector<Eigen::Triplet<double>> j_triplets;
//
//	j_triplets.reserve(getNumMuscleForces()*3);
//	int index = 0;
//	for(int i =0;i<mMuscles.size();i++)
//	{
//		int n = mMuscles[i]->getNumForces();
//		double act = mMuscles[i]->activation_level;
//		for(int j=0;j<n;j++)
//		{
//			j_triplets.push_back(Eigen::Triplet<double>((index+j)*3+0,i,Ji[(index+j)*3+0]/act));
//			j_triplets.push_back(Eigen::Triplet<double>((index+j)*3+1,i,Ji[(index+j)*3+1]/act));
//			j_triplets.push_back(Eigen::Triplet<double>((index+j)*3+2,i,Ji[(index+j)*3+2]/act));
//		}
//		index +=n;
//	}
//
//	J.setFromTriplets(j_triplets.cbegin(), j_triplets.cend());
//}
double g_t_tilda(double eps_t){
    double f_toe_tilda = 0.33;
    double k_toe = 3.0;
    double eps_t_0 = 0.033;
    double eps_t_toe = 0.609 * eps_t_0;
    double k_lin = 1.712;
	return f_toe_tilda / (exp(k_toe) - 1) * (exp(k_toe * eps_t / eps_t_toe) - 1);

    /*if (eps_t <= eps_t_toe){
        return f_toe_tilda / (exp(k_toe) - 1) * (exp(k_toe * eps_t / eps_t_toe) - 1);
    }else{
        return k_lin * (eps_t - eps_t_toe) + f_toe_tilda;
    }*/
}

double Muscle::g_t(double l_t){
    return f_m_o * g_t_tilda((l_t - l_t_sl) / l_t_sl);
}

double Muscle::f_t(double l_t){
    return g_t(l_t);
}

double g_pl_tilda_o(double l_m_tilda){
    double k_pe = 4.0;
    double eps_m_o = 0.6;
    return (exp(k_pe * (l_m_tilda - 1) / eps_m_o) - 1) / (exp(k_pe) - 1);
}

double g_pl_tilda(double l_m_tilda){
    if (l_m_tilda <= 1.0) return 0;
    else return g_pl_tilda_o(l_m_tilda);
}

double Muscle::g_pl(double l_m) {
    return g_pl_tilda(l_m / l_m_o);
}

double Muscle::f_pl(double l_m){
    return f_m_o * g_pl(l_m) * cos(pen_angle)/1.0;
}

double g_al_tilda(double l_m_tilda) {
	if(l_m_tilda>0)
		return exp(-pow(l_m_tilda-1, 2) / gam);
	return 0;
}

double Muscle::g_al(double l_m) {
	return g_al_tilda(l_m / l_m_o);
}
double Muscle::f_al(double l_m) {
	return f_m_o * g_al(l_m) * cos(pen_angle);
}

double Muscle::compute_f_mt(){
    double l_mt = getLength();
    double l=0, r=l_mt, mid;
    while (r-l > 1e-6) {
		mid = (l + r) / 2.0;
		double res = f_pl(mid)+f_al(mid)*activation_level - f_t(l_mt - mid);
		if (res >= 0) r = mid;
		else l = mid;
	}
	l_m_tilda = mid / l_m_o;
	l_m = mid;
	l_t = l_mt - mid;
//    printf("f_pl(%s) : %lf\n",name.c_str(), f_t(l_mt - mid));
    return f_t(l_mt - mid);
}

// Should be called after comute_f_mt
// double Muscle::f_al()
// {
// 	if(l_m_tilda>0)
// 	{
// 		return f_m_o * exp(-(l_m_tilda - 1) * (l_m_tilda - 1)/0.5);
// 	}
// 	return 0;

// }

void
MusculoSkeletalSystem::
computeMuscleForce(std::shared_ptr<Muscle> muscle, double *f_tilda)
{
    *f_tilda = muscle->compute_f_mt();
    // cout<<*f_tilda<<endl;
   // muscle->compute_f_mt();
   //  *f_tilda += muscle->f_al(muscle->l_m);
}
void
MusculoSkeletalSystem::
computeMuscleForceActive(std::shared_ptr<Muscle> muscle, double *f_tilda)
{

   muscle->compute_f_mt();
   *f_tilda = muscle->f_al(muscle->l_m)*muscle->activation_level;
}
void
MusculoSkeletalSystem::
computeMuscleForcePassive(std::shared_ptr<Muscle> muscle, double *f_tilda)
{
 	muscle->compute_f_mt();
    *f_tilda = muscle->f_pl(muscle->l_m);
;
}
Eigen::VectorXd
MusculoSkeletalSystem::
computeForceActive(const dart::simulation::WorldPtr rigid_world)
{
	Eigen::VectorXd b(getNumMuscleForces()*3);
//	std::cout<<"compute force : "<<std::endl;

	int index = 0;
	for(int i=0;i<mMuscles.size();i++)
	{
		auto& muscle = mMuscles[i];
		double f_tilda = 0.0;
		computeMuscleForceActive(muscle, &f_tilda);
		
		int n = muscle->getNumForces();

		Eigen::VectorXd f(n*3);
		muscle->transferForce(f_tilda,f);

		b.block(index*3,0,f.rows(),1) = f;
		index += n;
	}
//	std::cout<<b.transpose()<<std::endl;

	return b;
}
Eigen::VectorXd
MusculoSkeletalSystem::
computeForcePassive(const dart::simulation::WorldPtr rigid_world)
{
	Eigen::VectorXd b(getNumMuscleForces()*3);
//	std::cout<<"compute force : "<<std::endl;

	int index = 0;
	for(int i=0;i<mMuscles.size();i++)
	{
		auto& muscle = mMuscles[i];
		double f_tilda = 0.0;
		computeMuscleForcePassive(muscle, &f_tilda);
		
		int n = muscle->getNumForces();

		Eigen::VectorXd f(n*3);
		muscle->transferForce(f_tilda,f);

		b.block(index*3,0,f.rows(),1) = f;
		index += n;
	}
//	std::cout<<b.transpose()<<std::endl;

	return b;
}
Eigen::VectorXd
MusculoSkeletalSystem::
updateForceActive(Eigen::VectorXd force, int muscleIndex)
{
	vector<Eigen::VectorXd> result;

	int index =0;
	for(int i=0;i<mMuscles.size();i++)
	{
		auto& muscle = mMuscles[i];
		int n = muscle->getNumForces();
		if(i == muscleIndex)
		{
			double f_tilda = 0.0;
			computeMuscleForceActive(muscle, &f_tilda);
			Eigen::VectorXd f(n*3);
			muscle->transferForce(f_tilda,f);
			force.block(index*3, 0, f.rows(),1) = f;

			return force;
		}
		index += n;
	}
	cout<<"updateForceActive : wrong muscle index "<<muscleIndex<<endl;
	return Eigen::Vector3d::Zero();
}
Eigen::VectorXd
MusculoSkeletalSystem::
updateForcePassive(Eigen::VectorXd force, int muscleIndex)
{
	vector<Eigen::VectorXd> result;

	int index =0;
	for(int i=0;i<mMuscles.size();i++)
	{
		auto& muscle = mMuscles[i];
		int n = muscle->getNumForces();
		if(i == muscleIndex)
		{
			double f_tilda = 0.0;
			computeMuscleForcePassive(muscle, &f_tilda);
			Eigen::VectorXd f(n*3);
			muscle->transferForce(f_tilda,f);
			force.block(index*3, 0, f.rows(),1) = f;

			return force;
		}
		index += n;
	}
	cout<<"updateForceActive : wrong muscle index "<<muscleIndex<<endl;
	return Eigen::Vector3d::Zero();
}
int
MusculoSkeletalSystem::
getNumMuscleForces()
{
	int n=0;
	for(auto muscle : mMuscles)
		n += muscle->getNumForces();
	return n;
}

Eigen::MatrixXd MusculoSkeletalSystem::getJacobianTranspose() {
   Eigen::MatrixXd Jt(mSkeleton->getNumDofs(),getNumMuscleForces() * 3);
   for (int i=0,col=0;i<getNumMuscles();i++){
		Jt.block(0,col,mSkeleton->getNumDofs(), 3* mMuscles[i]->getNumForces()) = mMuscles[i]->getJacobianTranspose();
		col+= 3 * mMuscles[i]->getNumForces();
   }
   return Jt;
}

void
MusculoSkeletalSystem::
ApplyAdjustmentValue(const char *name)
{
	std::string filePath = "../generator/motionAdjust/adjustvalue/adjustment_";
    filePath = filePath + name + ".txt";
	string line;
	stringstream s;
	ifstream in(filePath);
    // std::cout<<"# reading file "<<filePath<<std::endl;
	string muscleName;
	float value;
	while(!in.eof())
	{
		getline(in, line);
		if(line.empty())
			break;
		s = stringstream(line);
		s>>muscleName >> value;
		for(int i =0;i< getNumMuscles();i++)
		{
			if(getMuscles()[i]->name == muscleName)
			{
				getMuscles()[i]->l_m_o *= value;
				getMuscles()[i]->l_t_sl *= value;
			}
		}
	}
	in.close();
}
void
MusculoSkeletalSystem::
ApplyAdjustmentValue_FiberTendon(const char *name)
{
	std::string filePath = "../generator/motionAdjust/adjustvalue/adjustment_";
    filePath = filePath + name + ".txt";
	string line;
	stringstream s;
	ifstream in(filePath);
    std::cout<<"# reading file "<<filePath<<std::endl;
	string muscleName;
	float a_value, b_value;
	while(!in.eof())
	{
		getline(in, line);
		if(line.empty())
			break;
		s = stringstream(line);
		s>>muscleName >> a_value >> b_value;
		for(int i =0;i< getNumMuscles();i++)
		{
			if(getMuscles()[i]->name == muscleName)
			{
				getMuscles()[i]->l_m_o *= a_value * b_value;
				getMuscles()[i]->l_t_sl *= a_value;
			}
		}
	}
	in.close();
}


void MusculoSkeletalSystem::updateSingleWaypoints(int waypointIdx) {
	double saveConfig[mSkeleton->getNumDofs()];
	for (int i=0;i<mSkeleton->getNumDofs();i++){
		saveConfig[i] = mSkeleton->getPosition(i);
		mSkeleton->setPosition(i, 0);
	}

	calculateSingleBlendingWeights(waypointIdx);
	updateBlendedWaypoints();

	for (int i=0;i<mSkeleton->getNumDofs();i++){
		mSkeleton->setPosition(i, saveConfig[i]);
	}
}

void MusculoSkeletalSystem::updateTotalWaypoints() {
	double saveConfig[mSkeleton->getNumDofs()];
	for (int i=0;i<mSkeleton->getNumDofs();i++){
		saveConfig[i] = mSkeleton->getPosition(i);
		mSkeleton->setPosition(i, 0);
	}

	calculateTotalBlendingWeights();
	updateBlendedWaypoints();

	for (int i=0;i<mSkeleton->getNumDofs();i++){
		mSkeleton->setPosition(i, saveConfig[i]);
	}
}


void MusculoSkeletalSystem::updateBlendedWaypoints() {
	for (auto& muscle: mMuscles){
		for (int i=0;i<muscle->blendedWaypoints.size();i++){
			muscle->blendedWaypoints[i] = mBlendedWaypoints[muscle->waypoint_indices[i]];
		}
	}
}

std::vector<int> sort_indices(const std::vector<double>& val)
{
	std::vector<int> idx(val.size());
	std::iota(idx.begin(),idx.end(),0);

	std::sort(idx.begin(),idx.end(),[&val](int i1,int i2){return val[i1]<val[i2];});

	return idx;
}

void MusculoSkeletalSystem::calculateSingleBlendingWeights(int bwpIdx){
	auto& bwp = mBlendedWaypoints[bwpIdx];

	bwp.relatedBodyNodes.clear();
	bwp.relativeCoords.clear();
	bwp.weights.clear();

	bwp.ap = mWaypoints[bwpIdx];

	Eigen::Vector3d globalCoord = getAnchorPointGlobalCoord(bwp.ap);

	if (originFlag[bwpIdx] || insertionFlag[bwpIdx]){
//		if (true){
//		for (int j=0;j<relationN;j++){
//			if (bwp.ap.first == bwp.relatedBodyNodes[j]) bwp.relativeCoords[j] = bwp.ap.second, bwp.weights[j]=1.0;
//			else bwp.relativeCoords[j]=Eigen::Vector3d(0,0,0),bwp.weights[j] = 0;
//		}
		bwp.relatedBodyNodes.emplace_back(bwp.ap.first);
		bwp.relativeCoords.emplace_back(bwp.ap.second);
		bwp.weights.emplace_back(1.0);
	}
	else{
		std::vector<double> distance;
		std::vector<Eigen::Vector3d> local_positions;
		distance.resize(mSkeleton->getNumBodyNodes(),0.0);
		local_positions.resize(mSkeleton->getNumBodyNodes());
		for(int i =0;i<mSkeleton->getNumBodyNodes();i++)
		{
			Eigen::Isometry3d T;
			T = mSkeleton->getBodyNode(i)->getTransform()*mSkeleton->getBodyNode(i)->getParentJoint()->getTransformFromChildBodyNode();
			// local_positions[bwpIdx] = glob_pos-skel->getBodyNode(bwpIdx)->getTransform().translation();
			// local_positions[bwpIdx] = glob_pos-skel->getBodyNode(bwpIdx)->getTransform().translation();
			local_positions[i] = mSkeleton->getBodyNode(i)->getTransform().inverse() * globalCoord;
			distance[i] = (globalCoord - T.translation()).norm();
			// distance[bwpIdx] = local_positions[bwpIdx].norm();
			// std::cout<<skel->getBodyNode(i)->getName()<<" "<<distance[i]<<std::endl;
		}
		std::vector<int> index_sort_by_distance = sort_indices(distance);

		double total_weight = 0.0;
		double threshold = 0.08;
//		if (bwpIdx >= 436 && bwpIdx <= 527) threshold = 0.15;
		if(distance[index_sort_by_distance[0]]<threshold)
		{
			bwp.weights.push_back(1.0/sqrt(distance[index_sort_by_distance[0]]));
			total_weight += bwp.weights[0];
			bwp.relatedBodyNodes.push_back(mSkeleton->getBodyNode(index_sort_by_distance[0]));
			bwp.relativeCoords.push_back(local_positions[index_sort_by_distance[0]]);

			if(bwp.relatedBodyNodes[0]->getParentBodyNode()!=nullptr)
			{
				auto bn_parent = bwp.relatedBodyNodes[0]->getParentBodyNode();
				bwp.weights.push_back(1.0/sqrt(distance[bn_parent->getIndexInSkeleton()]));
				total_weight += bwp.weights[1];
				bwp.relatedBodyNodes.push_back(bn_parent);
				bwp.relativeCoords.push_back(local_positions[bn_parent->getIndexInSkeleton()]);
			}

			for(int i = 0;i < bwp.weights.size();i++){

				bwp.weights[i] /= total_weight;
			}
		}
		else
		{
			bwp.relatedBodyNodes.emplace_back(bwp.ap.first);
			bwp.relativeCoords.emplace_back(bwp.ap.second);
			bwp.weights.emplace_back(1.0);
		}
	}
}

void MusculoSkeletalSystem::calculateTotalBlendingWeights() {

	// currently, each blendedWaypoints contains related Bodynodes.

	// calculate local coord and weights
	double saveConfig[mSkeleton->getNumDofs()];
	for (int i=0;i<mSkeleton->getNumDofs();i++){
		saveConfig[i] = mSkeleton->getPosition(i);
		mSkeleton->setPosition(i, 0);
	}
	for (int i=0;i<mBlendedWaypoints.size();i++){
		calculateSingleBlendingWeights(i);
	}
	for (int i=0;i<mSkeleton->getNumDofs();i++){
		mSkeleton->setPosition(i, saveConfig[i]);
	}
}

void MusculoSkeletalSystem::setPose(double *config){
	for (int i=0;i<mSkeleton->getNumDofs();i++){
		mSkeleton->setPosition(i, config[i] * M_PI / 180);
	}
}

void MusculoSkeletalSystem::saveMuscleXML(std::string fileName,double scale, Eigen::Vector3d _p) {

	printf("Saving Current Retargeted Muscle XML ------- ");
	double saveConfig[mSkeleton->getNumDofs()];
	for (int i=0;i<mSkeleton->getNumDofs();i++){
		saveConfig[i] = mSkeleton->getPosition(i);
		mSkeleton->setPosition(i, 0);
	}

	string filePath = string(MSS_ROOT_DIR) + "/model/standard/muscle/" + fileName;
	TiXmlDocument doc;
	TiXmlElement* title = new TiXmlElement("Muscle");
	doc.LinkEndChild(title);
	for (int idx=0;idx<mWaypoints.size();idx++){
		auto& bwp = mBlendedWaypoints[idx];
		auto& wp = mWaypoints[idx];
		TiXmlElement* waypoint = new TiXmlElement("Waypoint");
		waypoint->SetAttribute("body", bwp.ap.first->getName());
		auto T = bwp.ap.first->getTransform();
		Eigen::Vector3d body_global_coord;
		body_global_coord = T * wp.second;
		waypoint->SetAttribute("p", toString(body_global_coord).c_str());

		waypoint->SetAttribute("idx", to_string(idx).c_str());

//		title->LinkEndChild(waypoint);
	}

	for (auto& muscle : mMuscles){
		TiXmlElement* unit = new TiXmlElement("Unit");
		unit->SetAttribute("name", muscle->name);

		double l_m_o = muscle->l_m_o;
		double l_t_sl = muscle->l_t_sl;
		double sum = l_m_o + l_t_sl;
		l_m_o = l_m_o / sum * 1.2;
		l_t_sl = l_t_sl / sum * 1.2;
		unit->SetAttribute("f0", to_string(muscle->f_m_o));
		unit->SetAttribute("lm", to_string(l_m_o));
		unit->SetAttribute("lt", to_string(l_t_sl));
		unit->SetAttribute("pen_angle", to_string(muscle->pen_angle));
//		unit->SetAttribute("indices", toString(muscle->waypoint_indices));

		for (int idx=0;idx<muscle->waypoint_indices.size();idx++){
			auto& bwp = mBlendedWaypoints[muscle->waypoint_indices[idx]];
			auto& wp = mWaypoints[muscle->waypoint_indices[idx]];
			TiXmlElement* waypoint = new TiXmlElement("Waypoint");
			waypoint->SetAttribute("body", bwp.ap.first->getName());
			auto T = bwp.ap.first->getTransform();
			Eigen::Vector3d body_global_coord;
			body_global_coord = T * wp.second;
			if (scale > 0){
				body_global_coord = _p + (body_global_coord - _p) * scale;
			}
			waypoint->SetAttribute("p", toString(body_global_coord).c_str());

			waypoint->SetAttribute("idx", to_string(idx).c_str());

			unit->LinkEndChild(waypoint);
		}

		title->LinkEndChild(unit);
	}


	TiXmlPrinter printer;
	printer.SetIndent( "\n" );

	doc.Accept( &printer );
	doc.SaveFile(filePath);

	for (int i=0;i<mSkeleton->getNumDofs();i++){
		mSkeleton->setPosition(i, saveConfig[i]);
	}
	printf("%s Success\n", fileName.c_str());

	doc.Clear();
}



bool
MusculoSkeletalSystem::
checkPosAvailable()
{
	// SetZeroPose();
	int numMuscles =getNumMuscles();
	for(int i=0;i<numMuscles;i++)
	{
		if (mMuscles[i]->name == "R_Psoas_Major"){
//			cout<<mMuscles[i]->name<<" "<<mMuscles[i]->max_l_mt<<endl;
		}
		if(mMuscles[i]->getLength() > getMuscles()[i]->max_l_mt + 1e-4)
		{
			// return false;
			if((mMuscles[i]->name.find("Bicep_Brachii_Long_Head")==string::npos) &&
				(mMuscles[i]->name.find("Abductor_Pollicis_Longus")==string::npos) &&
				(mMuscles[i]->name.find("Extensor_Pollicis_Longus")==string::npos) &&
				(mMuscles[i]->name.find("Extensor_Digiti_Minimi")==string::npos) &&
				(mMuscles[i]->name.find("Extensor_Digitorum1")==string::npos))
			{
				// cout<<mMuscles[i]->name<<endl;	
				return false;
			}

			// cout<<"IsValid check"<<endl;
			 // cout<<mMuscles[i]->name<<endl;
			//  cout<<mMuscles[i]->getLength() <<", "<< mMuscles[i]->max_l_mt<<", "<<mMuscles[i]->l_m_o<<endl;
		}
	}


//
//	for (int i=0;i<mSkeleton->getNumBodyNodes();i++){
//		BodyNode* bn = mSkeleton->getBodyNode(i);
//		int jointDof = bn->getParentJoint()->getNumDofs();
//		for (int j=0;j<jointDof;j++){
//			if (!(bn->getParentJoint()->getPositionLowerLimit(j) <= bn->getParentJoint()->getPosition(j) &&
//				bn->getParentJoint()->getPositionUpperLimit(j) >= bn->getParentJoint()->getPosition(j))) {
//				cout << "JOINT LIMIT OVER!! " << bn->getName() << endl;
//				return false;
//			}
//		}
//	}

	// ComputeForce();

	// for(auto& muscle : mMuscles)
	// {
	// 	if(muscle->getLmtilda() > 1.5)
	// 	{
	// 		// cout<<mMuscles[15]->name<<endl;
	// 		cout<<muscle->name<<endl;
	// 		// cout<<muscle->l_m_tilda<<" "<<muscle->max_l_m_tilda<<endl;
	// 		// cout<<endl;
	// 		return false;
	// 	}
	// }
	return true;
}
void
MusculoSkeletalSystem::
SetZeroPose()
{
	int numDofs = getSkeleton()->getNumDofs();
	Eigen::VectorXd pos(numDofs);
	pos.setZero();
	getSkeleton()->setPositions(pos);
}



int MusculoSkeletalSystem::getBodyNodeIndex(std::string body_name){
	for (int i=0;i<mSkeleton->getNumBodyNodes();i++) if (mSkeleton->getBodyNode(i)->getName() == body_name) return i;
	assert(false);
	return -1;

}
Eigen::Vector3d MusculoSkeletalSystem::getUAxis(std::string body_name) {
	int bnIdx = getBodyNodeIndex(body_name);
	BodyNode *body = mSkeleton->getBodyNode(bnIdx);
	Eigen::Isometry3d parentJointT = body->getTransform() * body->getParentJoint()->getTransformFromChildBodyNode();

	Eigen::Vector3d jointX;

	jointX = Eigen::Vector3d(1, 0, 0);

	Eigen::AngleAxisd q = mCoordinateSystemAngleAxis[bnIdx];

	jointX = parentJointT.linear() * q.toRotationMatrix() * jointX;

	return jointX;
}

Eigen::Vector3d MusculoSkeletalSystem::getVAxis(std::string body_name) {
	int bnIdx = getBodyNodeIndex(body_name);
	BodyNode *body = mSkeleton->getBodyNode(bnIdx);
	Eigen::Isometry3d parentJointT = body->getTransform() * body->getParentJoint()->getTransformFromChildBodyNode();

	Eigen::Vector3d jointZ;

	jointZ = Eigen::Vector3d(0, 0, 1);

	Eigen::AngleAxisd q = mCoordinateSystemAngleAxis[bnIdx];

	jointZ = parentJointT.linear() * q.toRotationMatrix() * jointZ;

	return jointZ;
}

Eigen::Vector3d MusculoSkeletalSystem::getJtoBAxis(std::string body_name) {
	int bnIdx = getBodyNodeIndex(body_name);
	BodyNode *body = mSkeleton->getBodyNode(bnIdx);
	Eigen::Isometry3d parentJointT = body->getTransform() * body->getParentJoint()->getTransformFromChildBodyNode();

	Eigen::Vector3d jointY;

	jointY = Eigen::Vector3d(0, 1, 0);

	Eigen::AngleAxisd q = mCoordinateSystemAngleAxis[bnIdx];

	jointY = parentJointT.linear() * q.toRotationMatrix() * jointY;

	return jointY;
}


void MusculoSkeletalSystem::updateMuscleParametersByMaxLmt()
{
	for(auto& muscle : mMuscles)
	{
		// cout<<muscle->name<<endl;
		// cout<<muscle->max_l_mt<<endl;
		// cout<<muscle->l_m_o<<endl;
		// cout<<muscle->l_t_sl<<endl;

		if(muscle->max_l_mt / muscle->min_l_mt -1 < 0.01)
		{
			muscle->l_m_o = muscle->max_l_mt * muscle->l_m_o/(muscle->l_m_o + muscle->l_t_sl);
			muscle->l_t_sl = muscle->max_l_mt * muscle->l_t_sl/(muscle->l_m_o + muscle->l_t_sl);
			continue;
		}
		double a = MuscleCal::compute_a_val(muscle, muscle->max_l_mt, true);

		// cout<<a<<endl;
		// cout<<endl;
		muscle->l_m_o *= a;
		muscle->l_t_sl *= a;

		if(muscle->name.find("Vastus")!=string::npos)
		{
			muscle->max_l_m_tilda = 1.4;
			a = MuscleCal::compute_a_val(muscle, muscle->max_l_mt, true);
			muscle->l_m_o *= a;
			muscle->l_t_sl *= a;
		}
		else if(muscle->name.find("Teres_major")!=string::npos)
		{
			muscle->max_l_m_tilda = 1.4;
			a = MuscleCal::compute_a_val(muscle, muscle->max_l_mt, true);
			muscle->l_m_o *= a;
			muscle->l_t_sl *= a;
		}
		else if(muscle->name.find("Latissimus_Dorsi")!=string::npos)
		{
			muscle->max_l_m_tilda = 1.4;
			a = MuscleCal::compute_a_val(muscle, muscle->max_l_mt, true);
			muscle->l_m_o *= a;
			muscle->l_t_sl *= a;
		}
		else if(muscle->name.find("Triceps_Long_Head")!=string::npos)
		{
			muscle->max_l_m_tilda = 1.4;
			a = MuscleCal::compute_a_val(muscle, muscle->max_l_mt, true);
			muscle->l_m_o *= a;
			muscle->l_t_sl *= a;
		}
	}
}

void MusculoSkeletalSystem::updateMuscleParametersByMaxLmtFast()
{
	for(auto& muscle : mMuscles)
	{

		if(muscle->max_l_mt / muscle->min_l_mt -1 < 1e-2)
		{
			muscle->l_m_o = muscle->max_l_mt * muscle->l_m_o/(muscle->l_m_o + muscle->l_t_sl);
			muscle->l_t_sl = muscle->max_l_mt * muscle->l_t_sl/(muscle->l_m_o + muscle->l_t_sl);
			continue;
		}
		//when l_m_tilda = 1.6, l_t_sl = 1.3
		double a = muscle->max_l_mt/(muscle->l_m_o*1.6+muscle->l_t_sl*1.03);
		muscle->l_m_o *= a;
		muscle->l_t_sl *= a;
	}
}

void MusculoSkeletalSystem::saveSkeletonText(double scale, Eigen::Vector3d _p) {

	FILE *out=fopen("skeleton.txt","w");

	TiXmlDocument doc;
	if(!doc.LoadFile("../model/rtgSkeleton.xml")){
		std::cout << "Can't open file : " << "../model/rtgSkeleton.xml" << std::endl;
		return;
	}

	TiXmlElement *skeldoc = doc.FirstChildElement("Skeleton");
//	std::cout << skelname;

	for(TiXmlElement *body = skeldoc->FirstChildElement("Joint"); body != nullptr; body = body->NextSiblingElement("Joint")){
		// name
		std::string name = body->Attribute("name");
		fprintf(out,"%s\n",name.c_str());
		// parent name
		std::string parentName = body->Attribute("parent_name");
		fprintf(out,"%s\n",parentName.c_str());

		// size
		Eigen::Vector3d size = string_to_vector3d(std::string(body->Attribute("size")));
		if (scale<0)
			fprintf(out,"%lf %lf %lf\n",size[0]*100, size[1]*100, size[2]*100);
		else {
			size = size * scale;
			fprintf(out,"%lf %lf %lf\n",size[0]*100, size[1]*100, size[2]*100);
		}
		// body position
		TiXmlElement *bodyPosElem = body->FirstChildElement("BodyPosition");
		Eigen::Isometry3d bodyPosition;
		bodyPosition.setIdentity();
		bodyPosition.linear() = string_to_matrix3d(bodyPosElem->Attribute("linear"));
		bodyPosition.translation() = string_to_vector3d(bodyPosElem->Attribute("translation")) * 100;
		bodyPosition = Orthonormalize(bodyPosition);

		if (scale>0){
			bodyPosition.translation() = _p + (bodyPosition.translation() - _p) * scale;
		}

		for (int i=0;i<16;i++){
			fprintf(out,"%lf ",bodyPosition.data()[i]);
		}
		fprintf(out,"\n");

		// joint position
		TiXmlElement *jointPosElem = body->FirstChildElement("JointPosition");
		Eigen::Isometry3d jointPosition;
		jointPosition.setIdentity();
		if(jointPosElem->Attribute("linear")!=nullptr)
			jointPosition.linear() = string_to_matrix3d(jointPosElem->Attribute("linear"));
		jointPosition.translation() = string_to_vector3d(jointPosElem->Attribute("translation")) * 100;
		jointPosition = Orthonormalize(jointPosition);

		if (scale>0){
			jointPosition.translation() = _p + (jointPosition.translation() - _p) * scale;
		}

		for (int i=0;i<3;i++){
			fprintf(out,"%lf ", jointPosition.translation()[i]);
		}
		fprintf(out,"\n");

	}
	fclose(out);
}

void MusculoSkeletalSystem::saveSkeletonXml(std::string filename, double scale, Eigen::Vector3d _p) {
	TiXmlDocument doc;
	if(!doc.LoadFile("../model/rtgSkeleton.xml")){
		std::cout << "Can't open file : " << "../model/rtgSkeleton.xml" << std::endl;
		return;
	}

	TiXmlElement *skeldoc = doc.FirstChildElement("Skeleton");

	std::string skelname = skeldoc->Attribute("name");
	SkeletonPtr skel = Skeleton::create(skelname);
//	std::cout << skelname;

	for(TiXmlElement *body = skeldoc->FirstChildElement("Joint"); body != nullptr; body = body->NextSiblingElement("Joint")){
		// type
		std::string jointType = body->Attribute("type");
		// name
		std::string name = body->Attribute("name");
		// parent name
		std::string parentName = body->Attribute("parent_name");
		std::string bvhName = body->Attribute("bvh");
		std::string objName = body->Attribute("obj");
		// size
		Eigen::Vector3d size = string_to_vector3d(std::string(body->Attribute("size")));
		// body position
		TiXmlElement *bodyPosElem = body->FirstChildElement("BodyPosition");
		Eigen::Isometry3d bodyPosition;
		bodyPosition.setIdentity();
		bodyPosition.linear() = string_to_matrix3d(bodyPosElem->Attribute("linear"));
		bodyPosition.translation() = string_to_vector3d(bodyPosElem->Attribute("translation"));
		bodyPosition = Orthonormalize(bodyPosition);
		// joint position
		TiXmlElement *jointPosElem = body->FirstChildElement("JointPosition");
		Eigen::Isometry3d jointPosition;
		jointPosition.setIdentity();
		if(jointPosElem->Attribute("linear")!=nullptr)
			jointPosition.linear() = string_to_matrix3d(jointPosElem->Attribute("linear"));
		jointPosition.translation() = string_to_vector3d(jointPosElem->Attribute("translation"));
		jointPosition = Orthonormalize(jointPosition);
		// mass
		double mass = atof(body->Attribute("mass"));

		if (scale > 0){
			size = size * scale;
			Eigen::Vector3d newBodyPos = _p + (bodyPosition.translation() - _p) * scale;
			Eigen::Vector3d newJointPos = _p + (jointPosition.translation() - _p) * scale;
//			mass = mass * pow(scale, 0.5);

			body->SetAttribute("size", toString(size));
			bodyPosElem->SetAttribute("translation", toString(newBodyPos));
			jointPosElem->SetAttribute("translation", toString(newJointPos));

		}

	}
	TiXmlPrinter printer;
	printer.SetIndent( "\n" );

	doc.Accept( &printer );
	doc.SaveFile("../model/"+filename);
}


void MusculoSkeletalSystem::applyForces(Eigen::VectorXd& activationLevels){
	setActivationLevels(activationLevels);
	for (auto& m: mMuscles){
		m->applyForce();
	}
}
void MusculoSkeletalSystem::applyForces(){
	for (auto& m: mMuscles){
		m->applyForce();
	}
}

void
Muscle::
applyForce() {
	double f = compute_f_mt();
//	cout << "force : " << f << endl;
	for (int i = 0; i < blendedWaypoints.size() - 1; i++) {
		Eigen::Vector3d dir = getBlendedWaypointGlobalCoord(blendedWaypoints[i + 1]) -
							  getBlendedWaypointGlobalCoord(blendedWaypoints[i]);
		dir.normalize();
		dir = f * dir;
		blendedWaypoints[i].ap.first->addExtForce(dir, getBlendedWaypointGlobalCoord(blendedWaypoints[i]), false,
												  false);
	}
	for (int i = 1; i < blendedWaypoints.size(); i++) {
		Eigen::Vector3d dir = getBlendedWaypointGlobalCoord(blendedWaypoints[i - 1]) -
							  getBlendedWaypointGlobalCoord(blendedWaypoints[i]);
		dir.normalize();
		dir = f * dir;
		blendedWaypoints[i].ap.first->addExtForce(dir, getBlendedWaypointGlobalCoord(blendedWaypoints[i]), false,
												  false);
	}
}

// void Muscle::releaseMuscleParameterByPassiveTorque()
// {
// 	for(int index=0, j=0;j<ms->getNumMuscles();j++)
//    	{
// 		int muscleNumForces = ms->getMuscles()[j]->getNumForces();
// 		if(applyPassive && abs((muscleJacobianTranspose.block(dofIndex, index*3, 
// 		1, muscleNumForces*3) * muscleForcePassive.segment(index*3,muscleNumForces*3))[0])>1e-2)
//    		{
//    			dof_torque_passive += (muscleJacobianTranspose.block(dofIndex, index*3, 
//     		1, muscleNumForces*3) * muscleForcePassive.segment(index*3,muscleNumForces*3))[0];

//     		cout<<"Passive : "<<ms->getMuscles()[j]->name<<
//     		" "<<(muscleJacobianTranspose.block(dofIndex, index*3, 
//     		1, muscleNumForces*3) * muscleForcePassive.segment(index*3,muscleNumForces*3))[0]<<endl;

// 			}
//    		index += muscleNumForces;
//    	}


	// // int dofIndex = skel->getIndexOf(skel->getBodyNode(curDof)->getParentJoint()->getDof(0));
	// auto muscleForceActive =              ms->computeForceActive(mWorld->GetRigidWorld());
	// auto muscleForcePassive =              ms->computeForcePassive(mWorld->GetRigidWorld());
 //    auto muscleJacobianTranspose =  ms->getJacobianTranspose();

 //    double dof_torque_passive = 0.0;


 //    bool applyPassive= true;
 //    bool activeForcePositive = true;

 //    vector<int> relativeMuscleIndex;
 //   	// cout<<"------------------------------------"<<endl;
 //   	for(int index=0, j=0;j<ms->getNumMuscles();j++)
 //   	{
	// 	int muscleNumForces = ms->getMuscles()[j]->getNumForces();
	// 	if(applyPassive && abs((muscleJacobianTranspose.block(dofIndex, index*3, 
	// 	1, muscleNumForces*3) * muscleForcePassive.segment(index*3,muscleNumForces*3))[0])>1e-2)
 //   		{
 //   			dof_torque_passive += (muscleJacobianTranspose.block(dofIndex, index*3, 
 //    		1, muscleNumForces*3) * muscleForcePassive.segment(index*3,muscleNumForces*3))[0];

 //    		cout<<"Passive : "<<ms->getMuscles()[j]->name<<
 //    		" "<<(muscleJacobianTranspose.block(dofIndex, index*3, 
 //    		1, muscleNumForces*3) * muscleForcePassive.segment(index*3,muscleNumForces*3))[0]<<endl;

	// 		}
 //   		index += muscleNumForces;
 //   	}
// }

void MusculoSkeletalSystem::releaseMuscleParametersByPassiveTorque()
{
	Eigen::MatrixXd muscleJacobianTranspose = getJacobianTranspose();
	for(int dofIndex=0;dofIndex<getSkeleton()->getNumDofs();dofIndex++)
	{
		for(int i =0;i<mMuscles.size();i++)
		{
			int forceIndex = getForcesIndex(i);
			int numForces = mMuscles[i]->getNumForces();
			double muscleForce;
			computeMuscleForcePassive(mMuscles[i], &muscleForce);
			Eigen::VectorXd transferedForce(numForces*3);
			mMuscles[i]->transferForce(muscleForce,transferedForce);
			double passiveTorque = (muscleJacobianTranspose.block(dofIndex, forceIndex*3, 
	    		1, numForces*3) * transferedForce)[0];
			while(abs(passiveTorque)>3.0)
			{
				mMuscles[i]->l_m_o *= 1.01;
				mMuscles[i]->l_t_sl *= 1.01;
				computeMuscleForcePassive(mMuscles[i], &muscleForce);
				mMuscles[i]->transferForce(muscleForce,transferedForce);
				passiveTorque = (muscleJacobianTranspose.block(dofIndex, forceIndex*3, 
	    		1, numForces*3) * transferedForce)[0];
	    		cout<<mMuscles[i]->name<<" "<<passiveTorque<<" ";
			}
		}
	}
}


int MusculoSkeletalSystem::getForcesIndex(int muscleIndex)
{
	int index = 0;
	for(int j=0;j<muscleIndex;j++)
   	{
		int muscleNumForces = mMuscles[j]->getNumForces();
   		index += muscleNumForces;
   	}
   	return index;
}

// int MusculoSkeletalSystem::getNumForcesIndex(std::shared_ptr<Muscle> muscle)
// {
// 	int index = 0;
// 	int muscleIndex = 
// 	for(int j=0;j<muscleIndex;j++)
//    	{
// 		int muscleNumForces = mMuscles[j]->getNumForces();
//    		index += muscleNumForces;
//    	}
//    	return index;
// }
