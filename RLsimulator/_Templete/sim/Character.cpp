#include "Character.h"
#include "SkeletonBuilder.h"
#include <tinyxml.h>
using namespace dart::dynamics;

namespace MSS
{
Character::
Character(const dart::simulation::WorldPtr& world,const std::string& path)
	:mWorld(world)
{
	LoadSkeleton(path);
}
void
Character::
LoadSkeleton(const std::string& path)
{
	mSkeleton = MSS::SkeletonBuilder::BuildFromFile(path);
	//Load BVH map
	TiXmlDocument doc;
	if(!doc.LoadFile(path)){
		std::cout << "Can't open file : " << path << std::endl;
		return;
	}

	TiXmlElement *skeldoc = doc.FirstChildElement("Skeleton");
	
	std::string skelname = skeldoc->Attribute("name");

	for(TiXmlElement *body = skeldoc->FirstChildElement("Joint"); body != nullptr; body = body->NextSiblingElement("Joint")){
		// name
		std::string name = body->Attribute("name");

		// bvh
		if(body->Attribute("bvh")!=nullptr){
			mBVHMap.insert(std::make_pair(name,body->Attribute("bvh")));
		}
	}
}
void
Character::
LoadMuscles(const std::string& path)
{
	TiXmlDocument doc;
	if(!doc.LoadFile(path)){
		std::cout << "Can't open file : " << path << std::endl;
		return;
	}

	TiXmlElement *muscledoc = doc.FirstChildElement("Muscle");
	for(TiXmlElement* unit = muscledoc->FirstChildElement("Unit");unit!=nullptr;unit = unit->NextSiblingElement("Unit"))
	{
		std::string name = unit->Attribute("name");
		double f0 = std::stod(unit->Attribute("f0"));
		double lm = 1.0;//std::stod(unit->Attribute("lm"));
		double lt = 0.2;//std::stod(unit->Attribute("lt"));
		double pa = std::stod(unit->Attribute("pen_angle"));
		double lmax = -0.1;//std::stod(unit->Attribute("lmax"));

		mMuscles.push_back(new Muscle(name,f0,lm,lt,pa,lmax));
		int num_waypoints = 0;
		for(TiXmlElement* waypoint = unit->FirstChildElement("Waypoint");waypoint!=nullptr;waypoint = waypoint->NextSiblingElement("Waypoint"))	
			num_waypoints++;
		int i = 0;
		for(TiXmlElement* waypoint = unit->FirstChildElement("Waypoint");waypoint!=nullptr;waypoint = waypoint->NextSiblingElement("Waypoint"))	
		{
			std::string body = waypoint->Attribute("body");
			Eigen::Vector3d glob_pos = string_to_vector3d(waypoint->Attribute("p"));
			if(i==0||i==num_waypoints-1)
			// if(true)
				mMuscles.back()->AddAnchor(mSkeleton->getBodyNode(body),glob_pos);
			else
				mMuscles.back()->AddAnchor(mSkeleton,mSkeleton->getBodyNode(body),glob_pos,2);

			i++;
		}
	}
	

}
void
Character::
LoadMotionGraph(const std::string& path,const std::vector<int>& seq,double time_step)
{
	mMotionGraph = new MotionGraph(mSkeleton,mBVHMap,time_step);
	mMotionGraph->Parse(path);
	mMotionGraph->SetWalk(seq);
}

void
Character::
SetPDParameters(double kp, double kv)
{
	int dof = mSkeleton->getNumDofs();
	SetPDParameters(Eigen::VectorXd::Constant(dof, kp), Eigen::VectorXd::Constant(dof, kv));
}

void
Character::
SetPDParameters(const Eigen::VectorXd& kp, const Eigen::VectorXd& kv)
{
	this->mKp = kp;
	this->mKv = kv;
}
Eigen::VectorXd
Character::
GetSPDForces(const Eigen::VectorXd& p_desired, const Eigen::VectorXd& v_desired)
{
	auto& skel = mSkeleton;
	Eigen::VectorXd q = skel->getPositions();
	Eigen::VectorXd dq = skel->getVelocities();
	double dt = skel->getTimeStep();
	Eigen::MatrixXd M_inv = (skel->getMassMatrix() + Eigen::MatrixXd(dt*mKv.asDiagonal())).inverse();
		
	Eigen::VectorXd qdqdt = q + dq*dt;

	Eigen::VectorXd p_diff = -mKp.cwiseProduct(skel->getPositionDifferences(qdqdt,p_desired));

	Eigen::VectorXd v_diff = -mKv.cwiseProduct(dq);
	Eigen::VectorXd qddot = M_inv*(-skel->getCoriolisAndGravityForces()+
							p_diff+v_diff+skel->getConstraintForces());

	Eigen::VectorXd tau = p_diff + v_diff - dt*mKv.cwiseProduct(qddot);
	
	tau.head<6>().setZero();
	return tau;
}
Eigen::VectorXd
Character::
GetTargetPositions(const Eigen::VectorXd& mode_lb)
{
	int dof = mSkeleton->getPositions().rows();
	
	Eigen::VectorXd p = mMotionGraph->GetMotion();

	for(int i =0;i<mode_lb.rows();i++){
		mMotionActions[i]->SetLB(mode_lb[i]);
		mMotionActions[i]->Set(p);
	}

	return p;
}
std::pair<Eigen::VectorXd,Eigen::VectorXd>
Character::
GetTargetPositionsAndVelocitiesFromBVH(const Eigen::VectorXd& mode_lb)
{
	Eigen::VectorXd p = GetTargetPositions(mode_lb);
	mMotionGraph->SaveState();
	mMotionGraph->Step();
	Eigen::VectorXd p1 = GetTargetPositions(mode_lb);

	mMotionGraph->LoadState();
	int dof = mSkeleton->getPositions().rows();

	Eigen::VectorXd target_position = p;
	Eigen::VectorXd target_velocity = (p1-p)/mMotionGraph->GetTimeStep();

	Eigen::VectorXd current_position = mSkeleton->getPositions();
	Eigen::VectorXd current_velocity = mSkeleton->getVelocities();
	
	mSkeleton->setPositions(target_position);
	mSkeleton->setVelocities(target_velocity);
	dynamic_cast<dart::dynamics::FreeJoint*>(mSkeleton->getRootJoint())->setLinearVelocity(target_velocity.segment<3>(3));
	mSkeleton->computeForwardKinematics(true,false,false);
	target_velocity = mSkeleton->getVelocities();

	mSkeleton->setPositions(current_position);
	mSkeleton->setVelocities(current_velocity);
	mSkeleton->computeForwardKinematics(true,false,false);
	return std::make_pair(target_position,target_velocity);
}
std::pair<Eigen::VectorXd,Eigen::VectorXd>
Character::
GetKpKv(double default_val)
{
	int dof = mSkeleton->getPositions().rows();
	Eigen::VectorXd kp = Eigen::VectorXd::Constant(dof,default_val);
	Eigen::VectorXd kv = Eigen::VectorXd::Constant(dof,default_val);
	for(int i =0;i<dof;i++)
		kv[i] = sqrt(2*kp[i]);
	return std::make_pair(kp,kv);
}
};