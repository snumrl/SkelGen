#include "Environment.h"
#include "SkeletonBuilder.h"
#include <initializer_list>
#include <chrono>
#include <IpIpoptApplication.hpp>
using namespace Ipopt;
double scaleParmEnv = 0.7;
double scaleIndexEnv  = 0;
double scaleCoeffEnv  = (scaleParmEnv*scaleIndexEnv+1.0*(3.0-scaleIndexEnv))/3.0;
// double scaleCoeffEnv = 2.5;
namespace MSS
{

Environment::
Environment(int control_Hz,int simulation_Hz)
	:mControlHz(control_Hz),mSimulationHz(simulation_Hz),mWorld(std::make_shared<dart::simulation::World>()),w_p(0.4),w_v(0.1),w_ee(0.2),w_com(0.3),mSimCount(1),mRandomSampleIndex(-1),mCurrentRootPosition(Eigen::Vector3d::Zero()),mAverageVelocity(Eigen::Vector3d::Zero()),mGoal(Eigen::VectorXd::Zero(0))
{
	mWorld->setGravity(Eigen::Vector3d(0,-9.81,0));
	mWorld->setTimeStep(1.0/(double)mSimulationHz);
	
	mWorld->getConstraintSolver()->setCollisionDetector(dart::collision::DARTCollisionDetector::create());
	mGround = MSS::SkeletonBuilder::BuildFromFile(std::string(MSS_ROOT_DIR)+std::string("/model/ground_real.xml"));
	
	mWorld->addSkeleton(mGround);
	mCharacter = new Character(mWorld,std::string(MSS_ROOT_DIR)+std::string("/model/skel.xml"));
	mWorld->addSkeleton(mCharacter->GetSkeleton());

	mCharacter->LoadMuscles(std::string(MSS_ROOT_DIR)+std::string("/model/muscle.xml"));
	double num_total_related_dofs = 0;
	for(auto m : mCharacter->GetMuscles()){
		m->Update();
		num_total_related_dofs += m->GetNumRelatedDofs();
	}
	mCurrentMuscleTuple.JtA = Eigen::VectorXd::Zero(num_total_related_dofs);
	mCurrentMuscleTuple.L = Eigen::MatrixXd::Zero(mCharacter->GetSkeleton()->getNumDofs()-6,mCharacter->GetMuscles().size());
	mCurrentMuscleTuple.b = Eigen::VectorXd::Zero(mCharacter->GetSkeleton()->getNumDofs()-6);
	mCurrentMuscleTuple.tau_des = Eigen::VectorXd::Zero(mCharacter->GetSkeleton()->getNumDofs()-6);

	mActivationLevels = Eigen::VectorXd::Zero(mCharacter->GetMuscles().size());
	mAverageActivationLevels = Eigen::VectorXd::Zero(mCharacter->GetMuscles().size());
	std::cout<<"# muscles : "<<mCharacter->GetMuscles().size()<<std::endl;
	std::cout<<"# muscles dofs : "<<num_total_related_dofs<<std::endl;
	mCharacter->LoadMotionGraph(std::string(MSS_ROOT_DIR)+std::string("/motion/simple.graph"),std::vector<int>{0,0,0,0,0},1.0/(double)mControlHz);
	
	mCharacter->AddInterestBodies(std::vector<std::string> {"FemurR","TibiaR","TalusR"});
	mCharacter->AddInterestBodies(std::vector<std::string> {"FemurL","TibiaL","TalusL"});
	mCharacter->AddInterestBodies(std::vector<std::string> {"Spine","Torso","Neck","Head"});
	mCharacter->AddInterestBodies(std::vector<std::string> {"ShoulderR","ArmR","ForeArmR","HandR"});
	mCharacter->AddInterestBodies(std::vector<std::string> {"ShoulderL","ArmL","ForeArmL","HandL"});
	
	// mCharacter->AddEndEffector("TalusR");
	// mCharacter->AddEndEffector("TalusL");
	mCharacter->AddEndEffector("FemurR");
	mCharacter->AddEndEffector("FemurL");
	mCharacter->AddEndEffector("HandR");
	mCharacter->AddEndEffector("HandL");
	mCharacter->AddEndEffector("Head");
	// mCharacter->AddEndEffector("Pelvis");
	

	for(int i=6;i<mCharacter->GetSkeleton()->getNumDofs();i++)
	{
		std::string name = mCharacter->GetSkeleton()->getDof(i)->getName();
		Eigen::VectorXd lower = Eigen::VectorXd::Zero(mCharacter->GetSkeleton()->getNumDofs());
		Eigen::VectorXd upper = Eigen::VectorXd::Zero(mCharacter->GetSkeleton()->getNumDofs());
		int idx = mCharacter->GetSkeleton()->getDof(i)->getIndexInSkeleton();
		lower[idx] = mCharacter->GetSkeleton()->getDof(i)->getPositionLowerLimit();
		upper[idx] = mCharacter->GetSkeleton()->getDof(i)->getPositionUpperLimit();
		// std::cout<<name<<lower.transpose()<<"\n"<<upper.transpose()<<std::endl<<std::endl;
		mCharacter->AddMotionAction(new MotionAction(name,lower,upper));
		// std::cout<<name<<std::endl;
	}

	mAction = Eigen::VectorXd::Zero(mCharacter->GetMotionActions().size());
	
	auto kpkv = mCharacter->GetKpKv(400.0);
	mCharacter->SetPDParameters(kpkv.first,kpkv.second);

	for(int i=0;i<mCharacter->GetMuscles().size();i++)
	{
		if(mCharacter->GetMuscles()[i]->l_mt_max>0.0)
		{
			mCharacter->GetMuscles()[i]->Update();
			mCharacter->AddMuscleLimitConstraint(std::make_shared<MuscleLimitConstraint>(mCharacter->GetMuscles()[i]));
		}
		
	}

	for(int i=0;i<mCharacter->GetMuscleLimitConstraints().size();i++)
		mWorld->getConstraintSolver()->addConstraint(mCharacter->GetMuscleLimitConstraints()[i]);

	InitializeMotions();
	
	Reset(false);
}
void
Environment::
InitializeMotions()
{
	mMotionCorrection = new MotionCorrection(mCharacter,mWorld);
	mMotionCorrectionSolver = new IpoptApplication();
	mMotionCorrectionSolver->Options()->SetStringValue("hessian_constant", "yes");
	mMotionCorrectionSolver->Options()->SetIntegerValue("print_level", 2);
	mMotionCorrectionSolver->Options()->SetIntegerValue("max_iter", 100);
	mMotionCorrectionSolver->Options()->SetNumericValue("tol", 1e-6);

	auto mg = mCharacter->GetMotionGraph();
	// mg->SetTimeStep(0.);
	Eigen::VectorXd current_motion = mg->GetMotion();
	mMotionModified.push_back(std::make_pair(current_motion,0.0));
	while(true)
	{
		
		if((current_motion-mg->GetMotion()).norm()>1E-6)
		{
			// std::cout<<(current_motion-mg->GetMotion()).norm()<<std::endl;
			current_motion = mg->GetMotion();
			mMotionModified.push_back(std::make_pair(current_motion,mg->GetElapsedTime()));
		}
			
		mg->Step();
		if(mg->GetPhase()<0.0001)
			break;
	}
	for(int i =0;i<mMotionModified.size();i++){
		// static_cast<MotionCorrection*>(GetRawPtr(mMotionCorrection))->Setq0(mMotionModified[i].first);
		// mMotionCorrectionSolver->Initialize();
		// mMotionCorrectionSolver->OptimizeTNLP(mMotionCorrection);
		// Eigen::VectorXd sol = static_cast<MotionCorrection*>(GetRawPtr(mMotionCorrection))->GetSolution();
		// mMotionModified[i].first = sol;
		// mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);
		// std::cout<<mMotionModified[i].second<<" : "<<mMotionModified[i].first.transpose()<<std::endl;

	}
	// for(int i =0;i<mMotionModified.size();i++)
	// {
	// 	std::cout<<(mMotionModified[(i+1)%mMotionModified.size()].first-mMotionModified[i].first).tail(mCharacter->GetSkeleton()->getNumDofs()-6).norm()<<std::endl;
	// }
}
void
Environment::
Step()
{
	// Muscle

	int count = 0;
	for(auto muscle : mCharacter->GetMuscles())
	{
		muscle->activation = mActivationLevels[count++];
		muscle->Update();
		muscle->ApplyForceToBody();
	}


	// Torque
	// GetDesiredTorques();
	// mCharacter->GetSkeleton()->setForces(mTorqueDesired);


	
	
	
	// if(0)
	if(mSimCount == mRandomSampleIndex)
	{
		auto& skel = mCharacter->GetSkeleton();
		auto& muscles = mCharacter->GetMuscles();

		int n = skel->getNumDofs();
		int m = muscles.size();
		Eigen::MatrixXd JtA = Eigen::MatrixXd::Zero(n,m);
		Eigen::VectorXd Jtp = Eigen::VectorXd::Zero(n);

		for(int i=0;i<muscles.size();i++)
		{
			auto muscle = muscles[i];
			// muscle->Update();
			Eigen::MatrixXd Jt = muscle->GetJacobianTranspose();
			auto Ap = muscle->GetForceJacobianAndPassive();

			JtA.block(0,i,n,1) = Jt*Ap.first;
			Jtp += Jt*Ap.second;
		}

		mCurrentMuscleTuple.JtA = GetMuscleTorques();
		mCurrentMuscleTuple.L = JtA.block(6,0,n-6,m);
		mCurrentMuscleTuple.b = Jtp.segment(6,n-6);
		mCurrentMuscleTuple.tau_des = mTorqueDesired.tail(mTorqueDesired.rows()-6);
		mMuscleTuples.push_back(mCurrentMuscleTuple);
	}
	mWorld->step();
	mAverageVelocity += mCharacter->GetSkeleton()->getRootBodyNode()->getCOMLinearVelocity()/((double)(mSimulationHz/mControlHz));
	mSimCount++;
}
Eigen::VectorXd
Environment::
GetDesiredTorques()
{
	mTorqueDesired = mCharacter->GetSPDForces(mTarget.first, mTarget.second);
	return mTorqueDesired.tail(mTorqueDesired.rows()-6);
}
Eigen::VectorXd
Environment::
GetMuscleTorques()
{
	int index = 0;
	mCurrentMuscleTuple.JtA.setZero();
	for(auto muscle : mCharacter->GetMuscles())
	{
		muscle->Update();
		Eigen::VectorXd JtA_i = muscle->GetRelatedJtA();
		mCurrentMuscleTuple.JtA.segment(index,JtA_i.rows()) = JtA_i;
		index += JtA_i.rows();
	}
	
	return mCurrentMuscleTuple.JtA;
}
void
Environment::
Reset(bool random)
{
	mWorld->reset();

	mCharacter->GetSkeleton()->clearConstraintImpulses();
	mCharacter->GetSkeleton()->clearInternalForces();
	mCharacter->GetSkeleton()->clearExternalForces();
	
	 
	if(random){
		mCurrentMotion = rand()%(mMotionModified.size());
		mTimeElapsed = mCurrentMotion/(double)mControlHz;
		mCurrentMotionScaled = mCurrentMotion;
	}
	else{
		mCurrentMotion = 0;
		mCurrentMotionScaled = mCurrentMotion;
		mTimeElapsed = 0.0;
	}


	
	// mCharacter->GetMotionGraph()->Reset(mTimeElapsed);
	mAction.setZero();
	mAverageVelocity.setZero();
	mCurrentRootPosition.setZero();
	// mTarget = mCharacter->GetTargetPositionsAndVelocitiesFromBVH();
	
	mTarget.first = mMotionModified[mCurrentMotion].first;
	mTarget.second = (mMotionModified[(mCurrentMotion+1)%mMotionModified.size()].first-mMotionModified[mCurrentMotion].first)/(mMotionModified[mCurrentMotion+1].second-mMotionModified[mCurrentMotion].second);
	mTarget.second.segment(3,3) *= scaleCoeffEnv;

	mCharacter->GetSkeleton()->setPositions(mTarget.first);
	mCharacter->GetSkeleton()->setVelocities(mTarget.second/sqrt(scaleCoeffEnv));
	mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);
	mGoal.setZero();
	// static_cast<MotionCorrection*>(GetRawPtr(mMotionCorrection))->Setq0(mTarget.first);
	// mMotionCorrectionSolver->Initialize();
	// mMotionCorrectionSolver->OptimizeTNLP(mMotionCorrection);
	// Eigen::VectorXd sol = static_cast<MotionCorrection*>(GetRawPtr(mMotionCorrection))->GetSolution();
	// mCharacter->GetSkeleton()->setPositions(sol);
	// mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);
}
bool
Environment::
IsTerminalState()
{
	bool isTerminal = false;

	Eigen::VectorXd p = mCharacter->GetSkeleton()->getPositions();
	Eigen::VectorXd v = mCharacter->GetSkeleton()->getVelocities();

	double root_y = mCharacter->GetSkeleton()->getBodyNode(0)->getTransform().translation()[1];
	Eigen::Vector3d root_rot = p.segment<3>(0);
	Eigen::Vector3d root_v = mCharacter->GetSkeleton()->getBodyNode(0)->getCOMLinearVelocity();


	auto& skel = mCharacter->GetSkeleton();	
	std::pair<Eigen::VectorXd,Eigen::VectorXd> target;// = mCharacter->GetTargetPositionsAndVelocitiesFromBVH();
	target.first = mMotionModified[mCurrentMotion].first;
	target.second = (mMotionModified[(mCurrentMotion+1)%mMotionModified.size()].first-mMotionModified[mCurrentMotion].first)/(mMotionModified[mCurrentMotion+1].second-mMotionModified[mCurrentMotion].second);
	Eigen::VectorXd cur_pos = skel->getPositions();
	Eigen::VectorXd cur_vel = skel->getVelocities();
	Eigen::Vector3d com_diff;
	com_diff = skel->getCOM();
	skel->setPositions(target.first);
	skel->setVelocities(target.second);
	skel->computeForwardKinematics(true,false,false);
	com_diff -= skel->getCOM();
	skel->setPositions(cur_pos);
	skel->setVelocities(cur_vel);
	skel->computeForwardKinematics(true,false,false);
	// std::cout<<root_y<<std::endl;

	//ET
	// std::cout<<root_y<<std::endl;
	if(root_y<0.6 || root_y > 2.0)
		isTerminal = true;
	if(dart::math::isNan(v)||v.array().abs().maxCoeff()>5E2)
		isTerminal = true;
	if(dart::math::isNan(p))
		isTerminal = true;
	if(mTimeElapsed>20.0)
		isTerminal = true;
	// if(mAverageVelocity[2]<-0.1)
	// 	isTerminal=true;
	// if(com_diff.norm() > 1.0)
	// 	isTerminal = true;
	// if(std::abs(p[1])>1.0)
		// isTerminal=true;
	return isTerminal;
}
Eigen::VectorXd
Environment::
GetState()
{
	auto& skel = mCharacter->GetSkeleton();
	dart::dynamics::BodyNode* root = skel->getBodyNode(0);
	auto ibs = mCharacter->GetInterestBodies();
	int num_body_nodes = ibs.size();
	Eigen::VectorXd p,v;

	p.resize(num_body_nodes*3);
	v.resize((num_body_nodes+1)*3);

	for(int i =0;i<num_body_nodes;i++)
	{
		p.segment<3>(3*i) = ibs[i]->getCOM(root);
		v.segment<3>(3*i) = ibs[i]->getCOMLinearVelocity();
	}
	v.tail<3>() = root->getCOMLinearVelocity();

	// double phi = mCharacter->GetMotionGraph()->GetPhase();
	double phi = (double)mCurrentMotion/(double)(mMotionModified.size());
	// std::cout<<phi<<std::endl;
	p*=0.8;
	v*=0.2;

	Eigen::VectorXd s(p.rows()+v.rows()+1+mGoal.rows());
	s<<p,v,phi,mGoal;

	return s;
}
double exp_of_squared(const Eigen::VectorXd& vec,double w = 1.0)
{
	return exp(-w*vec.squaredNorm());
}
double exp_of_squared(const Eigen::Vector3d& vec,double w = 1.0)
{
	return exp(-w*vec.squaredNorm());
}
double exp_of_squared(double val,double w = 1.0)
{
	return exp(-w*val*val);
}
double
Environment::
GetReward()
{
	auto& skel = mCharacter->GetSkeleton();
	Eigen::VectorXd zeros = Eigen::VectorXd::Zero(mCharacter->GetMotionActions().size());
	// auto target = mCharacter->GetTargetPositionsAndVelocitiesFromBVH();
	std::pair<Eigen::VectorXd,Eigen::VectorXd> target;// = mCharacter->GetTargetPositionsAndVelocitiesFromBVH();
	target.first = mMotionModified[mCurrentMotion].first;
	target.second = (mMotionModified[(mCurrentMotion+1)%mMotionModified.size()].first-mMotionModified[mCurrentMotion].first)/(mMotionModified[mCurrentMotion+1].second-mMotionModified[mCurrentMotion].second);

	double scaledCurrentMotion = (mCurrentMotionScaled/sqrt(scaleCoeffEnv));

	double ratio = scaledCurrentMotion - ((int) scaledCurrentMotion);

	int scaledCurrentMotionFront = (int) scaledCurrentMotion;
	int trucMotionIndex = scaledCurrentMotionFront%mMotionModified.size();
	int trucMotionIndex2 = (scaledCurrentMotionFront+1)%mMotionModified.size();

	target.first = mMotionModified[trucMotionIndex].first * (1-ratio) + mMotionModified[trucMotionIndex2].first * ratio;
	target.second = (mMotionModified[trucMotionIndex2].first - mMotionModified[trucMotionIndex].first)/
	(mMotionModified[trucMotionIndex2].second - mMotionModified[trucMotionIndex].second)/sqrt(scaleCoeffEnv);

	Eigen::VectorXd cur_pos = skel->getPositions();
	Eigen::VectorXd cur_vel = skel->getVelocities();

	// std::cout<<std::abs(cur_pos[1])<<std::endl;
	Eigen::VectorXd p_diff_all = skel->getPositionDifferences(target.first,cur_pos);
	Eigen::VectorXd v_diff_all = skel->getVelocityDifferences(target.second,cur_vel);
	
	Eigen::VectorXd p_diff = Eigen::VectorXd::Zero(skel->getNumDofs());
	Eigen::VectorXd v_diff = Eigen::VectorXd::Zero(skel->getNumDofs());
	
	auto& bvh_map = mCharacter->GetBVHMap();
	for(auto ss : bvh_map)
	{
		auto joint = mCharacter->GetSkeleton()->getBodyNode(ss.first)->getParentJoint();
		int idx = joint->getIndexInSkeleton(0);
		if(joint->getType()=="FreeJoint")
			continue;
		else if(joint->getType()=="RevoluteJoint")
			p_diff[idx] = p_diff_all[idx];
		else if(joint->getType()=="BallJoint")
			p_diff.segment<3>(idx) = p_diff_all.segment<3>(idx);	
	}

	for(auto ss : bvh_map)
	{
		auto joint = mCharacter->GetSkeleton()->getBodyNode(ss.first)->getParentJoint();
		int idx = joint->getIndexInSkeleton(0);
		if(joint->getType()=="FreeJoint")
			continue;
		else if(joint->getType()=="RevoluteJoint")
			v_diff[idx] = v_diff_all[idx];
		else if(joint->getType()=="BallJoint")
			v_diff.segment<3>(idx) = v_diff_all.segment<3>(idx);
	}
	auto ees = mCharacter->GetEndEffectors();
	Eigen::VectorXd ee_diff(ees.size()*3);
	Eigen::Vector3d com_diff;
	for(int i =0;i<ees.size();i++)
		ee_diff.segment<3>(i*3) = ees[i]->getCOM();
	com_diff = skel->getCOM();
	
	skel->setPositions(target.first);
	skel->setVelocities(target.second);
	skel->computeForwardKinematics(true,false,false);
	com_diff -= skel->getCOM();
	for(int i =0;i<ees.size();i++)
		ee_diff.segment<3>(i*3) -= ees[i]->getCOM()+com_diff;
	com_diff[1] = 0.0;
	com_diff[2] = 0.0;

	skel->setPositions(cur_pos);
	skel->setVelocities(cur_vel);
	skel->computeForwardKinematics(true,false,false);

	double r_p = exp_of_squared(p_diff,1.0);
	double r_v = exp_of_squared(v_diff,0.1);
	double r_ee = exp_of_squared(ee_diff,40.0);
	double r_com = exp_of_squared(com_diff,5.0);
	Eigen::Vector3d rootDir = skel->getPositions().segment(0,3);
	double rootOri = rootDir.norm();
	double r_root = exp_of_squared(rootOri, 5.0);

	// double r = r_ee*(w_p*r_p + w_v*r_v);
	// double r = w_ee*r_ee + w_p*r_p + w_v*r_v + 3*w_com*r_com;
	double r = r_ee*(w_p*r_p + w_v*r_v + w_com*r_com + w_com*r_root);
	// double r = r_ee*3.0*(w_p*r_p + w_v*r_v);
	
	// Eigen::Vector3d goal_diff = Eigen::Vector3d()
	// double radius = mGoal[0]*0.5+1.2;
	// double theta = mGoal[1]*3.141592/6.0;
	// Eigen::Vector3d target_direction = Eigen::Vector3d(sin(theta),0.0,cos(theta));
	// double projection = target_direction.dot(mAverageVelocity);
	// Eigen::Vector3d avg_velocity = mAverageVelocity;
	// avg_velocity[1] = 0.0;
	// avg_velocity.normalize();
	// double avg_theta = acos(avg_velocity[2]);
	// if(avg_velocity[0]<0.0)
	// 	avg_theta = -avg_theta;
	// std::cout<<avg_theta<<std::endl;
	// Eigen::Vector3d goal_diff = target_velocity - mAverageVelocity;

	// double r_goal = exp(-2.5*std::max(0.0,mGoal[0]*0.5+1.0-mAverageVelocity[2]));
	// double r_goal = exp_of_squared(target_direction-)

	// double r_goal = 0.5*exp(-2.5*std::max(0.0,radius - projection));
	// if(theta>0.0)
	// 	r_goal += 0.5*exp(-2.5*std::max(0.0,theta - avg_theta));
	// else
	// 	r_goal += 0.5*exp(-2.5*std::max(0.0,avg_theta - theta));	
	// + 0.5*exp(-0.5*std::max(0.0,radius - projection));
	// Eigen::Vector3d goal_diff = target_direction - mAverageVelocity;
	// double r_goal = exp_of_squared(goal_diff,0.01);
	// std::cout<<r_goal<<std::endl;
	// double r = 0.6*r_track + 0.4*r_goal;

	if(dart::math::isNan(r))
		return 0.0;
	return r;
}
void
Environment::
SetAction(const Eigen::VectorXd& a)
{
	// std::cout<<mCharacter->GetSkeleton()->getPositions().segment<3>(0).transpose()<<std::endl;
	mAction.head(mCharacter->GetMotionActions().size()) = 0.1*a.head(mCharacter->GetMotionActions().size());
	// std::cout<<mAction.transpose()<<std::endl;
	mTimeElapsed += 1.0 / (double)mControlHz;
	// mCharacter->GetMotionGraph()->Step();
	mAverageVelocity.setZero();	
	// = mCharacter->GetTargetPositionsAndVelocitiesFromBVH(mAction.head(mCharacter->GetMotionActions().size()));
	mTarget.first = mMotionModified[mCurrentMotion].first;
	mTarget.first.segment<3>(3) += mCurrentRootPosition;
	mTarget.second = (mMotionModified[(mCurrentMotion+1)%mMotionModified.size()].first-mMotionModified[mCurrentMotion].first)/(mMotionModified[mCurrentMotion+1].second-mMotionModified[mCurrentMotion].second);

	double scaledCurrentMotion = (mCurrentMotionScaled/sqrt(scaleCoeffEnv));

	double ratio = scaledCurrentMotion - ((int) scaledCurrentMotion);

	int scaledCurrentMotionFront = (int) scaledCurrentMotion;
	int trucMotionIndex = scaledCurrentMotionFront%mMotionModified.size();
	int trucMotionIndex2 = (scaledCurrentMotionFront+1)%mMotionModified.size();

	mTarget.first = mMotionModified[trucMotionIndex].first * (1-ratio) + mMotionModified[trucMotionIndex2].first * ratio;
	mTarget.second = (mMotionModified[trucMotionIndex2].first - mMotionModified[trucMotionIndex].first)/
	(mMotionModified[trucMotionIndex2].second - mMotionModified[trucMotionIndex].second)/sqrt(scaleCoeffEnv);




	for(int i =0;i<mCharacter->GetMotionActions().size();i++){
		mCharacter->GetMotionActions()[i]->SetLB(mAction[i]);
		mCharacter->GetMotionActions()[i]->Set(mTarget.first);
	}
	// std::cout<<mCurrentMotion%mMotionModified.size()<<mTimeElapsed<<std::endl;
	// if((mCurrentMotion+1)%mMotionModified.size() == 0 && mTimeElapsed>0.03){
	// 	mCurrentRootPosition += mMotionModified.back().first.segment<3>(3) - mMotionModified[0].first.segment<3>(3);
	// 	mCurrentRootPosition[1] = 0.0;
	// }




	// std::cout<<(mTarget.first-save).norm();
	mSimCount = 0;
	mRandomSampleIndex = rand()%(mSimulationHz/mControlHz);

	mAverageActivationLevels.setZero();

	mCurrentMotion++;
	mCurrentMotion%=mMotionModified.size();

	mCurrentMotionScaled++;
	if (mCurrentMotionScaled/sqrt(scaleCoeffEnv) >mMotionModified.size())
		mCurrentMotionScaled =0;

	if(mCurrentMotionScaled == 0 && mTimeElapsed>0.03){
		mCurrentRootPosition += mMotionModified.back().first.segment<3>(3) - mMotionModified[0].first.segment<3>(3);
		mCurrentRootPosition[1] = 0.0;
	}

	// static_cast<MotionCorrection*>(GetRawPtr(mMotionCorrection))->Setq0(mTarget.first);
	// mMotionCorrectionSolver->Initialize();
	// mMotionCorrectionSolver->OptimizeTNLP(mMotionCorrection);
	// Eigen::VectorXd sol = static_cast<MotionCorrection*>(GetRawPtr(mMotionCorrection))->GetSolution();
	// mCharacter->GetSkeleton()->setPositions(sol);
	
}
void
Environment::
SetGoal(const Eigen::VectorXd& goal)
{
	mGoal = goal;
}
Eigen::VectorXd
Environment::
GetGoal()
{
	return mGoal;
}
std::pair<Eigen::VectorXd,Eigen::VectorXd>
Environment::
GetTargetPositionAndVelocities(const Eigen::VectorXd& action)
{
	std::pair<Eigen::VectorXd,Eigen::VectorXd> target;
	target.first = mMotionModified[mCurrentMotion].first;
	target.second = (mMotionModified[(mCurrentMotion+1)%mMotionModified.size()].first-mMotionModified[mCurrentMotion].first)/(mMotionModified[mCurrentMotion+1].second-mMotionModified[mCurrentMotion].second);


	double scaledCurrentMotion = (mCurrentMotionScaled/sqrt(scaleCoeffEnv));

	double ratio = scaledCurrentMotion - ((int) scaledCurrentMotion);

	int scaledCurrentMotionFront = (int) scaledCurrentMotion;
	int trucMotionIndex = scaledCurrentMotionFront%mMotionModified.size();
	int trucMotionIndex2 = (scaledCurrentMotionFront+1)%mMotionModified.size();

	target.first = mMotionModified[trucMotionIndex].first * (1-ratio) + mMotionModified[trucMotionIndex2].first * ratio;
	target.second = (mMotionModified[trucMotionIndex2].first - mMotionModified[trucMotionIndex].first)/
	(mMotionModified[trucMotionIndex2].second - mMotionModified[trucMotionIndex].second)/sqrt(scaleCoeffEnv);


	for(int i =0;i<mCharacter->GetMotionActions().size();i++){
		mCharacter->GetMotionActions()[i]->SetLB(action[i]);
		mCharacter->GetMotionActions()[i]->Set(target.first);
	}
	target.first.segment<3>(3) += mCurrentRootPosition;
	target.first.segment<3>(3)*=sqrt(scaleCoeffEnv);

	return target;
}
}
