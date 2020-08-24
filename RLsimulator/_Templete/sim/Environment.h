#ifndef __MSS_ENVIRONMENT_H__
#define __MSS_ENVIRONMENT_H__
#include "Character.h"
#include "MotionCorrection.h"

namespace MSS
{
class Character;
struct MuscleTuple
{
	Eigen::VectorXd JtA;
	Eigen::MatrixXd L;
	Eigen::VectorXd b;
	Eigen::VectorXd tau_des;
};
class Environment
{
public:
	Environment(int control_Hz=30,int simulation_Hz=900);
	void Step();

	void Reset(bool random=true);
	bool IsTerminalState();
	void InitializeMotions();
	//For Deep RL
	Eigen::VectorXd GetState();
	double GetReward();
	Eigen::VectorXd GetAction(){return mAction;}
	void SetAction(const Eigen::VectorXd& a);
	void SetActivationLevels(const Eigen::VectorXd& a){mActivationLevels = a;}
	void SetGoal(const Eigen::VectorXd& goal);
	Eigen::VectorXd GetGoal();

	int GetNumState(){return GetState().rows();};
	int GetNumAction(){return mAction.rows();};
	const dart::simulation::WorldPtr& GetWorld(){return mWorld;};
	Character* GetCharacter(){return mCharacter;};
	const dart::dynamics::SkeletonPtr& GetGround(){return mGround;}	
	
	int GetControlHz(){return mControlHz;};
	int GetSimulationHz(){return mSimulationHz;};
	int GetNumTotalRelatedDofs(){return mCurrentMuscleTuple.JtA.rows();}
	std::vector<MuscleTuple>& GetMuscleTuples(){return mMuscleTuples;};
	Eigen::VectorXd GetDesiredTorques();
	Eigen::VectorXd GetMuscleTorques();
	double GetElapsedTime(){return mTimeElapsed;}
	Eigen::VectorXd GetActivationLevels(){return mActivationLevels;}
	Eigen::VectorXd GetAverageActivationLevels(){return mActivationLevels;}
	Eigen::VectorXd GetAverageVelocity(){return mAverageVelocity;}
	std::pair<Eigen::VectorXd,Eigen::VectorXd> GetTargetPositionAndVelocities(const Eigen::VectorXd& action);
public:
	dart::simulation::WorldPtr mWorld;
	double mTimeElapsed;
	int mControlHz;
	int mSimulationHz;

	dart::dynamics::SkeletonPtr mGround;
	Character* mCharacter;
	
	double w_p,w_v,w_ee,w_com;
	Eigen::VectorXd mAction;

	std::pair<Eigen::VectorXd,Eigen::VectorXd> mTarget;
	Eigen::VectorXd mTorqueDesired;
	Eigen::VectorXd mActivationLevels;
	Eigen::VectorXd mAverageActivationLevels;

	std::vector<MuscleTuple> mMuscleTuples;
	MuscleTuple mCurrentMuscleTuple;
	int mSimCount;
	int mRandomSampleIndex;

	Ipopt::SmartPtr<Ipopt::TNLP> 			 			mMotionCorrection;
	Ipopt::SmartPtr<Ipopt::IpoptApplication> 			mMotionCorrectionSolver;

	std::vector<std::pair<Eigen::VectorXd,double>>	mMotionModified;
	int mCurrentMotion;
	int mCurrentMotionScaled;

	Eigen::Vector3d mCurrentRootPosition;
	Eigen::Vector3d mAverageVelocity;
	Eigen::VectorXd mGoal;
};

};


#endif