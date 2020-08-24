#ifndef __MSS_CHARACTER_H__
#define __MSS_CHARACTER_H__
#include "MotionGraph.h"
#include "Action.h"
#include "Muscle.h"
#include "MuscleLimitConstraint.h"
#include "dart/dart.hpp"

#include <algorithm>
namespace MSS
{
class Character
{
public:
	Character(const dart::simulation::WorldPtr& world,const std::string& path);

	void LoadSkeleton(const std::string& path);
	void LoadMuscles(const std::string& path);
	void LoadMotionGraph(const std::string& path,const std::vector<int>& seq,double time_step);
	void InitializeMotionCorrection();

	void AddMotionAction(MotionAction* mode){mMotionActions.push_back(mode);};

	void AddInterestBody(const std::string& str){mInterestBodies.push_back(mSkeleton->getBodyNode(str));};
	void AddInterestBodies(const std::vector<std::string>& str_list){for(int i =0;i<str_list.size();i++) AddInterestBody(str_list[i]);};
	void AddEndEffector(const std::string& str){mEndEffectors.push_back(mSkeleton->getBodyNode(str));};
	void AddMuscleLimitConstraint(const std::shared_ptr<MuscleLimitConstraint>& cst){mMuscleLimitConstraints.push_back(cst);}

	void SetPDParameters(double kp, double kv);
	void SetPDParameters(const Eigen::VectorXd& kp, const Eigen::VectorXd& kv);

	Eigen::VectorXd GetTargetPositions(const Eigen::VectorXd& mode_lb=Eigen::VectorXd::Zero(0));
	
	std::pair<Eigen::VectorXd,Eigen::VectorXd> GetTargetPositionsAndVelocitiesFromBVH(const Eigen::VectorXd& mode_lb=Eigen::VectorXd::Zero(0));

	std::pair<Eigen::VectorXd,Eigen::VectorXd> GetKpKv(double default_val);
	Eigen::VectorXd GetSPDForces(const Eigen::VectorXd& p_desired, const Eigen::VectorXd& v_desired);
	
	const dart::dynamics::SkeletonPtr& GetSkeleton() {return mSkeleton;};
	const std::vector<Muscle*>& GetMuscles() {return mMuscles;};
	MotionGraph* GetMotionGraph(){return mMotionGraph;};
	const std::vector<dart::dynamics::BodyNode*>& GetInterestBodies() {return mInterestBodies;};
	const std::vector<dart::dynamics::BodyNode*>& GetEndEffectors() {return mEndEffectors;};
	const std::vector<MotionAction*>& GetMotionActions(){return mMotionActions;};
	const std::map<std::string,std::string>& GetBVHMap(){return mBVHMap;};
	const std::vector<std::shared_ptr<MuscleLimitConstraint>>& GetMuscleLimitConstraints(){return mMuscleLimitConstraints;};

	const Eigen::VectorXd& GetModifiedTargetPositionsAndVelocitiesFromBVH(const Eigen::VectorXd& mode_lb=Eigen::VectorXd::Zero(0));
public:
	dart::simulation::WorldPtr mWorld;
	dart::dynamics::SkeletonPtr mSkeleton;
	std::vector<Muscle*> mMuscles;
	std::map<std::string,std::string> mBVHMap;
	MotionGraph* mMotionGraph;
	
	Eigen::VectorXd mKp, mKv;
	std::vector<MotionAction*> mMotionActions;

	std::vector<dart::dynamics::BodyNode*> mEndEffectors;
	std::vector<dart::dynamics::BodyNode*> mInterestBodies;
	std::vector<std::shared_ptr<MuscleLimitConstraint>> mMuscleLimitConstraints;

	std::vector<Eigen::VectorXd> mModifiedTargetPositions;
};
};
#endif