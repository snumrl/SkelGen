#ifndef __MSS_MOTION_GRAPH_H__
#define __MSS_MOTION_GRAPH_H__
#include "dart/dart.hpp"
#include "BVH.h"
namespace MSS
{

class MotionGraph
{
public:
	MotionGraph(const dart::dynamics::SkeletonPtr& skel,std::map<std::string,std::string> bvh_map,double time_step);
	void Parse(const std::string& path);

	void Step();

	void Reset(double t);
	void SetWalk(const std::vector<int>& sequence);
	Eigen::VectorXd GetMotion();

	void SaveState();
	void LoadState();
	void SetTimeStep(double t){mTimeStep = t;}
	double GetMaxTimeOfFirstSeq() {return mBVHs[mSequences[0]]->GetMaxTime();};
	double GetMaxTime() {return mMaxTime;};
	double GetPhase() {return mElapsedTime/mBVHs[mSequences[mCurrentSequence]]->GetMaxTime();}
	double GetTimeStep(){return mTimeStep;}
	double GetElapsedTime(){return mElapsedTime;}
	
	Eigen::VectorXd ComputeMotionVector(int i,double t);
	BVH* GetBVH(){return mBVHs[0];}
private:
	dart::dynamics::SkeletonPtr mSkeleton;
	double mTotalElapsedTime;
	double mMaxTime;
	double mElapsedTime;
	double mTimeStep;
	int mCurrentSequence;
	std::vector<int> mSequences;
	std::vector<BVH*> mBVHs;
	std::vector<std::vector<int>> mTransitions;
	std::map<std::string,std::string> mBVHMap;

	Eigen::Vector6d mCurrentRootOffset;
	Eigen::VectorXd md;
	Eigen::Matrix3d mRtarget,mRy;
	double mt_s;
	static double ScaleFunc(double t){if(t<1.0) return (t-1.0)*(2.0*t*t-t-1); else return 0.0;}
	Eigen::VectorXd ComputeDisplacementVector(int i,int j);
	Eigen::VectorXd AddDisplacementVector(const Eigen::VectorXd& m_t,const Eigen::VectorXd& d);
	


	double mSaveTotalElapsedTime;
	double mSaveElapsedTime;
	int mSaveCurrentSequence;
	Eigen::Vector6d mSaveCurrentRootOffset;
	Eigen::VectorXd mSaved;
	Eigen::Matrix3d mSaveRtarget;
	Eigen::Matrix3d mSaveRy;

};
};
#endif