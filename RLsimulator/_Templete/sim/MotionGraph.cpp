#include "MotionGraph.h"
#include <algorithm>
#include <math.h>
namespace MSS
{
MotionGraph::
MotionGraph(const dart::dynamics::SkeletonPtr& skel,std::map<std::string,std::string> bvh_map,double time_step)
	:mSkeleton(skel),mBVHMap(bvh_map),mTimeStep(time_step),mTotalElapsedTime(0.0),mElapsedTime(0.0),mCurrentSequence(-1),mCurrentRootOffset(Eigen::Vector6d::Zero()),md(Eigen::VectorXd::Zero(skel->getNumDofs())),mt_s(0.2),mRtarget(Eigen::Matrix3d::Identity()),mRy(Eigen::Matrix3d::Identity())
{

}
void
MotionGraph::
Parse(const std::string& path)
{
	std::ifstream ifs(path);
	if(!(ifs.is_open()))
	{
		std::cout<<"Can't read file "<<path<<std::endl;
		return;
	}
	std::string str;
	std::string index;
	std::stringstream ss;

	double x,y,z;
	int val,val1,val2;
	while(!ifs.eof())
	{
		str.clear();
		index.clear();
		ss.clear();

		std::getline(ifs,str);
		ss.str(str);
		ss>>index;
		if(index=="m")
		{
			ss>>index;
			mBVHs.push_back(new BVH());
			// std::cout<<index<<std::endl;
			mBVHs.back()->Parse(std::string(MSS_ROOT_DIR)+std::string("/motion/")+index);
		}
		else if(index=="t")
		{
			if(mTransitions.size()==0)
			{
				val = mBVHs.size();
				mTransitions.resize(val);
				for(int i =0;i<val;i++)
					mTransitions[i].reserve(val);
			}
			ss>>val1>>val2;
			// std::cout<<val1<<" "<<val2<<std::endl;
			mTransitions[val1].push_back(val2);	
		}

	}

    ifs.close(); }

void
MotionGraph::
Reset(double t)
{
	mTotalElapsedTime = t;
	mCurrentSequence = 0;
	mElapsedTime = t;
	mCurrentRootOffset.setZero();
	mRtarget.setIdentity();
	mRy.setIdentity();
	// mCurrentRootOffset.segment<3>(0) = dart::math::logMap(mBVHs[mSequences[mCurrentSequence]]->GetR(mElapsedTime));
	// mCurrentRootOffset.segment<3>(3) = dart::math::logMap(mBVHs[mSequences[mCurrentSequence]]->GetP(mElapsedTime));
	md.setZero();
}
void
MotionGraph::
SetWalk(const std::vector<int>& sequences)
{
	bool is_valid = true;
	for(int i =0;i<sequences.size()-1;i++)
	{
		if(std::find(mTransitions[sequences[i]].begin(), mTransitions[sequences[i]].end(), sequences[i+1]) == mTransitions[sequences[i]].end())
			is_valid = false;
	}

	if(!is_valid)
		std::cout<<"Put valid sequence."<<std::endl;

	mSequences = sequences;

	mMaxTime = 0.0;
	for(int i=0;i<mSequences.size();i++)
		mMaxTime += mBVHs[mSequences[i]]->GetMaxTime();
}
void
MotionGraph::
Step()
{
	mTotalElapsedTime += mTimeStep;
	mElapsedTime += mTimeStep;
	if(mElapsedTime>mBVHs[mSequences[mCurrentSequence]]->GetMaxTime())
	{
		using namespace dart::math;
		Eigen::VectorXd m_f = ComputeMotionVector(mSequences[mCurrentSequence],mElapsedTime);
		
		int next = (mCurrentSequence+1)%mSequences.size();
		md = ComputeDisplacementVector(mSequences[mCurrentSequence],mSequences[next]);
		mCurrentSequence = next;
		mElapsedTime =0.0;
		
		Eigen::VectorXd m_next_0 = ComputeMotionVector(mSequences[mCurrentSequence],mElapsedTime);
		//Find Closest Rotation
		Eigen::Matrix3d R_f = expMapRot(mCurrentRootOffset.segment<3>(0))*expMapRot(m_f.segment<3>(0));
		Eigen::Matrix3d R_target = R_f*expMapRot(m_next_0.segment<3>(0)).transpose();

		double y;
		double r11_plus_r33 = R_target(0,0)+R_target(2,2);
		double r31_minus_r13 = R_target(2,0)-R_target(0,2);
		if(std::abs(r11_plus_r33)<1E-4)
		{
			if(r31_minus_r13>0.0)
				y = -1.57;
			else
				y = 1.57;
		}
		else if(std::abs(r31_minus_r13)<1E-4)
		{
			if(r11_plus_r33>0.0)
				y = 0.0;
			else
				y = 3.14;
		}
		else
		{
			double tan_y = -(r31_minus_r13)/(r11_plus_r33);
			y = atan(tan_y);
			if(r11_plus_r33<0){
				y+=3.14;
			}
		}
		Eigen::Matrix3d Ry = expMapRot(Eigen::Vector3d(0,y,0));
		mRtarget = R_target;
		mRy = Ry;
		mCurrentRootOffset.segment<3>(3) = mCurrentRootOffset.segment<3>(3) + expMapRot(mCurrentRootOffset.segment<3>(0))*(m_f.segment<3>(3)-m_next_0.segment<3>(3));
		mCurrentRootOffset[4]=0.0;
		mCurrentRootOffset.segment<3>(0) = logMap(Ry);
		
	}
}
Eigen::VectorXd
MotionGraph::
GetMotion()
{
	using namespace dart::math;
	Eigen::VectorXd m_t = ComputeMotionVector(mSequences[mCurrentSequence],mElapsedTime);
	// if(mElapsedTime<mt_s)
	// {
	// 	double s = ScaleFunc(mElapsedTime/mt_s);

	// 	m_t = AddDisplacementVector(m_t,s*md);
	// 	Eigen::Matrix3d R_diff = mRy.transpose()*mRtarget;
	// 	Eigen::Vector3d R_diff_log = logMap(R_diff);
	// 	m_t.segment<3>(0) = logMap(expMapRot(s*R_diff_log)*expMapRot(m_t.segment<3>(0)));
	// }
	// Eigen::Matrix3d R_t = expMapRot(mCurrentRootOffset.segment<3>(0))*expMapRot(m_t.segment<3>(0));
	// m_t.segment<3>(0) = logMap(R_t);
	// // m_t.segment<3>(3) = mCurrentRootOffset.segment<3>(3) + expMapRot(mCurrentRootOffset.segment<3>(0))*(m_t.segment<3>(3)-mBVHs[mSequences[mCurrentSequence]]->GetP0());
	// m_t.segment<3>(3) = mCurrentRootOffset.segment<3>(3) + mRy*(m_t.segment<3>(3)-mBVHs[mSequences[mCurrentSequence]]->GetP0());
	return m_t;
}
Eigen::VectorXd
MotionGraph::
ComputeDisplacementVector(int i,int j)
{
	int dof = mSkeleton->getPositions().rows();
	Eigen::VectorXd d = Eigen::VectorXd::Zero(dof);

	Eigen::VectorXd m_i = ComputeMotionVector(i,mBVHs[i]->GetMaxTime());
	Eigen::VectorXd m_j = ComputeMotionVector(j,0.0);
	// d = m_i-m_j;
	for(auto ss :mBVHMap)
	{
		dart::dynamics::BodyNode* bn = mSkeleton->getBodyNode(ss.first);
		dart::dynamics::Joint* jn = bn->getParentJoint();
		int idx = jn->getIndexInSkeleton(0);
		if(jn->getType()=="FreeJoint")
		{
			Eigen::Matrix3d R_i = dart::dynamics::BallJoint::convertToRotation(m_i.segment<3>(idx));
			Eigen::Matrix3d R_j  = dart::dynamics::BallJoint::convertToRotation(m_j.segment<3>(idx));
			d.segment<3>(idx) = dart::dynamics::BallJoint::convertToPositions(R_i*R_j.transpose());
			d.segment<3>(idx+3) = m_i.segment<3>(idx) - m_j.segment<3>(idx);
		}
		else if(jn->getType()=="BallJoint")
		{
			Eigen::Matrix3d R_i = dart::dynamics::BallJoint::convertToRotation(m_i.segment<3>(idx));
			Eigen::Matrix3d R_j  = dart::dynamics::BallJoint::convertToRotation(m_j.segment<3>(idx));
			d.segment<3>(idx) = dart::dynamics::BallJoint::convertToPositions(R_i*R_j.transpose());
			// if(bn->getName()=="FemurL"){
			// 	std::cout<<d.segment<3>(idx).transpose()<<std::endl;
			// }
			// d.segment<3>(idx) = m_i.segment<3>(idx) - m_j.segment<3>(idx);
		}
		else if(jn->getType()=="RevoluteJoint")
			d[idx] = m_i[idx] - m_j[idx];
	}	
	// std::cout<<d.head<6>(0).transpose()<<std::endl;

	return d;
}
Eigen::VectorXd
MotionGraph::
AddDisplacementVector(const Eigen::VectorXd& m_t, const Eigen::VectorXd& d)
{
	int dof = mSkeleton->getPositions().rows();
	Eigen::VectorXd p = m_t;

	// p += d;
	// p.segment<6>(0) = m_t.segment<6>(0);
	for(auto ss :mBVHMap)
	{
		dart::dynamics::BodyNode* bn = mSkeleton->getBodyNode(ss.first);
		dart::dynamics::Joint* jn = bn->getParentJoint();
		int idx = jn->getIndexInSkeleton(0);
		// std::cout<<idx<<" ";
		if(jn->getType()=="FreeJoint")
		{
			// Eigen::Matrix3d R_mt = dart::math::expMapRot(m_t.segment<3>(idx));
			// Eigen::Matrix3d R_d  = dart::math::expMapRot(d.segment<3>(idx));
			// p.segment<3>(idx) = dart::math::logMap(R_mt*R_d);
		}
		else if(jn->getType()=="BallJoint")
		{
			Eigen::Matrix3d R_mt = dart::dynamics::BallJoint::convertToRotation(m_t.segment<3>(idx));
			Eigen::Matrix3d R_d  = dart::dynamics::BallJoint::convertToRotation(d.segment<3>(idx));
			p.segment<3>(idx) = dart::dynamics::BallJoint::convertToPositions(R_d*R_mt);
			
			// p.segment<3>(idx) = m_t.segment<3>(idx) + d.segment<3>(idx);
		}
		else if(jn->getType()=="RevoluteJoint")
			p[idx] = m_t[idx] + d[idx];
	}
	// std::cout<<std::endl;
	return p;
}
Eigen::VectorXd
MotionGraph::
ComputeMotionVector(int i,double t)
{
	
	int dof = mSkeleton->getPositions().rows();
	Eigen::VectorXd p = Eigen::VectorXd::Zero(dof);
	
	// std::cout<<i<<" : "<<t<<std::endl;
	mBVHs[i]->SetMotion(t);
	
	for(auto ss :mBVHMap)
	{
		dart::dynamics::BodyNode* bn = mSkeleton->getBodyNode(ss.first);
		Eigen::Matrix3d R = mBVHs[i]->Get(ss.second);
		dart::dynamics::Joint* jn = bn->getParentJoint();
		Eigen::Vector3d a = dart::dynamics::BallJoint::convertToPositions(R);
		int idx = jn->getIndexInSkeleton(0);
		if(jn->getType()=="FreeJoint")
		{
			p.segment<3>(idx) = a;
			// std::cout<<mBVHs[i]->GetRootCOM().transpose()<<std::endl;
			p.segment<3>(idx+3) = mBVHs[i]->GetRootCOM();

		}
		if(jn->getType()=="BallJoint")
			p.segment<3>(idx) = a;
		else if(jn->getType()=="RevoluteJoint"){
			Eigen::Vector3d u = dynamic_cast<dart::dynamics::RevoluteJoint*>(jn)->getAxis();
			// u.normalize();
			// std::cout<<u.transpose()<<std::endl;
			// Eigen::Vector3d w = dart::math::logMap(R);
			// double angle = w.norm();
			// w.normalize();
			// double tan = (u.cross(w)).norm()/u.dot(w);
			// std::cout<<w.transpose()<<std::endl;
			// double phi = angle*tan;
			// p[idx] = phi;
			if((u-Eigen::Vector3d::UnitX()).norm()<1E-4)
				p[idx] = a[0];
			else if((u-Eigen::Vector3d::UnitY()).norm()<1E-4)
				p[idx] = a[1];
			else if((u-Eigen::Vector3d::UnitZ()).norm()<1E-4)
				p[idx] = a[2];
			if(p[idx]>3.141592)
				p[idx]-=2*3.141592;
			else if(p[idx]<-3.141592)
				p[idx]+=2*3.141592;
		}
	}
	

	return p;
}
void
MotionGraph::
SaveState()
{
	mSaveTotalElapsedTime = mTotalElapsedTime;
	mSaveElapsedTime = mElapsedTime;
	mSaveCurrentSequence = mCurrentSequence;
	mSaveCurrentRootOffset = mCurrentRootOffset;
	mSaved = md;
	mSaveRtarget = mRtarget;
	mSaveRy = mRy;
}
void
MotionGraph::
LoadState()
{
	mTotalElapsedTime = mSaveTotalElapsedTime;
	mElapsedTime = mSaveElapsedTime;
	mCurrentSequence = mSaveCurrentSequence;
	mCurrentRootOffset = mSaveCurrentRootOffset;
	md = mSaved;
	mRtarget = mSaveRtarget;
	mRy = mSaveRy;
}
};