#ifndef __MSS_BVH_H__
#define __MSS_BVH_H__
#include <Eigen/Core>
#include <string>
#include <fstream>
#include <vector>
#include <map>
#include <utility>
#include <initializer_list>
 
namespace MSS
{
Eigen::Matrix3d R_x(double x);
Eigen::Matrix3d R_y(double y);
Eigen::Matrix3d R_z(double z);
class BVHNode
{
public:
	enum CHANNEL
	{
		Xpos=0,
		Ypos=1,
		Zpos=2,
		Xrot=3,
		Yrot=4,
		Zrot=5
	}; 
	static std::map<std::string,MSS::BVHNode::CHANNEL> CHANNEL_NAME;


	BVHNode(const std::string& name,BVHNode* parent);
	void SetChannel(int c_offset,std::vector<std::string>& c_name);
	void Set(const Eigen::VectorXd& m_t);
	void Set(const Eigen::Matrix3d& R_t);
	Eigen::Matrix3d Get();

	void AddChild(BVHNode* child);
	BVHNode* GetNode(const std::string& name);
private:
	BVHNode* mParent;
	std::vector<BVHNode*> mChildren;

	Eigen::Matrix3d mR;
	std::string mName;

	int mChannelOffset;
	int mNumChannels;
	std::vector<BVHNode::CHANNEL> mChannel;
};
class BVH
{
public:
	BVH();

	void SetMotion(double t);
	const Eigen::Vector3d& GetRootCOM(){return mRootCOM;}
	Eigen::Matrix3d Get(const std::string& bvh_node);

	double GetMaxTime(){return mNumTotalFrames*mTimeStep;}
	double GetTimeStep(){return mTimeStep;}
	void Parse(const std::string& file);
	Eigen::Vector3d GetP0();

public:
	std::vector<Eigen::VectorXd> mMotions;
	std::map<std::string,BVHNode*> mMap;
	double mTimeStep;
	int mNumTotalChannels;
	int mNumTotalFrames;

	BVHNode* mRoot;
	Eigen::Vector3d mRootCOM;
	Eigen::Vector3d mRootCOMOffset;

	BVHNode* ReadHierarchy(BVHNode* parent,const std::string& name,int& channel_offset,std::ifstream& is);
};

};

#endif