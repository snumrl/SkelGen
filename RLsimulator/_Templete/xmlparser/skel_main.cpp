#include <tinyxml.h>
#include <Eigen/Core>
#include <map>
#include <vector>
#include <fstream>
#define LARGE_VALUE 1E6
#define REVOLUTE_JOINT_LIMIT 0.05
#define PROXIMAL_JOINT_LIMIT 1.0
namespace Eigen
{
	typedef Matrix<double,1,1> Vector1d;
};
struct UserConstant
{
	UserConstant(std::string parent,std::string joint_type,double mass,const Eigen::VectorXd& joint_lower_limit,const Eigen::VectorXd& joint_upper_limit,const Eigen::Vector3d& axis = Eigen::Vector3d::Zero(),std::string bvh_map="None",std::string obj_map="None",bool con = false)
		:mParent(parent),mJointType(joint_type),mMass(mass),mJointLowerLimit(joint_lower_limit),mJointUpperLimit(joint_upper_limit),mAxis(axis),mBVHMap(bvh_map),mOBJMap(obj_map),collision_on(con)
	{

	}
	std::string mParent;
	std::string mJointType;
	double mMass;
	std::string mBVHMap;
	std::string mOBJMap;
	Eigen::VectorXd mJointLowerLimit;
	Eigen::VectorXd mJointUpperLimit;
	Eigen::Vector3d mAxis;
	bool collision_on;
};
struct MayaConstant
{
	MayaConstant(std::string name, const Eigen::Vector3d& size,const Eigen::VectorXd& R,const Eigen::Vector3d& bt,const Eigen::Vector3d& jt)
		:mName(name),mSize(size),mBodyR(R),mBodyT(bt),mJointT(jt)
	{
		// mSize *= 0.5;
		// std::cout<<name<<std::endl;
		// std::cout<<mSize.transpose()<<std::endl;
		// std::cout<<mBodyR.transpose()<<std::endl;
		// std::cout<<mBodyT.transpose()<<std::endl;
		// std::cout<<mJointT.transpose()<<std::endl<<std::endl;
	}
	std::string mName;
	Eigen::Vector3d mSize;
	Eigen::VectorXd mBodyR;
	Eigen::Vector3d mBodyT;
	Eigen::Vector3d mJointT;
};
std::string toString(const Eigen::Vector3d& v)
{
	std::string ret ="";
	for(int i=0;i<v.rows();i++){
		ret += std::to_string(v[i]);
		ret += " ";
	}
	return ret;

}
std::string toString(const Eigen::VectorXd& v)
{
	std::string ret ="";
	for(int i=0;i<v.rows();i++){
		ret += std::to_string(v[i]);
		ret += " ";
	}
	return ret;	
}
void ReadMayaConstant(std::vector<MayaConstant>& mc,std::string path)
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

	double a1,a2,a3,a4,a5,a6,a7,a8,a9;
	std::string name;
	Eigen::Vector3d size;
	Eigen::VectorXd bodyR(9);
	Eigen::Vector3d bodyT;
	Eigen::Vector3d jointT;
	while(!ifs.eof())
	{
		str.clear();
		index.clear();
		ss.clear();

		std::getline(ifs,str);
		ss.str(str);
		ss>>index;
		if(index=="n")
		{
			ss>>name;
		}
		else if(index=="s")
		{
			ss>>a1>>a2>>a3;
			size<<a1,a2,a3;
		}
		else if(index=="R")
		{
			ss>>a1>>a2>>a3>>a4>>a5>>a6>>a7>>a8>>a9;
			bodyR<<a1,a2,a3,a4,a5,a6,a7,a8,a9;
		}
		else if(index=="t")
		{
			ss>>a1>>a2>>a3;
			bodyT<<a1,a2,a3;
		}
		else if(index=="j")
		{
			ss>>a1>>a2>>a3;
			jointT<<a1,a2,a3;
			mc.push_back(MayaConstant(name,size,bodyR,bodyT,jointT));
		}
	}

	ifs.close();
}
#define DETAIL
int main(int argc,char** argv)
{
	std::map<std::string,UserConstant> ucs;
	std::vector<MayaConstant> mcs;
	ReadMayaConstant(mcs,argv[1]);
	
	ucs.insert(std::make_pair("Pelvis",UserConstant("None","FreeJoint",15.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_Hips","Pelvis_mesh.obj")));
	//Upper body
	// ucs.insert(std::make_pair("Spine",UserConstant("Pelvis","WeldJoint",5.0,Eigen::Vector3d(-0.4,-0.4,-0.4),Eigen::Vector3d(0.4,0.4,0.4),Eigen::Vector3d(0,1,0),"None","Spine_mesh.obj")));
	ucs.insert(std::make_pair("Spine",UserConstant("Pelvis","BallJoint",5.0,Eigen::Vector3d(-2.0,-2.0,-2.0),Eigen::Vector3d(2.0,2.0,2.0),Eigen::Vector3d(0,1,0),"Character1_Spine","Spine_mesh.obj")));
	ucs.insert(std::make_pair("Torso",UserConstant("Spine","BallJoint",10.0,Eigen::Vector3d(-2.0,-2.0,-2.0),Eigen::Vector3d(2.0,2.0,2.0),Eigen::Vector3d(0,1,0),"Character1_Spine1","Torso_mesh.obj")));
	ucs.insert(std::make_pair("Neck",UserConstant("Torso","BallJoint",2.0,Eigen::Vector3d(-2.0,-2.0,-2.0),Eigen::Vector3d(2.0,2.0,2.0),Eigen::Vector3d(1,0,0),"Character1_Neck","Neck_mesh.obj")));
	ucs.insert(std::make_pair("Head",UserConstant("Neck","WeldJoint",4.0,Eigen::Vector3d(-LARGE_VALUE,-LARGE_VALUE,-LARGE_VALUE),Eigen::Vector3d(LARGE_VALUE,LARGE_VALUE,LARGE_VALUE),Eigen::Vector3d(1,0,0),"None","Skull_mesh.obj")));
	// ucs.insert(std::make_pair("Head",UserConstant("Neck","WeldJoint",4.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(1,0,0),"Character1_Head","Skull_mesh.obj")));
	ucs.insert(std::make_pair("ShoulderL",UserConstant("Torso","WeldJoint",3.0,Eigen::Vector3d(-LARGE_VALUE,-LARGE_VALUE,-LARGE_VALUE),Eigen::Vector3d(LARGE_VALUE,LARGE_VALUE,LARGE_VALUE),Eigen::Vector3d(0,1,0),"None","L_Shoulder_mesh.obj")));
	ucs.insert(std::make_pair("ArmL",UserConstant("ShoulderL","BallJoint",2.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_LeftArm","L_Humerus_mesh.obj")));
	ucs.insert(std::make_pair("ForeArmL",UserConstant("ArmL","RevoluteJoint",1.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,1,0),"Character1_LeftForeArm","L_Radius_mesh.obj")));
	ucs.insert(std::make_pair("HandL",UserConstant("ForeArmL","BallJoint",0.4,Eigen::Vector3d(-0.7,-0.7,-0.7),Eigen::Vector3d(0.7,0.7,0.7),Eigen::Vector3d(0,0,0),"Character1_LeftHand","L_Hand_mesh.obj")));
	ucs.insert(std::make_pair("ShoulderR",UserConstant("Torso","WeldJoint",3.0,Eigen::Vector3d(-LARGE_VALUE,-LARGE_VALUE,-LARGE_VALUE),Eigen::Vector3d(LARGE_VALUE,LARGE_VALUE,LARGE_VALUE),Eigen::Vector3d(0,1,0),"None","R_Shoulder_mesh.obj")));
	ucs.insert(std::make_pair("ArmR",UserConstant("ShoulderR","BallJoint",2.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_RightArm","R_Humerus_mesh.obj")));
	ucs.insert(std::make_pair("ForeArmR",UserConstant("ArmR","RevoluteJoint",1.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,1,0),"Character1_RightForeArm","R_Radius_mesh.obj")));
	ucs.insert(std::make_pair("HandR",UserConstant("ForeArmR","BallJoint",0.4,Eigen::Vector3d(-0.7,-0.7,-0.7),Eigen::Vector3d(0.7,0.7,0.7),Eigen::Vector3d(0,0,0),"Character1_RightHand","R_Hand_mesh.obj")));
	//Lower body
	ucs.insert(std::make_pair("FemurL",UserConstant("Pelvis","BallJoint",7.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_LeftUpLeg","L_Femur_mesh.obj")));
	ucs.insert(std::make_pair("SupportMain",UserConstant("FemurL","WeldJoint",1.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(1,0,0),"None","SupportMain_mesh.obj")));
	ucs.insert(std::make_pair("FemurR",UserConstant("Pelvis","BallJoint",7.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_RightUpLeg","R_Femur_mesh.obj")));
	ucs.insert(std::make_pair("TibiaR",UserConstant("FemurR","RevoluteJoint",3.0,Eigen::Vector1d(0.0),Eigen::Vector1d(2.3),Eigen::Vector3d(1,0,0),"Character1_RightLeg","R_Tibia_mesh.obj")));
#ifndef DETAIL
	ucs.insert(std::make_pair("TalusL",UserConstant("TibiaL","BallJoint",1.0,Eigen::Vector3d(-0.5,-0.5,-0.2),Eigen::Vector3d(0.5,0.5,0.2),Eigen::Vector3d(0,0,0),"Character1_LeftFoot","L_Talus_merge_mesh.obj",true)));
	ucs.insert(std::make_pair("TalusR",UserConstant("TibiaR","BallJoint",1.0,Eigen::Vector3d(-0.5,-0.5,-0.2),Eigen::Vector3d(0.5,0.5,0.2),Eigen::Vector3d(0,0,0),"Character1_RightFoot","R_Talus_merge_mesh.obj",true)));
#else
	ucs.insert(std::make_pair("SupportChild",UserConstant("SupportMain","WeldJoint",0.6,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"None","SupportChild_mesh.obj",true)));
	// ucs.insert(std::make_pair("FootThumbL",UserConstant("TalusL","RevoluteJoint",0.2,Eigen::Vector1d(-0.6),Eigen::Vector1d(0.6),Eigen::Vector3d(1,0,0),"None","FootThumbL_mesh.obj",true)));
	// ucs.insert(std::make_pair("FootPinkyL",UserConstant("TalusL","RevoluteJoint",0.2,Eigen::Vector1d(-0.6),Eigen::Vector1d(0.6),Eigen::Vector3d(1,0,0),"None","FootPinkyL_mesh.obj",true)));

	ucs.insert(std::make_pair("TalusR",UserConstant("TibiaR","BallJoint",0.6,Eigen::Vector3d(-2.0,-2.0,-2.0),Eigen::Vector3d(2.0,2.0,2.0),Eigen::Vector3d(0,0,0),"Character1_RightFoot","TalusR_mesh2.obj",true)));
	ucs.insert(std::make_pair("FootThumbR",UserConstant("TalusR","RevoluteJoint",0.2,Eigen::Vector1d(-0.6),Eigen::Vector1d(0.6),Eigen::Vector3d(1,0,0),"None","FootThumbR_mesh.obj",true)));
	ucs.insert(std::make_pair("FootPinkyR",UserConstant("TalusR","RevoluteJoint",0.2,Eigen::Vector1d(-0.6),Eigen::Vector1d(0.6),Eigen::Vector3d(1,0,0),"None","FootPinkyR_mesh.obj",true)));
#endif
	TiXmlDocument doc;
	TiXmlElement* skel_elem = new TiXmlElement("Skeleton");
	skel_elem->SetAttribute("name","Foot");
	doc.LinkEndChild(skel_elem);
	for(int i =0;i<mcs.size();i++)
	{
		TiXmlElement* joint_elem = new TiXmlElement("Joint");
		std::cout<<mcs[i].mName<<std::endl;
		if(ucs.find(mcs[i].mName)==ucs.end())
			continue;
		auto& uc = ucs.at(mcs[i].mName);
		// std::cout<<mcs[i].mName<<std::endl;

		joint_elem->SetAttribute("type",uc.mJointType);
		joint_elem->SetAttribute("name",mcs[i].mName);
		joint_elem->SetAttribute("parent_name",uc.mParent);
		joint_elem->SetAttribute("size",toString(mcs[i].mSize));
		joint_elem->SetAttribute("mass",std::to_string(uc.mMass));
		if(uc.mBVHMap!="None"){
			joint_elem->SetAttribute("bvh",uc.mBVHMap);
		}
		if(uc.mOBJMap!="None"){
			// joint_elem->SetAttribute("obj",uc.mOBJMap);
		}
		if(uc.mAxis.norm()>1E-4)
		{
			joint_elem->SetAttribute("axis",toString(uc.mAxis));
		}
		if(uc.collision_on==true)
		{
			joint_elem->SetAttribute("contact","On");	
		}
		TiXmlElement* body_position_elem = new TiXmlElement("BodyPosition");
		body_position_elem->SetAttribute("linear",toString(mcs[i].mBodyR));
		body_position_elem->SetAttribute("translation",toString(mcs[i].mBodyT));
		TiXmlElement* joint_position_elem = new TiXmlElement("JointPosition");
		joint_position_elem->SetAttribute("translation",toString(mcs[i].mJointT));
		if(uc.mJointLowerLimit.norm()<100.0)
		{
			joint_position_elem->SetAttribute("lower",toString(uc.mJointLowerLimit));
			joint_position_elem->SetAttribute("upper",toString(uc.mJointUpperLimit));	
		}
		
		joint_elem->LinkEndChild(body_position_elem);
		joint_elem->LinkEndChild(joint_position_elem);
		skel_elem->LinkEndChild( joint_elem );
	}
	TiXmlPrinter printer;
	printer.SetIndent( "\n" );

	doc.Accept( &printer );
	doc.SaveFile("../model/character.xml");
	
	return 0;
}
