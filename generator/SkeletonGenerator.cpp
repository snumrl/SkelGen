//
// Created by hoseok on 10/30/18.
//

#include "SkeletonGenerator.h"
#include <tinyxml.h>
#include <fstream>

using namespace std;
using namespace dart::constraint;
using namespace dart::dynamics;
using namespace dart::simulation;

#include <Eigen/Core>
#include <map>
#include <vector>
#include <queue>
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

void ReadMayaConstant(std::vector<MayaConstant>& mcs,std::string path)
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
			mcs.push_back(MayaConstant(name,size,bodyR,bodyT,jointT));
		}
	}

	ifs.close();
}

// using maya skeleton textfile to make XML file
void makeSkeletonXml(std::string mayaPath, std::string xmlPath, std::string prefix){

	std::map<std::string,UserConstant> ucs;
	std::vector<MayaConstant> mcs;
	ReadMayaConstant(mcs,mayaPath);

	// UserConstant(parent, joint_type, mass, joint_lower_limit, joint_upper_limit, axis, bvh_map, obj_map, con)
	// Pelvis
	ucs.insert(std::make_pair("Pelvis",UserConstant("None","FreeJoint",12.16,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_Hips","Pelvis.obj")));
	//Upper body
//	ucs.insert(std::make_pair("Spine",UserConstant("Pelvis","BallJoint",5.0,Eigen::Vector3d(-0.4,-0.4,-0.4),Eigen::Vector3d(0.4,0.4,0.4),Eigen::Vector3d(0,1,0),"Character1_Spine","Spine.obj")));
//	ucs.insert(std::make_pair("Torso",UserConstant("Spine","BallJoint",10.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_Spine1","Torso.obj")));
//	ucs.insert(std::make_pair("Neck",UserConstant("Torso","RevoluteJoint",2.0,Eigen::Vector1d(-0.4),Eigen::Vector1d(0.4),Eigen::Vector3d(1,0,0),"Character1_Neck","Neck.obj")));
//	ucs.insert(std::make_pair("Head",UserConstant("Neck","BallJoint",4.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(1,0,0),"Character1_Head","Skull.obj")));
//	ucs.insert(std::make_pair("ShoulderL",UserConstant("Torso","RevoluteJoint",3.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,1,0),"Character1_LeftShoulder","L_Shoulder.obj")));
//	ucs.insert(std::make_pair("ArmL",UserConstant("ShoulderL","BallJoint",2.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_LeftArm","L_Humerus.obj")));
//	ucs.insert(std::make_pair("ForeArmL",UserConstant("ArmL","RevoluteJoint",1.0,Eigen::Vector1d(-3.0),Eigen::Vector1d(-0.0),Eigen::Vector3d(0,1,0),"Character1_LeftForeArm","L_Radius.obj")));
	// ucs.insert(std::make_pair("HandL",UserConstant("ForeArmL","BallJoint",0.7,Eigen::Vector3d(-0.7,-0.7,-0.7),Eigen::Vector3d(0.7,0.7,0.7),Eigen::Vector3d(0,0,0),"Character1_LeftHand","HandL_racket.obj")));
//	ucs.insert(std::make_pair("HandL",UserConstant("ForeArmL","BallJoint",0.7,Eigen::Vector3d(-0.7,-0.7,-0.7),Eigen::Vector3d(0.7,0.7,0.7),Eigen::Vector3d(0,0,0),"Character1_LeftHand","L_Hand.obj")));
//	ucs.insert(std::make_pair("ShoulderR",UserConstant("Torso","RevoluteJoint",3.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,1,0),"Character1_RightShoulder","R_Shoulder.obj")));
//	ucs.insert(std::make_pair("ArmR",UserConstant("ShoulderR","BallJoint",2.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_RightArm","R_Humerus.obj")));
//	ucs.insert(std::make_pair("ForeArmR",UserConstant("ArmR","RevoluteJoint",1.0,Eigen::Vector1d(-0.5),Eigen::Vector1d(0.0),Eigen::Vector3d(0,1,0),"Character1_RightForeArm","R_Radius.obj")));
//	ucs.insert(std::make_pair("HandR",UserConstant("ForeArmR","BallJoint",0.7,Eigen::Vector3d(-0.7,-0.7,-0.7),Eigen::Vector3d(0.7,0.7,0.7),Eigen::Vector3d(0,0,0),"Character1_RightHand","R_Hand.obj")));
	//Lower body
	ucs.insert(std::make_pair("R_Femur",UserConstant("Pelvis","BallJoint",8.512,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_RightUpLeg",prefix+"R_Femur.obj")));
	ucs.insert(std::make_pair("L_Femur",UserConstant("Pelvis","BallJoint",8.512,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_LeftUpLeg",prefix+"L_Femur.obj")));

	ucs.insert(std::make_pair("R_Tibia",UserConstant("R_Femur","RevoluteJoint",3.648,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(1,0,0),"Character1_RightLeg",prefix+"R_Tibia.obj")));
	ucs.insert(std::make_pair("L_Tibia",UserConstant("L_Femur","RevoluteJoint",3.648,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(1,0,0),"Character1_LeftLeg",prefix+"L_Tibia.obj")));

	ucs.insert(std::make_pair("R_Foot",UserConstant("R_Tibia","BallJoint",1.22,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_RightFoot",prefix+"R_Foot.obj")));
	ucs.insert(std::make_pair("L_Foot",UserConstant("L_Tibia","BallJoint",1.22,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_LeftFoot",prefix+"L_Foot.obj")));

	//Upper body
	ucs.insert(std::make_pair("Spine",UserConstant("Pelvis","BallJoint",5.0,Eigen::Vector3d(-0.4,-0.4,-0.4),Eigen::Vector3d(0.4,0.4,0.4),Eigen::Vector3d(0,1,0),"Character1_Spine","Spine.obj")));
	ucs.insert(std::make_pair("Torso",UserConstant("Spine","BallJoint",10.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_Spine1","Torso.obj")));
	ucs.insert(std::make_pair("Neck",UserConstant("Torso","RevoluteJoint",2.0,Eigen::Vector1d(-0.4),Eigen::Vector1d(0.4),Eigen::Vector3d(1,0,0),"Character1_Neck","Neck.obj")));
	ucs.insert(std::make_pair("Head",UserConstant("Neck","BallJoint",4.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(1,0,0),"Character1_Head","Head.obj")));
	ucs.insert(std::make_pair("L_Shoulder",UserConstant("Torso","RevoluteJoint",3.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,1,0),"Character1_LeftShoulder","L_Shoulder.obj")));
	ucs.insert(std::make_pair("L_Arm",UserConstant("L_Shoulder","BallJoint",2.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_LeftArm","L_Arm.obj")));
	ucs.insert(std::make_pair("L_ForeArm",UserConstant("L_Arm","RevoluteJoint",1.0,Eigen::Vector1d(-3.0),Eigen::Vector1d(-0.0),Eigen::Vector3d(0,1,0),"Character1_LeftForeArm","L_ForeArm.obj")));
	ucs.insert(std::make_pair("L_Hand",UserConstant("L_ForeArm","BallJoint",0.7,Eigen::Vector3d(-0.7,-0.7,-0.7),Eigen::Vector3d(0.7,0.7,0.7),Eigen::Vector3d(0,0,0),"Character1_LeftHand","L_Hand.obj")));
	ucs.insert(std::make_pair("R_Shoulder",UserConstant("Torso","RevoluteJoint",3.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,1,0),"Character1_RightShoulder","R_Shoulder.obj")));
	ucs.insert(std::make_pair("R_Arm",UserConstant("R_Shoulder","BallJoint",2.0,Eigen::Vector1d(-LARGE_VALUE),Eigen::Vector1d(LARGE_VALUE),Eigen::Vector3d(0,0,0),"Character1_RightArm","R_Arm.obj")));
	ucs.insert(std::make_pair("R_ForeArm",UserConstant("R_Arm","RevoluteJoint",1.0,Eigen::Vector1d(-0.5),Eigen::Vector1d(0.0),Eigen::Vector3d(0,1,0),"Character1_RightForeArm","R_ForeArm.obj")));
	ucs.insert(std::make_pair("R_Hand",UserConstant("R_ForeArm","BallJoint",0.7,Eigen::Vector3d(-0.7,-0.7,-0.7),Eigen::Vector3d(0.7,0.7,0.7),Eigen::Vector3d(0,0,0),"Character1_RightHand","R_Hand.obj")));
	TiXmlDocument doc;
	TiXmlElement* skel_elem = new TiXmlElement("Skeleton");
	skel_elem->SetAttribute("name",prefix+"Skeleton");
	doc.LinkEndChild(skel_elem);
	for(int i =0;i<mcs.size();i++)
	{
		TiXmlElement* joint_elem = new TiXmlElement("Joint");
//		std::cout<<mcs[i].mName<<std::endl;
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
			 joint_elem->SetAttribute("obj",uc.mOBJMap);
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
	doc.SaveFile(xmlPath);
}


SkeletonPtr BuildFromFile(const std::string& filename, bool obj_visual_consensus, Eigen::Vector3d rootTranslation){
	TiXmlDocument doc;
	if(!doc.LoadFile(filename)){
		std::cout << "Can't open file : " << filename << std::endl;
		return nullptr;
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
		std::string objName = "None";
		if(body->Attribute("obj")!=nullptr)
			objName = body->Attribute("obj");
		BodyNode *parent;
		if(!parentName.compare("None"))
			parent = nullptr;
		else {
			parent = skel->getBodyNode(parentName);
		}

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

		bool contact = false;

		if(body->Attribute("contact")!=nullptr)
		{
			if(std::string(body->Attribute("contact"))=="On")
				contact = true;
		}
		if(!jointType.compare("FreeJoint") ){
			MakeFreeJointBody(
					name,
					objName,
					skel,
					parent,
					size,
					jointPosition,
					bodyPosition,
					mass,
					contact,
					obj_visual_consensus
			);
		}
		else if(!jointType.compare("BallJoint")){
			// joint limit
			bool limit_enforced = false;
			Eigen::Vector3d upperLimit(1E6,1E6,1E6), lowerLimit(-1E6,-1E6,-1E6);
			if(jointPosElem->Attribute("upper")!=nullptr)
			{
				limit_enforced = true;
				upperLimit = string_to_vector3d(jointPosElem->Attribute("upper"));
				lowerLimit = string_to_vector3d(jointPosElem->Attribute("lower"));
			}

			MakeBallJointBody(
					name,
					objName,
					skel,
					parent,
					size,
					jointPosition,
					bodyPosition,
					limit_enforced,
					upperLimit,
					lowerLimit,
					mass,
					contact,
					obj_visual_consensus
			);
		}
		else if(!jointType.compare("RevoluteJoint")){
			// joint limit
			bool limit_enforced = false;
			double upperLimit(1E6), lowerLimit(-1E6);
			if(jointPosElem->Attribute("upper")!=nullptr)
			{
				limit_enforced = true;
				upperLimit = atof(jointPosElem->Attribute("upper"));
				lowerLimit = atof(jointPosElem->Attribute("lower"));
			}

			// axis
			Eigen::Vector3d axis = string_to_vector3d(body->Attribute("axis"));

			MakeRevoluteJointBody(
					name,
					objName,
					skel,
					parent,
					size,
					jointPosition,
					bodyPosition,
					limit_enforced,
					upperLimit,
					lowerLimit,
					mass,
					axis,
					contact,
					obj_visual_consensus
			);
		}
		else if(!jointType.compare("PrismaticJoint")){
			// joint limit
			TiXmlElement *jointLimitElem = body->FirstChildElement("Limit");
			bool isLimitEnforced = false;
			double upperLimit, lowerLimit;
			if( jointLimitElem != nullptr ){
				isLimitEnforced = true;
				upperLimit = atof(jointLimitElem->Attribute("upper"));
				lowerLimit = atof(jointLimitElem->Attribute("lower"));
			}
			// axis
			Eigen::Vector3d axis = string_to_vector3d(body->Attribute("axis"));

			MakePrismaticJointBody(
					name,
					objName,
					skel,
					parent,
					size,
					jointPosition,
					bodyPosition,
					isLimitEnforced,
					upperLimit,
					lowerLimit,
					mass,
					axis,
					contact,
					obj_visual_consensus
			);
		}
		else if(!jointType.compare("WeldJoint")){
			MakeWeldJointBody(
					name,
					objName,
					skel,
					parent,
					size,
					jointPosition,
					bodyPosition,
					mass,
					contact,
					obj_visual_consensus
			);
		}
		skel->getBodyNode(0)->getTransform();
	}
//	std::cout<<"(DOFs : "<<skel->getNumDofs()<<")"<< std::endl;
//	skel->getBodyNode("Pelvis")->getParentJoint()->setPosition(3,rootTranslation[0]);
//	skel->setPosition(3,1.0);
//	skel->getJoint(0)->setPosition(0,rootTranslation[0]);
	return skel;
}

void BuildFromSkeleton(const std::string& inputSkeletonXmlPath, const std::string& userInput, const std::string& outputSkeletonXmlPath,
                       vector<MetaBoneInfo>& mbis){
	TiXmlDocument inputDoc, outputDoc;
	if(!inputDoc.LoadFile(inputSkeletonXmlPath)){
		std::cout << "Can't open file : " << inputSkeletonXmlPath << std::endl;
		return;
	}

	TiXmlElement *inputSkelDoc = inputDoc.FirstChildElement("Skeleton");

	TiXmlElement* skel_elem = new TiXmlElement("Skeleton");
	skel_elem->SetAttribute("name","rtgSkeleton");
	outputDoc.LinkEndChild(skel_elem);

	vector< pair<string, Eigen::Isometry3d> > diffMatrices;

	for(TiXmlElement *body = inputSkelDoc->FirstChildElement("Joint"); body != nullptr; body = body->NextSiblingElement("Joint")){
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

		// contactness
		bool contact = false;

		if(body->Attribute("contact")!=nullptr)
		{
			if(std::string(body->Attribute("contact"))=="On")
				contact = true;
		}

		// axis
		Eigen::Vector3d axis(0,0,0);
		if(body->Attribute("axis")!=nullptr)
		{
			axis = string_to_vector3d(body->Attribute("axis"));
		}


		TiXmlElement* joint_elem = new TiXmlElement("Joint");
		joint_elem->SetAttribute("type",jointType);
		joint_elem->SetAttribute("name",name);
		joint_elem->SetAttribute("parent_name",parentName);
		joint_elem->SetAttribute("size",toString(size));
		joint_elem->SetAttribute("mass",std::to_string(mass));
		joint_elem->SetAttribute("bvh",bvhName);
		joint_elem->SetAttribute("obj",objName);
		if (body->Attribute("axis")!=nullptr){
			joint_elem->SetAttribute("axis",body->Attribute("axis"));
		}
		TiXmlElement* body_position_elem = new TiXmlElement("BodyPosition");
		body_position_elem->SetAttribute("linear",toString(bodyPosition));
		body_position_elem->SetAttribute("translation",toString(Eigen::Vector3d(bodyPosition.translation())));
		TiXmlElement* joint_position_elem = new TiXmlElement("JointPosition");
		if(jointPosElem->Attribute("linear")!=nullptr)
			joint_position_elem->SetAttribute("linear",toString(jointPosition));
		joint_position_elem->SetAttribute("translation",toString(Eigen::Vector3d(jointPosition.translation())));


		// joint limit
		if (jointPosElem->Attribute("lower") != nullptr){
			joint_position_elem->SetAttribute("lower", jointPosElem->Attribute("lower"));
		}
		if (jointPosElem->Attribute("upper") != nullptr){
			joint_position_elem->SetAttribute("upper", jointPosElem->Attribute("upper"));
		}


		joint_elem->LinkEndChild(body_position_elem);
		joint_elem->LinkEndChild(joint_position_elem);
		skel_elem->LinkEndChild( joint_elem );

	}

	/************************** OUTPUT **************************/


//		if (rtgFlag) objName = "rtg" + objName;
	TiXmlElement *outputSkelDoc = outputDoc.FirstChildElement("Skeleton");

	for(TiXmlElement *body = outputSkelDoc->FirstChildElement("Joint"); body != nullptr; body = body->NextSiblingElement("Joint")){
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

		// contactness
		bool contact = false;

		if(body->Attribute("contact")!=nullptr)
		{
			if(std::string(body->Attribute("contact"))=="On")
				contact = true;
		}

		// axis
		Eigen::Vector3d axis(0,0,0);
		if(body->Attribute("axis")!=nullptr)
		{
			axis = string_to_vector3d(body->Attribute("axis"));
		}

		// search this node in meta bone info set
		bool rtgFlag = false;

		Eigen::Vector3d shiftVector(0,0,0); // every child bodies shifts "shiftVector"
		double lengthRatio = 0, angle = 0;           // every child bodies rotates angle with axis as "shiftVector"
		for (auto& mbi: mbis){
			if (mbi.name == name){
				// lengthening
				shiftVector = bodyPosition.translation() - jointPosition.translation(); // body to joint (global vector of local y-axis)
				shiftVector.normalize();
				int dir;
				if (name.find("Femur") != string::npos || name.find("Tibia") != string::npos
				    || name.find("Spine") != string::npos
				    || name.find("Neck") != string::npos) dir = 1;
				else dir = 0;

				if (name.find("Torso") != string::npos){
					shiftVector = Eigen::Vector3d(1,0,0);
					lengthRatio = (size[dir] * (mbi.alpha_lengthening - 1.0)) / 2.0;
					size[dir] *= mbi.alpha_lengthening;
					angle = 0;
				}else {

					// for this bodynode, translating toward "shiftVector" direction
					lengthRatio = (size[dir] * (mbi.alpha_lengthening - 1.0));
					bodyPosition.translation() += lengthRatio / 2.0 * shiftVector;
					bodyPosElem->SetAttribute("translation", toString(Eigen::Vector3d(bodyPosition.translation())));
					size[dir] *= mbi.alpha_lengthening;

					// twist
					angle = mbi.theta_distal * M_PI / 180.0;
				}

				rtgFlag = true;

				break;
			}
		}


		if (rtgFlag) {
			// size reform
			body->SetAttribute("size",toString(size));

			// obj reform
			objName = "rtg" + objName;
			body->SetAttribute("obj",objName);

			Eigen::Vector3d stdPoint(0,0,0);
			Eigen::AngleAxisd rot = Eigen::AngleAxisd(angle, -shiftVector);

			queue<string> queueName;
			for (TiXmlElement *_body = outputSkelDoc->FirstChildElement("Joint"); _body != nullptr; _body = _body->NextSiblingElement("Joint")) {
				if (_body->Attribute("parent_name") == name)
					queueName.push(_body->Attribute("name"));
			}
			while (!queueName.empty()){

				string _name = queueName.front(); queueName.pop();

				TiXmlElement *_body = outputSkelDoc->FirstChildElement("Joint");
				for (; _body != nullptr; _body = _body->NextSiblingElement("Joint")) {
					if (_body->Attribute("parent_name") == _name)
						queueName.push(_body->Attribute("name"));
				}

				_body = outputSkelDoc->FirstChildElement("Joint");
				for (; _body != nullptr; _body = _body->NextSiblingElement("Joint")) {
					if (_body->Attribute("name") == _name)
						break;
				}


				// body position
				TiXmlElement *_bodyPosElem = _body->FirstChildElement("BodyPosition");
				Eigen::Isometry3d _bodyPosition;
				_bodyPosition.setIdentity();
				_bodyPosition.linear() = string_to_matrix3d(_bodyPosElem->Attribute("linear"));
				_bodyPosition.translation() = string_to_vector3d(_bodyPosElem->Attribute("translation"));
				_bodyPosition = Orthonormalize(_bodyPosition);
				// joint position
				TiXmlElement *_jointPosElem = _body->FirstChildElement("JointPosition");
				Eigen::Isometry3d _jointPosition;
				_jointPosition.setIdentity();
				if(_jointPosElem->Attribute("linear")!=nullptr)
					_jointPosition.linear() = string_to_matrix3d(_jointPosElem->Attribute("linear"));
				_jointPosition.translation() = string_to_vector3d(_jointPosElem->Attribute("translation"));
				_jointPosition = Orthonormalize(_jointPosition);
				// axis
				Eigen::Vector3d _axis(0,0,0);
				if(_body->Attribute("axis")!=nullptr)
					_axis = string_to_vector3d(_body->Attribute("axis"));


				if (stdPoint.norm()<1e-3){ // first child
					stdPoint = _jointPosition.translation();
				}

				// bodyposition transform rotating
//				_jointPosition.linear() = rot * _jointPosition.linear();
				_bodyPosition.linear() = rot * _bodyPosition.linear();

				// joint & body translation
				_jointPosition.translation() = stdPoint + rot * (_jointPosition.translation() - stdPoint);
				_bodyPosition.translation() = stdPoint + rot * (_bodyPosition.translation() - stdPoint);

				if (name == "Torso"){
					if (_name[0]=='L'){
						_jointPosition.translation() += shiftVector * lengthRatio;
						_bodyPosition.translation() += shiftVector * lengthRatio;
					}else if (_name[0]=='R'){
						_jointPosition.translation() -= shiftVector * lengthRatio;
						_bodyPosition.translation() -= shiftVector * lengthRatio;
					}
				}
				else {
					_jointPosition.translation() += shiftVector * lengthRatio;
					_bodyPosition.translation() += shiftVector * lengthRatio;
				}

				// axis rotating
				_axis = rot * _axis;

				// element fixing
				_bodyPosElem->SetAttribute("linear",toString(_bodyPosition));
				_bodyPosElem->SetAttribute("translation",toString(Eigen::Vector3d(_bodyPosition.translation())));
				if(_jointPosElem->Attribute("linear")!=nullptr)
					_jointPosElem->SetAttribute("linear",toString(_jointPosition));
				_jointPosElem->SetAttribute("translation",toString(Eigen::Vector3d(_jointPosition.translation())));

				if(_body->Attribute("axis")!=nullptr)
					_body->SetAttribute("axis", toString(_axis));


				_name = _body->Attribute("name");

			}
		}

	}



	TiXmlPrinter printer;
	printer.SetIndent( "\n" );

	outputDoc.Accept( &printer );
	outputDoc.SaveFile(outputSkeletonXmlPath);
}