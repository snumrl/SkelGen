//
// Created by minseok on 11/09/18.
//
#include <Eigen/Geometry>
#include "BVHparser.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <algorithm>
#define M_PI 3.14159265358979323846

using namespace std;

MotionNode::MotionNode()
{
	name = "";
	axisOrder = "";
	isRoot = false;
	isEnd = false;
	parent = nullptr;
	next = nullptr;
	childs = vector<MotionNode*>();
	channelNum = 0;
}
MotionNode* MotionNode::getParent()
{
	return parent;
}
vector<MotionNode*> MotionNode::getChilds()
{
	return childs;
}
void MotionNode::setParent(MotionNode* pnode)
{
	parent = pnode;
	pnode->addChild(this);
}
void MotionNode::addChild(MotionNode* cnode)
{
	childs.push_back(cnode);
}
void MotionNode::setRoot()
{
	isRoot = true;
}
void MotionNode::setEnd()
{
	isEnd = true;
}
void MotionNode::setName(string mname)
{
	name = mname;
}
void MotionNode::setAxisOrder(string maxisOrder)
{
	axisOrder = maxisOrder;
}
void MotionNode::setOffset(float x, float y, float z)
{
	offset[0] = x;
	offset[1] = y;
	offset[2] = z;
}
void MotionNode::setNext(MotionNode *nextNode)
{
	next = nextNode;
}
void MotionNode::setChannelNum(int mchannelNum)
{
	channelNum = mchannelNum;
}
bool MotionNode::checkEnd()
{
	return isEnd;
}
string MotionNode::getName_std()
{
	if(match_name_list.size()>=1)
		return match_name_list[0];
	else
	{
		cout<<"no such name in list"<<name<<endl;
		return "";
	}
}
string MotionNode::getName()
{
	return name;
}
string MotionNode::getAxisOrder()
{
	return axisOrder;
}
void MotionNode::getOffset(float* outOffset)
{
	for(int i=0;i<3;i++)
	{
		outOffset[i] = offset[i];
	}
}
float MotionNode::getOffset(int index)
{
	return offset[index];
}
int MotionNode::getChannelNum()
{
	return channelNum;
}
MotionNode* MotionNode::getNextNode()
{
	return next;
}
void MotionNode::initData(int frames)
{
	data = new float*[frames];
	for(int i = 0; i < frames; i++)
	{
		data[i] = new float[channelNum];
	}
}
bool MotionNode::ContainedInModel()
{
	if(match_name_list.size()==0)
		return false;
	else
		return true;
}
/// convert quaternion -> log(quaternion)
Eigen::Vector3d QuaternionToAngleAxis(Eigen::Quaterniond qt)
{
	double angle = atan2(qt.vec().norm(), qt.w());
	if(std::abs(angle) < 1e-4)
		return Eigen::Vector3d::Zero();
	return angle * qt.vec().normalized();
}

/// convert log(quaternion) -> quaternion
Eigen::Quaterniond AngleAxisToQuaternion(Eigen::Vector3d angleAxis)
{
	Eigen::Quaterniond qt;
	if(angleAxis.norm() < 1e-4)
		return Eigen::Quaterniond(1,0,0,0);
	qt.vec() = angleAxis.normalized() * sin(angleAxis.norm());
	qt.w() = cos(angleAxis.norm());
	return qt;
}

void BVHparser::InitMatchNameListForMotionNode(const char* path)
{
	string line;
	stringstream s;
	ifstream in(path);
    // std::cout<<"# reading file "<<path<<std::endl;
	string muscleName_std;
	string muscleName;
	MotionNode* curNode = getRootNode();
	float value;
	while(!in.eof())
	{
		getline(in, line);
		if(line.empty())
			break;
		s = stringstream(line);
		std::vector<string> match_name_list;
		while(!s.eof())
		{
			s>>muscleName;
			match_name_list.push_back(muscleName);
		}
		curNode = getRootNode();
		while(curNode!=NULL)
		{
			for(auto& name : match_name_list)
			{
				if(curNode->getName() == name)
				{
					curNode->match_name_list = match_name_list;
					break;
				}
			}
			curNode = curNode->getNextNode();
		}
	}
	in.close();
}

BVHparser::BVHparser(const char* path)
{
	this->scale = 1.0;
	int lineNum = 0;
	int channelNum = 0;
	string channels[6];
	float offx, offy, offz;
	mPath = path;
	lowerBodyOnly = false;
	upperBodyOnly = false;
	ifstream in;
	in.open(path, ios::in);
	if(!in)
	{
		cerr << "Cannot open "<<path<<endl; exit(1);
	}
	string line;
	getline(in, line);										//HIERARCHY
	lineNum++;

	if(line.compare(0,9,"HIERARCHY")!=0)
	{ 
		cout << "Check the file format. The line number "<<lineNum<<" is not fit to the format"<<endl;
		cout<<line.compare("HIERARCHY")<<endl;
		cout<<line.length()<<endl;
		cout << line<<endl;
		exit(-1);
	}
	getline(in, line);										//ROOT Hips
	lineNum++;
	rootNode = new MotionNode();
	rootNode->setRoot();
	istringstream s(line);
	string bvh_keyword;
	string bvh_nodeName;
	s >> bvh_keyword; s >> bvh_nodeName;
	if(bvh_keyword.compare("ROOT")!=0)
	{
		cout << "Check the file format. The line number "<<lineNum<<" is not fit to the format"<<endl;
		cout << line<<endl;
		exit(-1);
	}
	rootNode->setName(bvh_nodeName);
	getline(in, line);										//{

	getline(in, line);										//	OFFSET 0.00 0.00 0.00
	s.str("");
	s = istringstream(line);
	s >> bvh_keyword; s >> offx; s >> offy; s >> offz;
	rootNode->setOffset(offx, offy -90.0, offz);


	getline(in, line);										//	CHANNELS 6 Xposition Yposition Zposition Xrotation Yrotation Zrotation
	s.str("");
	s = istringstream(line);
	s >> bvh_keyword; s >> channelNum;
	string newAxisOrder = "";
	for(int i = 0; i < channelNum; i++)
	{
		s >> channels[i];
		if(channels[i].substr(1) == "rotation")
		{
			transform(channels[i].begin(), channels[i].end(), channels[i].begin(), ::tolower);
			newAxisOrder += channels[i][0];
		}
	}
	rootNode->setAxisOrder(newAxisOrder);
	rootNode->setChannelNum(channelNum);
	MotionNode* prevNode = rootNode;
	MotionNode* prevNode4NextNode = rootNode;
	getline(in, line);
	while(line.compare(0, 6, "MOTION") != 0)						
	{
		s.str("");
		s = istringstream(line);
		s >> bvh_keyword; s >> bvh_nodeName;
		if(bvh_keyword=="JOINT")							//	JOINT LeftUpLeg
		{
			MotionNode *newNode = new MotionNode();
			newNode->setName(bvh_nodeName);

			getline(in, line);								//	{
			getline(in, line);								//		OFFSET 3.64953 0.00000 0.00000
			s.str("");
			s = istringstream(line);
			s >> bvh_keyword; s >> offx; s >> offy; s >> offz;
			newNode->setOffset(offx, offy, offz);

			getline(in, line);								//		CHANNELS 3 Xrotation Yrotation Zrotation
			s.str("");
			s = istringstream(line);
			s >> bvh_keyword; s >> channelNum;

			newAxisOrder = "";
			for(int i = 0;i < channelNum;i++)
			{
				s >> channels[i];
				if(channels[i].substr(1) == "rotation")
				{
					transform(channels[i].begin(), channels[i].end(), channels[i].begin(), ::tolower);
					newAxisOrder += channels[i][0];
				}
			}
			newNode->setParent(prevNode);
			newNode->setChannelNum(channelNum);
			newNode->setAxisOrder(newAxisOrder);
			prevNode4NextNode->setNext(newNode);
			prevNode = newNode;
			prevNode4NextNode = newNode;
		}
		else if(bvh_keyword =="End")						//	End Site
		{
			MotionNode *newNode = new MotionNode();
			newNode->setName(prevNode->getName()+bvh_nodeName);
			newNode->setEnd();
			getline(in, line);								//	{
			getline(in, line);								//		OFFSET 3.64953 0.00000 0.00000
			s.str("");
			s = istringstream(line);
			s >> bvh_keyword; s >> offx; s >> offy; s >> offz;
			newNode->setOffset(offx, offy, offz);

			newNode->setParent(prevNode);
			getline(in, line);								//	}
		}
		else if(bvh_keyword =="}")
		{
			prevNode = prevNode->getParent();
		}
		else
		{
			cout << "Check the file format. "<< bvh_keyword <<" is right?" <<endl;
			exit(-1);
		}
		getline(in, line);
	}

// 	Start MotionNode										//MOTION
	string str1, str2;	//to get the string for format
	float fvalue;
	getline(in, line);										//Frames: 4692
	s.str("");
	s = istringstream(line);
	s >> str1; s >> fvalue;
	frames = fvalue;

	getline(in, line);										//Frame Time: 0.008333
	s.str("");
	s = istringstream(line);
	s >> str1; s >> str2; s >> fvalue;
	frameTime = fvalue;

	// cout << "frames : " << frames <<", frame time : " << frameTime << endl;

	float f[6];
	MotionNode *curNode;
	curNode = rootNode;
	while(curNode != nullptr)
	{
		curNode->initData(frames);
		curNode = curNode->getNextNode();
	}

	for(int i = 0; i < frames; i++)
	{
		curNode = rootNode;
		getline(in, line);
		s.str("");
		s = istringstream(line);
		while(curNode != nullptr)
		{
			for(int j = 0; j < curNode->getChannelNum(); j++)
			{
				s >> f[j];
				curNode->data[i][j] = f[j];
			}
			curNode = curNode->getNextNode();
		}

	}
	in.close();
	Eigen::Quaterniond q;
	Eigen::Vector3d euler;
	curNode = rootNode;
	for(int i = 0; i < frames; i++)
	{
		q = Eigen::Quaterniond::Identity();
		for(int j = 0; j < 3; j++)
		{
			if(curNode->getAxisOrder().substr(j,1).compare("x") == 0)
			{
				q = q * Eigen::AngleAxisd(curNode->data[i][j+3]/360.0 * 2.0 * M_PI, Eigen::Vector3d::UnitX());
			}
			else if(curNode->getAxisOrder().substr(j,1).compare("y") == 0)
			{
				q = q * Eigen::AngleAxisd(curNode->data[i][j+3]/360.0 * 2.0 * M_PI, Eigen::Vector3d::UnitY());
			}
			else
			{
				q = q * Eigen::AngleAxisd(curNode->data[i][j+3]/360.0 * 2.0 * M_PI, Eigen::Vector3d::UnitZ());
			}
		}
		float theta;
		theta = atan2(sqrt(q.x()*q.x() + q.y()*q.y() + q.z()*q.z()), q.w());
		Eigen::Vector3d root_axis = Eigen::Vector3d(q.x(), q.y(), q.z());
		root_axis.normalize();
		curNode->data[i][3] = theta * root_axis.x();
		curNode->data[i][4] = theta * root_axis.y();
		curNode->data[i][5] = theta * root_axis.z();
	}
	curNode = curNode->getNextNode();
	while(curNode != nullptr)
	{
		for(int i = 0; i < frames; i++)
		{
			q = Eigen::Quaterniond::Identity();
			for(int j = 0; j < 3; j++)
			{
				if(curNode->getAxisOrder().substr(j,1).compare("x") == 0)
				{
					q = q * Eigen::AngleAxisd(curNode->data[i][j]/360.0 * 2.0 * M_PI, Eigen::Vector3d::UnitX());
				}
				else if(curNode->getAxisOrder().substr(j,1).compare("y") == 0)
				{
					q = q * Eigen::AngleAxisd(curNode->data[i][j]/360.0 * 2.0 * M_PI, Eigen::Vector3d::UnitY());
				}
				else
				{
					q = q * Eigen::AngleAxisd(curNode->data[i][j]/360.0 * 2.0 * M_PI, Eigen::Vector3d::UnitZ());
				}
			}


			float theta;
			theta = atan2(sqrt(q.x()*q.x() + q.y()*q.y() + q.z()*q.z()), q.w());
			Eigen::Vector3d root_axis = Eigen::Vector3d(q.x(), q.y(), q.z());
			root_axis.normalize();
			curNode->data[i][0] = theta * root_axis.x();
			curNode->data[i][1] = theta * root_axis.y();
			curNode->data[i][2] = theta * root_axis.z();
		}
		curNode = curNode->getNextNode();
	}
}

MotionNode* BVHparser::getRootNode()
{
	return rootNode;
}
