//
// Created by hoseok on 12/4/18.
//

#include "functions.h"
#include <fstream>
#include <iostream>
#include <stdio.h>
using namespace std;
std::vector<double> split_to_double(const std::string& input, int num)
{
	std::vector<double> result;
	std::string::size_type sz = 0, nsz = 0;
	for(int i = 0; i < num; i++){
		result.emplace_back(std::stof(input.substr(sz), &nsz));
		sz += nsz;
	}
	return result;
}
std::vector<int> split_to_int(const std::string& input)
{
	std::vector<int> result;
	std::string::size_type lastPos = input.find_first_not_of(" ", 0);
	std::string::size_type pos     = input.find_first_of(" ", lastPos);

	while (std::string::npos != pos || std::string::npos != lastPos)
	{
		result.emplace_back(stoi(input.substr(lastPos, pos - lastPos)));
		lastPos = input.find_first_not_of(" ", pos);
		pos = input.find_first_of(" ", lastPos);
	}
	return result;
}
Eigen::Vector3d string_to_vector3d(const std::string& input){
	std::vector<double> v = split_to_double(input, 3);
	Eigen::Vector3d res;
	res << v[0], v[1], v[2];

	return res;
}
std::vector<int> string_to_int_array(const std::string& input){
	return split_to_int(input);
}

Eigen::Matrix3d string_to_matrix3d(const std::string& input){
	std::vector<double> v = split_to_double(input, 9);
	Eigen::Matrix3d res;
	res << v[0], v[1], v[2],
			v[3], v[4], v[5],
			v[6], v[7], v[8];

	return res;
}

Eigen::Vector3d Proj(const Eigen::Vector3d& u,const Eigen::Vector3d& v)
{
	Eigen::Vector3d proj;
	proj = u.dot(v)/u.dot(u)*u;
	return proj;
}
Eigen::Isometry3d Orthonormalize(const Eigen::Isometry3d& T_old)
{
	Eigen::Isometry3d T;
	T.translation() = T_old.translation();
	Eigen::Vector3d v0,v1,v2;
	Eigen::Vector3d u0,u1,u2;
	v0 = T_old.linear().col(0);
	v1 = T_old.linear().col(1);
	v2 = T_old.linear().col(2);

	u0 = v0;
	u1 = v1 - Proj(u0,v1);
	u2 = v2 - Proj(u0,v2) - Proj(u1,v2);

	u0.normalize();
	u1.normalize();
	u2.normalize();

	T.linear().col(0) = u0;
	T.linear().col(1) = u1;
	T.linear().col(2) = u2;
	return T;
}


std::string toString(const std::vector<int> v){
	std::string ret ="";
	for(int i=0;i<v.size();i++){
		ret += std::to_string(v[i]);
		ret += " ";
	}
	return ret;
}

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

std::string toString(const Eigen::Isometry3d& v){
	std::string ret="";
	ret += std::to_string(v.linear().row(0)[0]) + " ";
	ret += std::to_string(v.linear().row(0)[1]) + " ";
	ret += std::to_string(v.linear().row(0)[2]) + " ";
	ret += std::to_string(v.linear().row(1)[0]) + " ";
	ret += std::to_string(v.linear().row(1)[1]) + " ";
	ret += std::to_string(v.linear().row(1)[2]) + " ";
	ret += std::to_string(v.linear().row(2)[0]) + " ";
	ret += std::to_string(v.linear().row(2)[1]) + " ";
	ret += std::to_string(v.linear().row(2)[2]) + " ";
	return ret;
}

void ReadUserInput(std::vector<MetaBoneInfo>& mbis, std::string path){
	std::ifstream ifs(path);
	if(!(ifs.is_open()))
	{
		std::cout<<"Can't read file "<<path<<std::endl;
		return;
	}
	std::string str;
	std::string index;
	std::stringstream ss;

	mbis.clear();

	double l, s, t;
	std::string name;
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
		else if(index=="l")
		{
			ss>>l;
		}
		else if(index=="s")
		{
			ss>>s;
		}
		else if(index=="t")
		{
			ss>>t;
			mbis.emplace_back(MetaBoneInfo(name, l, s, t));
		}else if (index=="!"){
			break;
		}
	}

	ifs.close();
}


Eigen::Quaterniond makeQuaternion(double theta, double phi, double roll){
	// order : y thetaIdx -> z phiIdx -> axial rollIdx
	Eigen::AngleAxisd aaTheta(theta, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd aaPhi(phi, Eigen::Vector3d::UnitY());

	Eigen::Vector3d axis = aaPhi * aaTheta * Eigen::Vector3d(0,1,0);
	Eigen::AngleAxisd aaRoll(roll, axis);

	Eigen::Quaterniond quatTheta(aaTheta);
	Eigen::Quaterniond quatPhi(aaPhi);
	Eigen::Quaterniond quatRoll(aaRoll);

	return quatRoll*quatPhi*quatTheta;
}

Eigen::Quaterniond makeConicQuaternion(double theta, double phi, Eigen::Vector3d thetaAxis, Eigen::Vector3d phiAxis){
	// order : y thetaIdx -> z phiIdx -> axial rollIdx
	Eigen::AngleAxisd aaTheta(theta, thetaAxis);
	Eigen::AngleAxisd aaPhi(phi, phiAxis);

	Eigen::Quaterniond quatTheta(aaTheta);
	Eigen::Quaterniond quatPhi(aaPhi);

	return quatPhi*quatTheta;
}

Eigen::Quaterniond makeAxialQuaternion(Eigen::Vector3d axis, double roll){
	Eigen::AngleAxisd aaRoll(roll, axis);

	Eigen::Quaterniond quatRoll(aaRoll);

	return quatRoll;
}

Eigen::Vector3d rotate(Eigen::Vector3d p, double theta, double phi, double roll){
	Eigen::Quaterniond rot = makeQuaternion(theta, phi, roll);
	Eigen::Quaterniond _p(0,p[0],p[1],p[2]);
	return (rot*_p*rot.inverse()).vec();
}