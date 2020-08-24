#include <iostream>
#include <map>
#include <vector>
#include <queue>
#include <tinyxml.h>
#include <fstream>
#include <string.h>
#include <string>
#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;

FILE *in=fopen("input.xml","r"), *out=fopen("muscle_params.xml","w");

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

std::string toString(const vector<double>& v)
{
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

int cnt=0;
int main() {
	TiXmlDocument inputDoc, outputDoc;
	if (!inputDoc.LoadFile("../input.xml")) {
		std::cout << "Can't open file : " << "../input.xml" << std::endl;
		return 0;
	}

	TiXmlElement *outputMuscleDoc = new TiXmlElement("Muscle");
	outputDoc.LinkEndChild(outputMuscleDoc);

	TiXmlElement *muscleDoc = inputDoc.FirstChildElement("Muscle");

	for (TiXmlElement *unitElem = muscleDoc->FirstChildElement("Unit"); unitElem!=nullptr; unitElem = unitElem->NextSiblingElement("Unit")){
		if (unitElem->Attribute("name")[0] == 'L') continue;

		string name = unitElem->Attribute("name");

		TiXmlElement *right = new TiXmlElement("Unit");
		right->SetAttribute("name", name);
		right->SetAttribute("f0", unitElem->Attribute("f0"));
		right->SetAttribute("lm", unitElem->Attribute("lm"));
		right->SetAttribute("lt", unitElem->Attribute("lt"));
		right->SetAttribute("pen_angle", unitElem->Attribute("pen_angle"));

		TiXmlElement *left = new TiXmlElement("Unit");
		left->SetAttribute("name", "L" + name.substr(1));
		left->SetAttribute("f0", unitElem->Attribute("f0"));
		left->SetAttribute("lm", unitElem->Attribute("lm"));
		left->SetAttribute("lt", unitElem->Attribute("lt"));
		left->SetAttribute("pen_angle", unitElem->Attribute("pen_angle"));

		for (TiXmlElement *wpElem = unitElem->FirstChildElement("Waypoint");
			 wpElem != nullptr; wpElem = wpElem->NextSiblingElement("Waypoint")) {

			string body = wpElem->Attribute("body");
			vector<double> p = split_to_double(wpElem->Attribute("p"), 3);

			TiXmlElement *rwp = new TiXmlElement("Waypoint");
			rwp->SetAttribute("body", wpElem->Attribute("body"));
			rwp->SetAttribute("p", wpElem->Attribute("p"));
			rwp->SetAttribute("idx", wpElem->Attribute("idx"));
			right->LinkEndChild(rwp);

			p[0] *= -1;

			TiXmlElement *lwp = new TiXmlElement("Waypoint");
			if (body[0]=='R'){
				lwp->SetAttribute("body", "L"+body.substr(1));
			}else{
				lwp->SetAttribute("body", body);
			}
			lwp->SetAttribute("p", toString(p));
			lwp->SetAttribute("idx", wpElem->Attribute("idx"));
			left->LinkEndChild(lwp);
		}

		outputMuscleDoc->LinkEndChild(left);
		outputMuscleDoc->LinkEndChild(right);

	}

	/*for (TiXmlElement *wpElem = muscleDoc->FirstChildElement("Waypoint");
		 wpElem != nullptr; wpElem = wpElem->NextSiblingElement("Waypoint")) {
		waypointVec.push_back(wpElem);
	}
	for (TiXmlElement *leftMuscle = muscleDoc->FirstChildElement("Unit");
		 leftMuscle != nullptr; leftMuscle = leftMuscle->NextSiblingElement("Unit")) {
		if (leftMuscle->Attribute("name")[0] == 'R') continue;
		TiXmlElement *rightMuscle = leftMuscle->NextSiblingElement("Unit");

		vector<int> lwp = split_to_int(leftMuscle->Attribute("indices"));
		vector<int> rwp = split_to_int(rightMuscle->Attribute("indices"));

		cout << leftMuscle->Attribute("name") << endl;

		for (int i = 0; i < lwp.size(); i++) {
			vector<double> rp = split_to_double(waypointVec[rwp[i]]->Attribute("p"), 3);
			rp[0] *= -1;
			waypointVec[lwp[i]]->SetAttribute("p", toString(rp));
		}
		leftMuscle = leftMuscle->NextSiblingElement("Unit");
	}
*/

	TiXmlPrinter printer;
	printer.SetIndent("\n");

	outputDoc.Accept(&printer);
	outputDoc.SaveFile("../output.xml");
	return 0;
}
