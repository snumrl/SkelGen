#include <iostream>
#include <tinyxml.h>
#include <string>
#include <map>
#include <fstream>
using namespace std;

map<string, string> XMLNaming = {{"Pelvis", "Pelvis"},
								 {"R_Femur", "FemurR"},
								 {"L_Femur","FemurL"},
								 {"R_Tibia","TibiaR"},
								 {"L_Tibia","TibiaL"},
								 {"R_Foot","TalusR"},
								 {"L_Foot","TalusL"},
								 {"Spine","Spine"},
								 {"Torso","Torso"},
								 {"Neck","Neck"},
								 {"Head","Head"},
								 {"R_Shoulder","ShoulderR"},
								 {"R_Arm","ArmR"},
								 {"R_ForeArm","ForeArmR"},
								 {"R_Hand","HandR"},
								 {"L_Shoulder","ShoulderL"},
								 {"L_Arm","ArmL"},
								 {"L_ForeArm","ForeArmL"},
								 {"L_Hand","HandL"},
								 {"None","None"}};

int main() {
	TiXmlDocument inputDoc, outputDoc;
	if(!inputDoc.LoadFile("../Rhs.xml")){
		std::cout << "Can't open file : " << "../Rhs.xml" << std::endl;
		return 0;
	}
	TiXmlElement *skelDoc = inputDoc.FirstChildElement("Skeleton");

	for(TiXmlElement *body = skelDoc->FirstChildElement("Joint"); body != nullptr; body = body->NextSiblingElement("Joint")) {
		std::string name = body->Attribute("name");
		std::string parent_name = body->Attribute("parent_name");
		body->SetAttribute("name", XMLNaming[name]);
		body->SetAttribute("parent_name", XMLNaming[parent_name]);
	}
	TiXmlPrinter printer;
	printer.SetIndent( "\n" );

	inputDoc.Accept( &printer );
	inputDoc.SaveFile("../Lsw.xml");
	return 0;
}