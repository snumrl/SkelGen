#include <iostream>
#include <tinyxml.h>
#include <string>
#include <map>
#include <fstream>
using namespace std;

map<string, string> XMLNaming = {{"Pelvis", "Pelvis"},
								 {"FemurR", "R_Femur"},
								 {"FemurL","L_Femur"},
								 {"TibiaR","R_Tibia"},
								 {"TibiaL","L_Tibia"},
								 {"TalusR","R_Foot"},
								 {"TalusL","L_Foot"},
								 {"FootThumbR","R_Foot"},
								 {"FootThumbL","L_Foot"},
								 {"FootPinkyR","R_Foot"},
								 {"FootPinkyL","L_Foot"},
								 {"Spine","Spine"},
								 {"Torso","Torso"},
								 {"Neck","Neck"},
								 {"Head","Head"},
								 {"ShoulderR","R_Shoulder"},
								 {"ArmR","R_Arm"},
								 {"ForeArmR","R_ForeArm"},
								 {"HandR","R_Hand"},
								 {"ShoulderL","L_Shoulder"},
								 {"ArmL","L_Arm"},
								 {"ForeArmL","L_ForeArm"},
								 {"HandL","L_Hand"},
								 {"None","None"}};

int main() {
	TiXmlDocument inputDoc, outputDoc;
	if(!inputDoc.LoadFile("../Lsw.xml")){
		std::cout << "Can't open file : " << "../Lsw.xml" << std::endl;
		return 0;
	}
	TiXmlElement *muscleDoc = inputDoc.FirstChildElement("Muscle");

	for(TiXmlElement *wp = muscleDoc->FirstChildElement("Waypoint"); wp != nullptr; wp = wp->NextSiblingElement("Waypoint")) {
		std::string body = wp->Attribute("body");
		wp->SetAttribute("body", XMLNaming[body]);
	}
	TiXmlPrinter printer;
	printer.SetIndent( "\n" );

	inputDoc.Accept( &printer );
	inputDoc.SaveFile("../Rhs.xml");
	return 0;
}
