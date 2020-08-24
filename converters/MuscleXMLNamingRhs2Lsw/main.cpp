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
		std::cout << "Can't open file : " << "../Lsw.xml" << std::endl;
		return 0;
	}
	TiXmlElement *muscleDoc = inputDoc.FirstChildElement("Muscle");

	for(TiXmlElement* muscle = muscleDoc->FirstChildElement("Unit");muscle!=nullptr;muscle = muscle->NextSiblingElement("Unit"))
	{
		string indices = "";
		for (TiXmlElement* waypoint = muscle->FirstChildElement("Waypoint"); waypoint!=nullptr; waypoint = waypoint->NextSiblingElement("Waypoint")) {
			std::string body = waypoint->Attribute("body");
			waypoint->SetAttribute("body", XMLNaming[body]);
		}
	}
	TiXmlPrinter printer;
	printer.SetIndent( "\n" );

	inputDoc.Accept( &printer );
	inputDoc.SaveFile("../Lsw.xml");
	return 0;
}
