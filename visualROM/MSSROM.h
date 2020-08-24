//
// Created by minseok on 12/15/18.
//

#ifndef MSSROM_H
#define MSSROM_H
#include "dart/dart.hpp"
#include "../render/SimWindow.h"
#include "../world/IntegratedWorld.h"
#include "../romEditor/ROMEditor.h"
#include <math.h>
#include <string>
#include <stdio.h>

const int numTheta = 18;
const int numPhi = 36;
const int numRoll = 36;
const int numRev = 36;

#define NOTHING -1
#define ID_Pelvis 0
#define ID_R_Femur 1
#define ID_R_Tibia 3
#define ID_R_Foot 5
#define ID_L_Femur 2
#define ID_L_Tibia 4
#define ID_L_Foot 6
#define ID_Spine 7
#define ID_Torso 8
#define ID_Neck 9
#define ID_Head 10
#define ID_L_Shoulder 15
#define ID_L_Arm 16
#define ID_L_ForeArm 17
#define ID_L_Hand 18
#define ID_R_Shoulder 11
#define ID_R_Arm 12
#define ID_R_ForeArm 13
#define ID_R_Hand 14

class EditInfo{
public:
	EditInfo(){
		translate = 0;
		scale = 1.0;
	}
	double translate;
	double scale;
	double center;
	double prev_center;
};

class BodyROM
{
public:
    BodyROM(std::string name, JOINT_TYPE type);
    std::string name;
    int data_ball[numTheta+1][numPhi][numRoll];
    int data_revolute[numRev+1];
    Eigen::Vector3d lowerLimit, upperLimit;
    JOINT_TYPE type;

    // Editing Information
	EditInfo revoluteEditInfo; // axial for ball & socket, theta for revolute
	Eigen::AngleAxisd conicTranslate;
	double conicScale;

	Eigen::Vector3d conicEdit(Eigen::Vector3d vec);
	double revoluteEdit(double angle);

	// Try j2b vector average
	Eigen::Vector3d centerJ2B;

	Eigen::Vector3d cur_centerJ2B;
	Eigen::Vector3d prev_centerJ2B;
	double cur_roll;

	Eigen::Vector3d comfortablePositionBall;
	double comfortablePositionRevolute;


	int ID;
};

class MSSROM
{
public:
    MSSROM(std::shared_ptr<MusculoSkeletalSystem>& ms);
    MSSROM(std::shared_ptr<MusculoSkeletalSystem>& ms, EditInfoDB* edb);
    void InitROM();
    void InitEDB();
    void ComputeROMCenter(std::string body_name, bool nonMuscleOn = true);

    void ComputeROM(std::string body_name, bool nonMuscleOn = true);
    void LoadROM(std::string body_name);
    BodyROM* getROM(std::string body_name);

	void setPose(std::string body_name, double theta, double phi=0.0, double roll=0.0);
	void setPose(std::string body_name, Eigen::Vector3d desired_j2b, double roll);
	Eigen::Vector3d getJ2BfromThetaPhi(std::string body_name, double theta, double phi);
	Eigen::Quaterniond getRotation(std::string body_name, double theta, double phi, double roll);

	void setComfortablePosition();
	void setComfortablePosition(std::string body_name);


    std::shared_ptr<MusculoSkeletalSystem> ms;
    std::vector<BodyROM*> BodyROMs;

    EditInfoDB* edb;

};

bool IsBallJoint(std::string body_name);
bool IsMajorBallJoint(std::string body_name);

#endif