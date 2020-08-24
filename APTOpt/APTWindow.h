//
// Created by minseok on 11/22/18.
//

#ifndef APTWINDOW_H
#define APTWINDOW_H
#include "dart/dart.hpp"
#include "../render/SimWindow.h"
#include "../world/IntegratedWorld.h"
#include "../visualROM/MSSROM.h"
#include "APTOptimizer.h"
// #include "../nnChecker/nnChecker.h"
#include <math.h>
#include <string>
#include <stdio.h>
#include <map>

class APTWindow : public SimWindow{
public:
    // Render Part
	APTWindow();
    void InitROM();
    void InitSkelRertargeting();
    void InitROMEditing();
    void InitAPTRetargeting();
    void Keyboard(unsigned char key,int x,int y) override;
    void Timer(int value) override;
    void Display() override;

    //Saved ROM in ROM Window
    MSSROM* MSSROM_std;
    MSSROM* MSSROM_rtg;


    //APT opimizer
    APTOptimizer* aptOptimizer;

    //we will recalculate/ conicEdit rom just one time for change.
    bool poseChanged;
    bool romChanged;

    int mShiftMouseType;
	int selectedROM;
    //prev_x, prev_y dosen't change in a drag
    int prev_x;
    int prev_y;


    // Editing
	std::map<int, std::string> ROMNameMap;
	bool isEditing;
	int editIndex; // 0:theta, 1:phi, 2:roll
	BodyROM* editingROM;

    void drawAPTGraph(std::shared_ptr<MusculoSkeletalSystem> ms, std::string label = "");

    void checkTorque(std::shared_ptr<MusculoSkeletalSystem> ms);


    //----------------------Skeleton Retaretting
    void readJointMap();
    void executePrintingNormalizedLmt(std::string sm_name, std::string muscle_name);

    // Simple Motion
    int smIdx, smStep, stepIdx;
    int getMuscleIdx(std::string muscle_name);
    int getSimpleMotionIdx(std::string sm_name);
    std::map<std::string, int> jointMap;
    std::vector<SimpleMotion> simpleMotions;
    void readSimpleMotion();

    // Re-target Skeleton Info
    SkeletonInfo rtgSkeletonInfo;

    // temporarily variable and functions
    int thetaIdx, phiIdx, rollIdx;
	int thetaIdx2, phiIdx2, rollIdx2;
    double theta, phi, roll;
	void setStdPose(double *config){
		auto& stdSkel = mWorld->GetMusculoSkeletalSystem()->getSkeleton();
		for (int i=0;i<stdSkel->getNumDofs();i++){
			stdSkel->setPosition(i, config[i]);
		}
	}

    Eigen::Vector3d bvhPositionOffset;

    int ratioCnt;

};
#endif //EXAM_EXAMWINDOW_