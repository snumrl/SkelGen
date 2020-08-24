//
// Created by hoseok on 9/16/18.
//

#ifndef EXAM_EXAMWINDOW_H
#define EXAM_EXAMWINDOW_H

#include <render/SimWindow.h>
#include <math.h>
#include <string>
#include <map>
#include <stdio.h>
#include "../simpleMotion/SimpleMotion.h"
#include "../visualROM/MSSROM.h"
#include "../romEditor/ROMEditor.h"

enum MODE{
	SKELETON_EDIT, MUSCLE_GENERATE, MUSCLE_CALIBRATE, ROM_EDIT, MUSCLE_PARAMETER_ADAPTING
};


class EditorWindow : public SimWindow{
public:
	EditorWindow();

	void Keyboard(unsigned char key,int x,int y) override;
	void Timer(int value) override;
	void Motion(int x, int y) override;
	void Mouse(int button, int state, int x, int y) override;

	void readJointMap();
	void drawGraph();
	void executePrintingNormalizedLmt(std::string sm_name, std::string muscle_name);

	void setStdPose(double *config);
	void setRtgPose(double *config);

	void Display() override;

	void computeROMs();
	void loadROMs();

	bool isEditableSkeleton(int ID);
	bool isEditableROM(int ID);

	bool isBallROM(int ID);
	bool isRevoluteROM(int ID);

	// UI Process
	MODE mode;

	// Editing
	std::map<int, std::string> ID2Name;
	std::map<std::string, int> name2ID;

	// Skeleton Editing
	int selectedSkeletonID;
	bool isEditingSkeleton;
	int editingSkeletonID; // 0:theta, 1:phi, 2:roll


	// ROM Editing
	int selectedROMID;
	bool isEditingROM;
	BodyROM* editingROM;


	// Simple Motion
	int smIdx, smStep, stepIdx;
	int getMuscleIdx(std::string muscleName);
	int getSimpleMotionIdx(std::string sm_name);
	std::map<std::string, int> jointMap;
	std::vector<SimpleMotion> simpleMotions;
	void readSimpleMotion();
	void findSimpleMotionRelatedMuscleIndices();

	void updateTargetSkeleton();

	int selectObject(GLint x, GLint y);

	MSSROM* ROM;

	int prevX, prevY;

	// Mouse user interaction for conic translation
	void moveROMSphere(int x, int y);

	// Mouse user interaction for conic translation
	void moveRevoluteArc(int x, int y);

	// Mouse user interaction for conic scaleing
	void scaleROMSphere(bool up);

	// Mouse user interaction for conic scaleing
	void scaleRevoluteArc(bool up);

	// Convert the format of user input to edb for new muscle parameter fitting
	void convertUserInputIntoEDB();
	void tempSaveCurrent();
	Eigen::Vector3d prev_cur_centerJ2B;
	double			prev_translateRev;
	Eigen::Vector3d getMouseRayVector(int x, int y);
	Eigen::Vector3d pointOnPlane(Eigen::Vector3d ray, Eigen::Vector3d center, Eigen::Vector3d normal);

	bool romSphereDrag;

	void setInitialCameraView();

	// TEMP
	bool mIsDrawingSkeleton;
	bool loadFlag;

	//controls
    double R_Femur;
    double R_Tibia;

    bool nonMuscleOn;
};


#endif //EXAM_EXAMWINDOW_H
