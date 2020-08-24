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
#include "testset.h"
#include "../simpleMotion/SimpleMotion.h"
#include "../generator/SkeletonInfo.h"


class ExamWindow : public SimWindow{
public:
	ExamWindow();
    ExamWindow(TEST_TYPE test_type);

    void Keyboard(unsigned char key,int x,int y) override;
    void Timer(int value) override;

    void readJointMap();
	void drawGraph();
    void executePrintingNormalizedLmt(std::string sm_name, std::string muscle_name);

    void setStdPose(double *config);
    void setRtgPose(double *config);


    TEST_TYPE test_type;


    // Simple Motion
	int smIdx, smStep, stepIdx;
    int getMuscleIdx(std::string muscle_name);
    int getSimpleMotionIdx(std::string sm_name);
    std::map<std::string, int> jointMap;
	std::vector<SimpleMotion> simpleMotions;
	void readSimpleMotion();

	// Re-target Skeleton Info
	SkeletonInfo rtgSkeletonInfo;
	void updateTargetSkeleton();

	// Temporarily
	double posX, posZ;
};


#endif //EXAM_EXAMWINDOW_H
