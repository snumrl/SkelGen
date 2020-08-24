//
// Created by minseok on 11/09/18.
//
#ifndef MOTIONADJUST_H
#define MOTIONADJUST_H
#include <boost/filesystem.hpp>
#include <stdlib.h>
#include <memory>
#include "../../world/IntegratedWorld.h"
#include "../../model/MusculoSkeletalSystem.h"
#include "../../render/SimWindow.h"
// #include "../../nnChecker/nnChecker.h"
#include "muscleCal.h"
#include "BVHparser.h"
#include <stdio.h>
#include <vector>
#include <string>
#include <GL/glut.h>
/// Set adjustment value from various bvh files.

class MotionAdjust
{
public:
	MotionAdjust(const char* dir, std::shared_ptr<MusculoSkeletalSystem>& ms);
	void InitBVHLists(const char* dir);
	void SetAdjustmentValueForMotionFile(bool write, bool apply, std::string label_name = "");
	void UpdateMuscleMINMAX_l_mt_list();
	void LoadNewBVHFile();
	void LeftRightSyncAdjustmentValue();
	std::shared_ptr<MusculoSkeletalSystem> ms;
	std::vector<std::string> bvhLists;
	BVHparser* bvhParser;
	int bvh_index;
	int bvh_frame;

	std::vector<double> muscle_max_l_mt_list;
    std::vector<double> muscle_min_l_mt_list;

    // NNChecker *nnChecker;
};
namespace BVHmanager
{
	void SetPositionFromBVH(dart::dynamics::SkeletonPtr skel, BVHparser *bvhParser, int bvh_frame);
}
#endif