//
// Created by minseok on 12/13/18.
//

#ifndef ROMEditor_H
#define ROMEditor_H
#include "../model/MusculoSkeletalSystem.h"
#include "../generator/motionAdjust/BVHparser.h"
#include "dart/dart.hpp"

class EditInfoDB
{
public:
	EditInfoDB();
	// Set center joint to body vector
	void setCenterJ2B_vector(std::string body_name, Eigen::Vector3d centerJ2B);

	// Set center v vector for roll rotation
	void setCenterV_vector(std::string body_name, Eigen::Vector3d centerV);

	// Set center revolute angle value
	void setRevCenter(std::string body_name, double rev_center);

	// Set Editing information for ball-and-socket joint. This should be called after setCenterJ2B_vector.
	void setEditInfo(std::string body_name, Eigen::Vector3d new_centerJ2B, double centerJ2B_scale, double roll_trans, double roll_scale);

	// Set Editing information for revolute joint. This should be called after setRevCenter.
	void setEditInfo(std::string body_name, double rev_trans, double rev_scale);

	// Find the index of the body if not exist, return -1.
	int findBodyIndex(std::string body_name);

	// Reset the editing info. not implemented
	void reset();

	std::vector<std::string> body_name_vector;

	// For ball-and-socket joint
	std::vector<Eigen::Vector3d> centerJ2B_vector;
	std::vector<Eigen::Vector3d> centerV_vector;
	std::vector<Eigen::Vector3d> new_centerJ2B_vector;
	std::vector<double> centerJ2B_scale_vector;
	std::vector<double> roll_trans_vector;
	std::vector<double> roll_scale_vector;


	// For revolute joint
	std::vector<double> rev_center_vector;
	std::vector<double> rev_trans_vector;
	std::vector<double> rev_scale_vector;
};

namespace ROMEditor{
	// Change the value of muscle->max_l_mt for edited ROM.
	void editMPwithROM(std::shared_ptr<MusculoSkeletalSystem>& ms, EditInfoDB* edb);

	// Displacement mapping for a bvh motion pose.
	void setPositionFromFilteredBVH(std::shared_ptr<MusculoSkeletalSystem>& ms, BVHparser *bvhParser, int bvh_frame, EditInfoDB* edb);
}
#endif