#ifndef __DART_HELPER_H__
#define __DART_HELPER_H__
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"

enum JOINT_TYPE
{
	WELD,
	REVOLUTE,
	UNIVERSAL,
	EULER,
	BALL_AND_SOCKET,
	FREE
};

enum POSE_TYPE
{
	STAND,
	SUPINE,
	PRONE
};
dart::dynamics::BodyNode* MakeWeldJointBody(
		const std::string& body_name,
		const std::string& obj_file,
		const dart::dynamics::SkeletonPtr& target_skel,
		dart::dynamics::BodyNode* const parent,
		const Eigen::Vector3d& size,
		const Eigen::Isometry3d& joint_position,
		const Eigen::Isometry3d& body_position,
		double mass,
		bool contact,
		bool obj_visual_consensus = false);
dart::dynamics::BodyNode* MakePrismaticJointBody(
		const std::string& body_name,
		const std::string& obj_file,
		const dart::dynamics::SkeletonPtr& target_skel,
		dart::dynamics::BodyNode* const parent,
		const Eigen::Vector3d& size,
		const Eigen::Isometry3d& joint_position,
		const Eigen::Isometry3d& body_position,
		bool isLimitEnforced,
		double upper_limit,
		double lower_limit,
		double mass,
		const Eigen::Vector3d& axis,
		bool contact,
		bool obj_visual_consensus = false);
dart::dynamics::BodyNode* MakeRevoluteJointBody(
		const std::string& body_name,
		const std::string& obj_file,
		const dart::dynamics::SkeletonPtr& target_skel,
		dart::dynamics::BodyNode* const parent,
		const Eigen::Vector3d& size,
		const Eigen::Isometry3d& joint_position,
		const Eigen::Isometry3d& body_position,
		bool isLimitEnforced,
		double upper_limit,
		double lower_limit,
		double mass,
		const Eigen::Vector3d& axis,
		bool contact,
		bool obj_visual_consensus = false);
dart::dynamics::BodyNode* MakeBallJointBody(
		const std::string& body_name,
		const std::string& obj_file,
		const dart::dynamics::SkeletonPtr& target_skel,
		dart::dynamics::BodyNode* const parent,
		const Eigen::Vector3d& size,
		const Eigen::Isometry3d& joint_position,
		const Eigen::Isometry3d& body_position,
		bool isLimitEnforced,
		const Eigen::Vector3d& upper_limit,
		const Eigen::Vector3d& lower_limit,
		double mass,
		bool contact,
		bool obj_visual_consensus = false);
dart::dynamics::BodyNode* MakeFreeJointBody(
		const std::string& body_name,
		const std::string& obj_file,
		const dart::dynamics::SkeletonPtr& target_skel,
		dart::dynamics::BodyNode* const parent,
		const Eigen::Vector3d& size,
		const Eigen::Isometry3d& joint_position,
		const Eigen::Isometry3d& body_position,
		double mass,
		bool contact,
		bool obj_visual_consensus = false);
#endif