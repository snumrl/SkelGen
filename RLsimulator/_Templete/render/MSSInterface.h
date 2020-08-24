#ifndef __MSS_INTERFACE_H__
#define __MSS_INTERFACE_H__
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"
#include "GLfunctions.h"
#include "Character.h"
namespace GUI
{
void DrawSkeleton(
	const dart::dynamics::SkeletonPtr& skel,
	const Eigen::Vector3d& color=Eigen::Vector3d(1.1,1.1,1.1),
	bool drawBox=false);
void DrawMuscles(const std::vector<MSS::Muscle*>& muscles);

void DrawGround(double y);
void DrawShape(const Eigen::Isometry3d& T,
	const dart::dynamics::Shape* shape,
	const Eigen::Vector3d& color);
};


#endif