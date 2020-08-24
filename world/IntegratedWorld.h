#ifndef __INTEGRATED_WORLD_H__
#define __INTEGRATED_WORLD_H__

#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"

#include "../model/DART_helper.h"

#include "../generator/SkeletonGenerator.h"
#include "../generator/MuscleGenerator.h"

#include "../simpleMotion/SimpleMotion.h"

class MusculoSkeletalSystem;

class IntegratedWorld
{
public:
	IntegratedWorld();


	void Initialize(POSE_TYPE pose_type = POSE_TYPE::SUPINE, std::string rtgMuscleFileName="muscle_params.xml");
	void TimeStepping(const Eigen::VectorXd& u = Eigen::VectorXd::Zero(0),bool bake = true);
	void updateTargetSkeleton();
	void updateTargetMuscle();

//	std::shared_ptr<IntegratedWorld> 	Clone();

	const dart::simulation::WorldPtr& GetRigidWorld() {return mRigidWorld;};
	const std::shared_ptr<MusculoSkeletalSystem>& GetMusculoSkeletalSystem(){return mStdMusculoSkeletalSystem;};
	const std::shared_ptr<MusculoSkeletalSystem>& GetCmpMusculoSkeletalSystem(){return mRtgMusculoSkeletalSystem;};
//private:
	dart::simulation::WorldPtr 							mRigidWorld;
	std::shared_ptr<MusculoSkeletalSystem> 				mStdMusculoSkeletalSystem;
	std::shared_ptr<MusculoSkeletalSystem> 				mRtgMusculoSkeletalSystem;

	std::vector<MetaBoneInfo> mbis;
};



#endif