#include "IntegratedWorld.h"
#include "../model/DART_helper.h"
#include "../model/MusculoSkeletalSystem.h"
#include <fstream>
#include <sstream>
#include <tinyxml.h>
#include <boost/filesystem.hpp>
#include <stdio.h>
#include "../generator/ObjEditing.h"
#include "../functions/functions.h"

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace std;

IntegratedWorld::
IntegratedWorld()
{
	ReadUserInput(mbis, "../model/userInput.txt");
}
void
IntegratedWorld::
TimeStepping(const Eigen::VectorXd& u,bool bake)
{
#ifdef USE_MUSCLE
	if(u.rows()!=0)
		mStdMusculoSkeletalSystem->SetActivationLevels(u);
//	mStdMusculoSkeletalSystem->transformAttachmentPoints();
#endif
	
	double nn = 5;
	for(int i =0; i<nn;i++)
	{
#ifdef USE_MUSCLE
		//mStdMusculoSkeletalSystem->applyForcesToSkeletons(mSoftWorld);
#else
		if(u.rows()!=0)
		mStdMusculoSkeletalSystem->getSkeleton()->setForces(
			mStdMusculoSkeletalSystem->getSkeleton()->getMassMatrix()*u +
			mStdMusculoSkeletalSystem->getSkeleton()->getCoriolisAndGravityForces());
#endif
#if DO_EXAM
//        printf("DO_EXAM!\n");
#else
        mRigidWorld->step();
#endif
	}

	
}
SkeletonPtr createFloor()
{
	SkeletonPtr floor = Skeleton::create("floor");

	BodyNodePtr body = floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

	std::shared_ptr<BoxShape> box(
		new BoxShape(Eigen::Vector3d(10.0, 0.001, 10.0)));
	auto shapeNode
	= body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
	shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.3,0.3,0.3));

	Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
	tf.translation() = Eigen::Vector3d(0.0, -0.05, 0.0);
	body->getParentJoint()->setTransformFromParentBodyNode(tf);
	body->setRestitutionCoeff(1.0);
	return floor;
}
void
IntegratedWorld::
Initialize(POSE_TYPE pose_type, string rtgMuscleFileName) {
//	SkeletonPtr floor = createFloor();

	cout << "IntegratedWorld Initialization ----- " << endl;

	//Init Rigid world
	mRigidWorld = std::make_shared<dart::simulation::World>();
	mRigidWorld->setGravity(Eigen::Vector3d(0, -9.81, 0));
	mRigidWorld->setTimeStep(1.0 / 1000.0);
	mRigidWorld->checkCollision();

	//Init MusculoSkeletalSystem
	SkeletonInfo srcSkeletonInfo;

	mStdMusculoSkeletalSystem = std::make_shared<MusculoSkeletalSystem>();

//	makeSkeletonMayaTextfile(srcSkeletonInfo, "stdSkeleton.txt", Eigen::Vector3d(0.0,0.9809, -0.0308));
//	makeSkeletonXml("stdSkeleton.txt","stdSkeleton.xml");

//	makeSkeletonLowerBody(mStdMusculoSkeletalSystem, srcSkeletonInfo);
	mStdMusculoSkeletalSystem->mSkeleton = BuildFromFile(string(MSS_ROOT_DIR) + "/model/stdSkeleton.xml", false);
	MakeMuscles(string(MSS_ROOT_DIR) + "/model/standard/muscle/muscle_params.xml", mStdMusculoSkeletalSystem);
	mStdMusculoSkeletalSystem->initialize(mRigidWorld);
	mStdMusculoSkeletalSystem->updateTotalWaypoints();

	makeObjVisualConsensus(mStdMusculoSkeletalSystem->mSkeleton, string(MSS_ROOT_DIR) + "/model/stdSkeleton.xml");


	// make rtgSkeleton.xml
	BuildFromSkeleton(string(MSS_ROOT_DIR) + "/model/stdSkeleton.xml", string(MSS_ROOT_DIR) + "/model/userInput.txt", string(MSS_ROOT_DIR) + "/model/rtgSkeleton.xml", mbis);

	// make rtgSkeleton bone meshes
	boneMeshEditing(string(MSS_ROOT_DIR) + "/model/stdSkeleton.xml", string(MSS_ROOT_DIR) + "/model/rtgSkeleton.xml", string(MSS_ROOT_DIR) + "/model/userInput.txt");

	// make rtgMSS by using xml file
	mRtgMusculoSkeletalSystem = std::make_shared<MusculoSkeletalSystem>();
	mRtgMusculoSkeletalSystem->mSkeleton = BuildFromFile(string(MSS_ROOT_DIR) + "/model/rtgSkeleton.xml", true, Eigen::Vector3d(0.0, 0.9809, -0.0308));

	// make muscles
	MakeMuscles(string(MSS_ROOT_DIR) + "/model/standard/muscle/"+rtgMuscleFileName, mRtgMusculoSkeletalSystem);
	mRtgMusculoSkeletalSystem->initialize(mRigidWorld);

	cout << "Success" << endl;


}


void
IntegratedWorld::
updateTargetSkeleton() {
	// remove original skeleton
	mRigidWorld->removeSkeleton(mRtgMusculoSkeletalSystem->getSkeleton());

	// make rtgSkeleton.xml
	BuildFromSkeleton(string(MSS_ROOT_DIR) + "/model/stdSkeleton.xml", string(MSS_ROOT_DIR) + "/model/userInput.txt", string(MSS_ROOT_DIR) + "/model/rtgSkeleton.xml", mbis);

	// make rtgSkeleton bone meshes
	boneMeshEditing(string(MSS_ROOT_DIR) + "/model/stdSkeleton.xml", string(MSS_ROOT_DIR) + "/model/rtgSkeleton.xml", string(MSS_ROOT_DIR) + "/model/userInput.txt");

	// make rtgMSS by using xml file
	mRtgMusculoSkeletalSystem = std::make_shared<MusculoSkeletalSystem>();
	mRtgMusculoSkeletalSystem->mSkeleton = BuildFromFile(string(MSS_ROOT_DIR) + "/model/rtgSkeleton.xml", true, Eigen::Vector3d(0.0, 0.9809, -0.0308));

	// make muscles
	MakeMuscles(string(MSS_ROOT_DIR) + "/model/standard/muscle/muscle_params.xml", mRtgMusculoSkeletalSystem);
	mRtgMusculoSkeletalSystem->initialize(mRigidWorld);
	mRtgMusculoSkeletalSystem->updateTotalWaypoints();
}