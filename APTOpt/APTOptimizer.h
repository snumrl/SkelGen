//
// Created by minseok on 12/28/18.
//
#ifndef APTOPT_H
#define APTOPT_H
#include "../model/MusculoSkeletalSystem.h"
#include "../simpleMotion/SimpleMotion.h"

const int MUSCLENUM = 284;
const int SIMPLEMOTIONNUM = 1;
const int NUMSTEPS = 200;

class APTHelper
{
public:
	APTHelper();
	// double muscleLmt[MUSCLENUM][SIMPLEMOTIONNUM][100];	//muscle index, simplemotion index, progress ratio index
	int muscleMaxAngle[SIMPLEMOTIONNUM];	//simplemotion index. return max angle progress ratio index
	double muscleMaxTorque[SIMPLEMOTIONNUM];	//simplemotion index. return max angle progress ratio index

	std::vector<Eigen::VectorXd> precomputedActiveForce;
	std::vector<Eigen::VectorXd> precomputedPassiveForce;
	std::vector<Eigen::MatrixXd> precomputedJacobianTranspose;
};

class APTOptimizer
{
public:
	APTOptimizer(std::shared_ptr<MusculoSkeletalSystem> stdMSS, std::shared_ptr<MusculoSkeletalSystem> rtgMSS);
	void initialize();

	// Lmt will not be changed by muscle parameter tunning
	void precomputeSimpleMotionLmt();
	void precomputeSimpleMotionLmt(std::shared_ptr<MusculoSkeletalSystem> MSS, APTHelper* aptHelper);

	void precomputeForceAndJacobian(std::shared_ptr<MusculoSkeletalSystem> MSS, APTHelper* aptHelper);

	void retargetAPT();

	void optimizeAPT();
	void optimizeAPT_Numerical();

	void optimizeStandardAPT(double desiredAngle);

	void applyVariableToMSS(Eigen::VectorXd muscleLmoNormalized, std::shared_ptr<MusculoSkeletalSystem> MSS);

	void computeSimpleMotionAPT(std::shared_ptr<MusculoSkeletalSystem> MSS, APTHelper* aptHelper);
	void computeSimpleMotionAPT(std::shared_ptr<MusculoSkeletalSystem> MSS, APTHelper* aptHelper, int changedIndex);

	double getObjectiveValue(Eigen::VectorXd x);
	double getObjectiveValue(Eigen::VectorXd x, int changedIndex);

	double getObjectiveValueStd(Eigen::VectorXd x, double desiredAngle);
	double getObjectiveValueStd(Eigen::VectorXd x, int changedIndex, double desiredAngle);

	Eigen::VectorXd getObjectiveValueGradient(Eigen::VectorXd x);
	Eigen::VectorXd getObjectiveValueGradientStd(Eigen::VectorXd x, double desiredAngle);

	int getMuscleIdx(std::string muscleName);

	std::shared_ptr<MusculoSkeletalSystem> stdMSS;
	std::shared_ptr<MusculoSkeletalSystem> rtgMSS;

	APTHelper* stdAptHelper;
	APTHelper* rtgAptHelper;

	// simplemotions
    void readJointMap();
	void readSimpleMotion();
    int getSimpleMotionIdx(std::string sm_name);
    std::map<std::string, int> jointMap;
    std::vector<SimpleMotion> simpleMotions;

};
#endif