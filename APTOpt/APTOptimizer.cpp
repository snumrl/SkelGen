#include <math.h>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include "APTOptimizer.h"
#include "../model/MusculoSkeletalSystem.h"
#include "../simpleMotion/SimpleMotion.h"
#include "../generator/motionAdjust/muscleCal.h"
#include <stdio.h>
using namespace std;
using namespace dart;
using namespace dart::dynamics;

extern std::string curSimpleMotion;
extern std::string curDof;

int negativeness = -1;

std::chrono::time_point<std::chrono::system_clock> time_check_s = std::chrono::system_clock::now();

void time_check_start()
{
	time_check_s = std::chrono::system_clock::now();
}

void time_check_end()
{
	std::chrono::duration<double> elapsed_seconds;
	elapsed_seconds = std::chrono::system_clock::now()-time_check_s;
	cout<<elapsed_seconds.count()<<endl;
}
APTHelper::APTHelper()
{
}

APTOptimizer::APTOptimizer(std::shared_ptr<MusculoSkeletalSystem> stdMSS, std::shared_ptr<MusculoSkeletalSystem> rtgMSS)
:stdMSS(stdMSS), rtgMSS(rtgMSS)
{
	this->initialize();
}

void APTOptimizer::initialize()
{
	readJointMap();
	readSimpleMotion();
	stdAptHelper = new APTHelper();
	rtgAptHelper = new APTHelper();
}

// void APTOptimizer::precomputeSimpleMotionLmt()
// {
// 	precomputeSimpleMotionLmt(this->stdMSS, stdAptHelper);
// 	precomputeSimpleMotionLmt(this->rtgMSS, rtgAptHelper);
// }

void APTOptimizer::readJointMap() {
	FILE *in=fopen((string(MSS_ROOT_DIR) + "/simpleMotion/JointMap.txt").c_str(), "r");
	char line[1005];
	while (fgets(line, 100, in) != NULL) {
		string str = string(line);
		char *p = strtok(line, " \n");
		string name = p;
		p = strtok(NULL, " \n");
		string bnName = p;
		p = strtok(NULL, " \n");
		int idx = atoi(p);
		dart::dynamics::BodyNode* bn = rtgMSS->getSkeleton()->getBodyNode(bnName);
		int offset = bn->getParentJoint()->getDof(0)->getIndexInSkeleton();
		jointMap[name] = offset + idx;
	}

}

void APTOptimizer::readSimpleMotion() {
	readJointMap();
	FILE *in = fopen((string(MSS_ROOT_DIR) + "/simpleMotion/SimpleMotionSet.txt").c_str(), "r");
	char line[1005];
	vector<SimpleMotion> _simpleMotions;
	while (fgets(line, 100, in) != NULL) {
		string str = string(line);
		if (line[0] == 'n') {
			char *p = strtok(line, " \n");
			p = strtok(NULL, " \n");
			_simpleMotions.emplace_back(SimpleMotion());
			_simpleMotions.back().motionName = p;

			if (p=="T_Pose" || p=="Stand_Pose"){
				for (int i=0;i<stdMSS->getNumMuscles();i++)
					_simpleMotions.back().relatedMuscleIndices.emplace_back(i);
			}
		} else if (line[0] == 'c') {
			char *p = strtok(line, " \n");
			p = strtok(NULL, " \n");
			int idx = jointMap[p];

			double s, e;
			p = strtok(NULL, " \n");
			s = atof(p);
			p = strtok(NULL, " \n");
			e = atof(p);

			_simpleMotions.back().startConfig[idx] = s;
			_simpleMotions.back().endConfig[idx] = e;
		} else if (line[0] == 'm') {
			char *p = strtok(line, " \n");
			p = strtok(NULL, " \n");
			string muscleName = p;

			_simpleMotions.back().relatedMuscleIndices.emplace_back(getMuscleIdx(muscleName));
		} else if (line[0] == 'p') {
			char *p = strtok(line, " \n");
			p = strtok(NULL, " \n");
			string motionName = p;
			for (auto &_sm: _simpleMotions) {
				if (_sm.motionName == motionName) {
					simpleMotions.emplace_back(_sm);
					break;
				}
			}
		}
	}
}
int APTOptimizer::getMuscleIdx(string muscle_name){
	for (int i=0;i< stdMSS->getNumMuscles();i++){
		if (muscle_name == stdMSS->getMuscles()[i]->name){
			return i;
		}
	}
//	assert(false && printf("Cannot find muscle name %s" , muscle_name.c_str()));
	return 0;
}

int APTOptimizer::getSimpleMotionIdx(std::string sm_name) {
	for (int i=0;i<simpleMotions.size();i++){
		if (simpleMotions[i].motionName == sm_name) return i;
	}
	return -1;
}

// Retarget the Angle of Peak Torque
void APTOptimizer::retargetAPT()
{
	// precomputeSimpleMotionLmt();
	computeSimpleMotionAPT(this->stdMSS, this->stdAptHelper);
	computeSimpleMotionAPT(this->rtgMSS, this->rtgAptHelper);
	for(int i=0;i<SIMPLEMOTIONNUM;i++)
	{
		cout<<this->stdAptHelper->muscleMaxAngle[i]<<", "<<this->stdAptHelper->muscleMaxTorque[i]<<endl;
		cout<<this->rtgAptHelper->muscleMaxAngle[i]<<", "<<this->rtgAptHelper->muscleMaxTorque[i]<<endl;
	}

	//we hardcode the max angle value
	this->stdAptHelper->muscleMaxAngle[0] = 30;

	// change rtg MSS's parameters
	precomputeForceAndJacobian(this->stdMSS, this->stdAptHelper);
	precomputeForceAndJacobian(this->rtgMSS, this->rtgAptHelper);
	optimizeStandardAPT(30);
	optimizeAPT_Numerical();


}


// void APTOptimizer::precomputeSimpleMotionLmt(std::shared_ptr<MusculoSkeletalSystem> MSS, APTHelper* aptHelper)
// {
// 	double config[DOF];
// 	SkeletonPtr skel = MSS->getSkeleton();
// 	Eigen::VectorXd originPose = skel->getPositions();
// 	for(int simplei=0;simplei<SIMPLEMOTIONNUM;simplei++)
// 	{
// 		for(int ratioj=0;ratioj<100;ratioj++)
// 		{
// 			configInterpolation(simpleMotions[simplei].startConfig, simpleMotions[simplei].endConfig, config, ratioj/100.0, DOF);
// 			Eigen::VectorXd pos(DOF);
// 			for(int k=0;k<DOF;k++)
// 			{
// 				pos[k] = config[k] * M_PI / 180.0;
// 			}
// 			skel->setPositions(pos);

// 			for(int musclek=0;musclek<MSS->getNumMuscles();musclek++)
// 			{
// 				aptHelper->muscleLmt[musclek][simplei][ratioj] = MSS->getMuscles()[musclek]->getLength();
// 				// cout<<MSS->getMuscles()[musclek]->getLength()<<endl;
// 			}
// 		}
// 	}
// 	skel->setPositions(originPose);
// }

// All of the APTOptimizer functions are called after updateMuscleParametersByMaxLmt
void APTOptimizer::precomputeForceAndJacobian(std::shared_ptr<MusculoSkeletalSystem> MSS, APTHelper* aptHelper)
{
	SkeletonPtr skel = MSS->getSkeleton();
	double config[DOF];
	int simpleMotionIndex = getSimpleMotionIdx(curSimpleMotion);
	Eigen::VectorXd originPose = skel->getPositions();
	for(int i=0;i<SIMPLEMOTIONNUM;i++)
	{
		int maxTorqueidx = 0;
		double maxTorque = 0;
		for(int j=0;j<NUMSTEPS;j++)
		{
			int jacIndex =0;
			configInterpolation(simpleMotions[simpleMotionIndex].startConfig, simpleMotions[simpleMotionIndex].endConfig, config, j/((double) NUMSTEPS), DOF);
			Eigen::VectorXd pos(DOF);
			for(int k=0;k<DOF;k++)
			{
				pos[k] = config[k] * M_PI / 180.0;
			}
			skel->setPositions(pos);
			aptHelper->precomputedActiveForce.push_back(MSS->computeForceActive());
			aptHelper->precomputedPassiveForce.push_back(MSS->computeForcePassive());
        	aptHelper->precomputedJacobianTranspose.push_back(MSS->getJacobianTranspose());
        }
    }
    skel->setPositions(originPose);
}


void APTOptimizer::computeSimpleMotionAPT(std::shared_ptr<MusculoSkeletalSystem> MSS, APTHelper* aptHelper)
{
	Eigen::VectorXd activationLevels(MSS->getNumMuscles());
	activationLevels.setOnes();

	//set to max activation.
	MSS->setActivationLevels(activationLevels);
	int targetJoint = MSS->getSkeleton()->getIndexOf(MSS->getSkeleton()->getBodyNode(curDof)->getParentJoint()->getDof(0));
	double config[DOF];
	int simpleMotionIndex = getSimpleMotionIdx(curSimpleMotion);
	SkeletonPtr skel = MSS->getSkeleton();
	Eigen::VectorXd originPose = skel->getPositions();

	for(int i=0;i<SIMPLEMOTIONNUM;i++)
	{
		int maxTorqueidx = 0;
		double maxTorque = 0;
		for(int j=0;j<NUMSTEPS;j++)
		{
			int jacIndex =0;
			configInterpolation(simpleMotions[simpleMotionIndex].startConfig, simpleMotions[simpleMotionIndex].endConfig, config, j/((double) NUMSTEPS), DOF);
			Eigen::VectorXd pos(DOF);
			for(int k=0;k<DOF;k++)
			{
				pos[k] = config[k] * M_PI / 180.0;
			}
			skel->setPositions(pos);
			Eigen::VectorXd muscleForceActive = MSS->computeForceActive();
			Eigen::VectorXd muscleForcePassive = MSS->computeForcePassive();
        	Eigen::MatrixXd muscleJacobianTranspose =  MSS->getJacobianTranspose();
			double curTorque = 0;

			for(int k=0; k<MSS->getNumMuscles();k++)
			{
				int muscleNumForces = MSS->getMuscles()[k]->getNumForces();
	       		if((muscleJacobianTranspose.block(targetJoint, jacIndex*3, 
	        		1, muscleNumForces*3) * muscleForceActive.segment(jacIndex*3,muscleNumForces*3))[0]<-1e-2)
	       		{
		   			curTorque += (muscleJacobianTranspose.block(targetJoint, jacIndex*3, 
		        		1, muscleNumForces*3) * muscleForceActive.segment(jacIndex*3,muscleNumForces*3))[0];
	   			}
	       		jacIndex += muscleNumForces;
	       	}
   			curTorque += (muscleJacobianTranspose * muscleForcePassive)[targetJoint];
   			curTorque *= negativeness;
       		if(curTorque>maxTorque)
       		{
       			maxTorque = curTorque;
       			maxTorqueidx = j;
       		}
		}
		aptHelper->muscleMaxAngle[i] = maxTorqueidx;
		aptHelper->muscleMaxTorque[i] = maxTorque;
		skel->setPositions(originPose);
	}
}

void APTOptimizer::computeSimpleMotionAPT(std::shared_ptr<MusculoSkeletalSystem> MSS, APTHelper* aptHelper, int changedIndex)
{
	Eigen::VectorXd activationLevels(MSS->getNumMuscles());
	activationLevels.setOnes();
	//set to max activation.
	MSS->setActivationLevels(activationLevels);
	int targetJoint = MSS->getSkeleton()->getIndexOf(MSS->getSkeleton()->getBodyNode(curDof)->getParentJoint()->getDof(0));
	double config[DOF];
	int simpleMotionIndex = getSimpleMotionIdx(curSimpleMotion);
	SkeletonPtr skel = MSS->getSkeleton();
	Eigen::VectorXd originPose = skel->getPositions();
	for(int i=0;i<SIMPLEMOTIONNUM;i++)
	{
		int maxTorqueidx = 0;
		double maxTorque = 0;
		for(int j=0;j<NUMSTEPS;j++)
		{
			int jacIndex =0;
			configInterpolation(simpleMotions[simpleMotionIndex].startConfig, simpleMotions[simpleMotionIndex].endConfig, config, j/((double) NUMSTEPS), DOF);
			Eigen::VectorXd pos(DOF);
			for(int k=0;k<DOF;k++)
			{
				pos[k] = config[k] * M_PI / 180.0;
			}
			skel->setPositions(pos);
			Eigen::VectorXd muscleForceActive = MSS->updateForceActive(aptHelper->precomputedActiveForce[j], changedIndex);
			Eigen::VectorXd muscleForcePassive = MSS->updateForcePassive(aptHelper->precomputedPassiveForce[j], changedIndex);
        	Eigen::MatrixXd muscleJacobianTranspose =  aptHelper->precomputedJacobianTranspose[j];


        	// cout<<"dot test : "<<(muscleJacobianTranspose * muscleForcePassive).transpose()<<endl;
        	// cout<<"dot test : "<<(muscleForcePassive).normalized().dot(
        	// (MSS->computeForcePassive()).normalized())<<endl;//.transpose()<<endl;
        	// cout<<endl;
			double curTorque = 0;

			for(int k=0; k<MSS->getNumMuscles();k++)
			{
				int muscleNumForces = MSS->getMuscles()[k]->getNumForces();
	       		if((muscleJacobianTranspose.block(targetJoint, jacIndex*3, 
	        		1, muscleNumForces*3) * muscleForceActive.segment(jacIndex*3,muscleNumForces*3))[0]<-1e-2)
	       		{
		   			curTorque += (muscleJacobianTranspose.block(targetJoint, jacIndex*3, 
		        		1, muscleNumForces*3) * muscleForceActive.segment(jacIndex*3,muscleNumForces*3))[0];
	   			}
	       		jacIndex += muscleNumForces;
	       	}
   			curTorque += (muscleJacobianTranspose * muscleForcePassive)[targetJoint];

   			curTorque *= negativeness;
       		if(curTorque>maxTorque)
       		{
       			maxTorque = curTorque;
       			maxTorqueidx = j;
       		}
		}
		// cout<<maxTorqueidx<<endl;
		aptHelper->muscleMaxAngle[i] = maxTorqueidx;
		aptHelper->muscleMaxTorque[i] = maxTorque;
		// cout<<aptHelper->muscleMaxAngle[i]<<endl;
		skel->setPositions(originPose);
	}
}

void APTOptimizer::applyVariableToMSS(Eigen::VectorXd muscleLmoNormalized, std::shared_ptr<MusculoSkeletalSystem> MSS)
{
	for(int i=0;i<MSS->getNumMuscles();i++)
	{
		auto& muscle = MSS->getMuscles()[i];
		muscle->l_m_o = muscleLmoNormalized[i];
		muscle->l_t_sl = 1 - muscleLmoNormalized[i];
	}
	// MSS->updateMuscleParametersByMaxLmt();
	MSS->updateMuscleParametersByMaxLmtFast();
}


double APTOptimizer::getObjectiveValue(Eigen::VectorXd muscleLmoNormalized)
{
	this->applyVariableToMSS(muscleLmoNormalized, this->rtgMSS);
	this->computeSimpleMotionAPT(this->rtgMSS, this->rtgAptHelper);
	return (this->stdAptHelper->muscleMaxAngle[0] - this->rtgAptHelper->muscleMaxAngle[0])*(this->stdAptHelper->muscleMaxAngle[0] - this->rtgAptHelper->muscleMaxAngle[0]);
}

double APTOptimizer::getObjectiveValue(Eigen::VectorXd muscleLmoNormalized, int changedIndex)
{
	this->applyVariableToMSS(muscleLmoNormalized, this->rtgMSS);
	this->computeSimpleMotionAPT(this->rtgMSS, this->rtgAptHelper, changedIndex);
	return (this->stdAptHelper->muscleMaxAngle[0] - this->rtgAptHelper->muscleMaxAngle[0])*(this->stdAptHelper->muscleMaxAngle[0] - this->rtgAptHelper->muscleMaxAngle[0]);
}
double APTOptimizer::getObjectiveValueStd(Eigen::VectorXd muscleLmoNormalized, double desiredAngle)
{
	this->applyVariableToMSS(muscleLmoNormalized, this->stdMSS);
	this->computeSimpleMotionAPT(this->stdMSS, this->stdAptHelper);
	return (this->stdAptHelper->muscleMaxAngle[0] - desiredAngle)*(this->stdAptHelper->muscleMaxAngle[0] - desiredAngle);
}

double APTOptimizer::getObjectiveValueStd(Eigen::VectorXd muscleLmoNormalized, int changedIndex, double desiredAngle)
{
	this->applyVariableToMSS(muscleLmoNormalized, this->stdMSS);
	this->computeSimpleMotionAPT(this->stdMSS, this->stdAptHelper, changedIndex);
	return (this->stdAptHelper->muscleMaxAngle[0] - desiredAngle)*(this->stdAptHelper->muscleMaxAngle[0] - desiredAngle);
}

Eigen::VectorXd APTOptimizer::getObjectiveValueGradient(Eigen::VectorXd muscleLmoNormalized)
{
	double stepSize = 0.1;
	Eigen::VectorXd grad_vec(MUSCLENUM);
	grad_vec.setZero();
	Eigen::VectorXd n_x, p_x;
	double n_val, p_val;
	n_x = muscleLmoNormalized;
	p_x = muscleLmoNormalized;


	for(int i=0;i<MUSCLENUM;i++)
	{
		// cout<<"muscle : "<<i<<endl;
	// time_check_start();
		n_x = muscleLmoNormalized;
		p_x = muscleLmoNormalized;

		n_x[i] += -stepSize;
		p_x[i] += stepSize;
		if(n_x[i]<0 || p_x[i] >=1)
		{
			grad_vec[i] = 0.0;
			this->applyVariableToMSS(muscleLmoNormalized, this->rtgMSS);
			continue;
		}
		// cout<<n_x.transpose()<<endl;
		n_val = getObjectiveValue(n_x, i);
		// if(rtgMSS->getMuscles()[i]->name == "R_Bicep_Femoris_Longus"||
		// 	rtgMSS->getMuscles()[i]->name == "R_Semimembranosus")
		// {
		// 	// cout<<"negative : "<<this->rtgAptHelper->muscleMaxAngle[0]<<endl;
		// }

		p_val = getObjectiveValue(p_x, i);
		// if(rtgMSS->getMuscles()[i]->name == "R_Bicep_Femoris_Longus"||
		// 	rtgMSS->getMuscles()[i]->name == "R_Semimembranosus")
		// {
		// 	// cout<<"positive : "<<this->rtgAptHelper->muscleMaxAngle[0]<<endl;
		// }
		grad_vec[i] = (p_val-n_val)/(2 * stepSize);
		// n_x = muscleLmoNormalized;
		// p_x = muscleLmoNormalized;
		this->applyVariableToMSS(muscleLmoNormalized, this->rtgMSS);
		if(grad_vec[i] != 0)
		{
			cout<<this->rtgMSS->getMuscles()[i]->name<<" "<<muscleLmoNormalized[i]<<" "<<grad_vec[i]<<endl;//<<" "<<muscleLmoNormalized[i]<<endl;
		}
	// time_check_end();
	}
	this->applyVariableToMSS(muscleLmoNormalized, this->rtgMSS);
	return grad_vec;
}


Eigen::VectorXd APTOptimizer::getObjectiveValueGradientStd(Eigen::VectorXd muscleLmoNormalized, double desiredAngle)
{
	double stepSize = 0.1;
	Eigen::VectorXd grad_vec(MUSCLENUM);
	grad_vec.setZero();
	Eigen::VectorXd n_x, p_x;
	double n_val, p_val;
	n_x = muscleLmoNormalized;
	p_x = muscleLmoNormalized;


	for(int i=0;i<MUSCLENUM;i++)
	{
		// cout<<"muscle : "<<i<<endl;
	// time_check_start();
		n_x = muscleLmoNormalized;
		p_x = muscleLmoNormalized;

		n_x[i] += -stepSize;
		p_x[i] += stepSize;
		if(n_x[i]<0 || p_x[i] >=1)
		{
			grad_vec[i] = 0.0;
			this->applyVariableToMSS(muscleLmoNormalized, this->stdMSS);
			continue;
		}
		// cout<<n_x.transpose()<<endl;
		n_val = getObjectiveValueStd(n_x, i, desiredAngle);
		if(stdMSS->getMuscles()[i]->name == "R_Bicep_Femoris_Longus"||
			stdMSS->getMuscles()[i]->name == "R_Semimembranosus")
		{
			// cout<<"negative : "<<this->rtgAptHelper->muscleMaxAngle[0]<<endl;
		}

		p_val = getObjectiveValueStd(p_x, i, desiredAngle);
		if(stdMSS->getMuscles()[i]->name == "R_Bicep_Femoris_Longus"||
			stdMSS->getMuscles()[i]->name == "R_Semimembranosus")
		{
			// cout<<"positive : "<<this->rtgAptHelper->muscleMaxAngle[0]<<endl;
		}
		grad_vec[i] = (p_val-n_val)/(2 * stepSize);
		// n_x = muscleLmoNormalized;
		// p_x = muscleLmoNormalized;
		this->applyVariableToMSS(muscleLmoNormalized, this->stdMSS);
		if(grad_vec[i] != 0)
		{
			cout<<this->stdMSS->getMuscles()[i]->name<<" "<<muscleLmoNormalized[i]<<" "<<grad_vec[i]<<endl;//<<" "<<muscleLmoNormalized[i]<<endl;
		}
	// time_check_end();
	}
	this->applyVariableToMSS(muscleLmoNormalized, this->stdMSS);
	return grad_vec;
}
void APTOptimizer::optimizeStandardAPT(double desiredAngle)
{
	cout<<"STD optimizing start"<<endl;
	int maxNumSteps = 5;
	double stepSize = 0.00002;

	Eigen::VectorXd startValue(MUSCLENUM);
	for(int i=0;i<MUSCLENUM;i++)
	{
		startValue[i] = stdMSS->getMuscles()[i]->l_m_o/(stdMSS->getMuscles()[i]->l_m_o+stdMSS->getMuscles()[i]->l_t_sl);
	}
	Eigen::VectorXd curValue = startValue;
	Eigen::VectorXd curGradient;
	cout<<"Initial Energy Value : "<<this->getObjectiveValueStd(startValue, desiredAngle)<<endl;

	int gastro_lh_index = 0;
	for(int i=0;i<rtgMSS->getNumMuscles();i++)
	{
		if(rtgMSS->getMuscles()[i]->name == "R_Semimembranosus")
		{
			gastro_lh_index = i;
		}
	}

	// for(int i=0;i<200;i++)
	// {
	// 	curValue[gastro_lh_index] = i/200.0;
	// 	cout<<"Current Energy Value : "<<this->getObjectiveValueStd(curValue, desiredAngle)<<endl;
	// 	cout<<i/200.0<<" : "<<this->stdAptHelper->muscleMaxAngle[0]<<endl;
	// 	cout<<endl;
	// }

	for(int i=0;i<maxNumSteps;i++)
	{
		cout<<"step ("<<i<<"/"<<maxNumSteps<<")"<<endl;
		curGradient = getObjectiveValueGradientStd(curValue, desiredAngle);
		curValue +=  -curGradient * stepSize;
		// for(int j=0;j<curValue.size();j++)
		// {
		// 	curValue
		// }
		for(int j=0; j<curValue.size();j++)
		{
			if(curValue[j]<0)
				curValue[j] = 0.0;
			else if(curValue[j]>1.0)
				curValue[j] = 1.0;
		}
		cout<<curGradient.transpose()<<endl;
		cout<<"Current Energy Value : "<<this->getObjectiveValueStd(curValue, desiredAngle)<<endl;
		cout<<"Optimial : "<<desiredAngle<<" Curent : "<<this->stdAptHelper->muscleMaxAngle[0]<<endl;
		cout<<endl;
	}
	cout<<"STD optimizing complete"<<endl;

	// Eigen::VectorXd stdValue(this->stdMSS->getNumMuscles());
	// Eigen::VectorXd rtgValue(this->rtgMSS->getNumMuscles());
	// for(int i=0;i<MUSCLENUM;i++)
	// {
	// 	stdValue[i] = stdMSS->getMuscles()[i]->l_m_o/(stdMSS->getMuscles()[i]->l_m_o+stdMSS->getMuscles()[i]->l_t_sl);
	// 	rtgValue[i] = rtgMSS->getMuscles()[i]->l_m_o/(rtgMSS->getMuscles()[i]->l_m_o+rtgMSS->getMuscles()[i]->l_t_sl);
	// 	cout<<stdMSS->getMuscles()[i]->name<<" "<<stdValue[i]<<" "<<rtgValue[i]<<endl;
	// }
}

void APTOptimizer::optimizeAPT_Numerical()
{
	int maxNumSteps = 5;
	double stepSize = 0.00002;

	Eigen::VectorXd startValue(MUSCLENUM);
	for(int i=0;i<MUSCLENUM;i++)
	{
		startValue[i] = rtgMSS->getMuscles()[i]->l_m_o/(rtgMSS->getMuscles()[i]->l_m_o+rtgMSS->getMuscles()[i]->l_t_sl);
	}
	Eigen::VectorXd curValue = startValue;
	Eigen::VectorXd curGradient;
	cout<<"Initial Energy Value : "<<this->getObjectiveValue(startValue)<<endl;

	int gastro_lh_index = 0;
	for(int i=0;i<rtgMSS->getNumMuscles();i++)
	{
		if(rtgMSS->getMuscles()[i]->name == "R_Gastrocnemius_Lateral_Head")
		{
			gastro_lh_index = i;
		}
	}

	// for(int i=0;i<100;i++)
	// {
	// 	curValue[gastro_lh_index] = i/100.0;
	// 	cout<<"Current Energy Value : "<<this->getObjectiveValue(curValue)<<endl;
	// 	cout<<i/100.0<<" : "<<this->rtgAptHelper->muscleMaxAngle[0]<<endl;
	// 	cout<<endl;
	// }

	for(int i=0;i<maxNumSteps;i++)
	{
		cout<<"step ("<<i<<"/"<<maxNumSteps<<")"<<endl;
		curGradient = getObjectiveValueGradient(curValue);
		curValue +=  -curGradient * stepSize;
		// for(int j=0;j<curValue.size();j++)
		// {
		// 	curValue
		// }
		for(int j=0; j<curValue.size();j++)
		{
			if(curValue[j]<0)
				curValue[j] = 0.0;
			else if(curValue[j]>1.0)
				curValue[j] = 1.0;
		}
		cout<<curGradient.transpose()<<endl;
		cout<<"Current Energy Value : "<<this->getObjectiveValue(curValue)<<endl;
		cout<<"Optimial : "<<this->stdAptHelper->muscleMaxAngle[0]<<" Curent : "<<this->rtgAptHelper->muscleMaxAngle[0]<<endl;
		cout<<endl;
	}

	Eigen::VectorXd stdValue(this->stdMSS->getNumMuscles());
	Eigen::VectorXd rtgValue(this->rtgMSS->getNumMuscles());
	for(int i=0;i<MUSCLENUM;i++)
	{
		stdValue[i] = stdMSS->getMuscles()[i]->l_m_o/(stdMSS->getMuscles()[i]->l_m_o+stdMSS->getMuscles()[i]->l_t_sl);
		rtgValue[i] = rtgMSS->getMuscles()[i]->l_m_o/(rtgMSS->getMuscles()[i]->l_m_o+rtgMSS->getMuscles()[i]->l_t_sl);
		cout<<stdMSS->getMuscles()[i]->name<<" "<<stdValue[i]<<" "<<rtgValue[i]<<endl;
	}

}
