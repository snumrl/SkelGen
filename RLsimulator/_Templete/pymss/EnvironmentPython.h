#ifndef __MSS_ENVIRONMENT_PYTHON_H__
#define __MSS_ENVIRONMENT_PYTHON_H__
#include "Environment.h"
#include <vector>
#include <string>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include "WrapperFunctions.h"

class EnvironmentPython
{
public:
	EnvironmentPython(int simulationHz);
	//For general properties
	int GetNumState();
	int GetNumAction();
	int GetNumDofs();
	int GetSimulationHz(){return mSlaves[0]->GetSimulationHz();};
	int GetControlHz(){return mSlaves[0]->GetControlHz();};

	//For each slave
	void Step(int id);
	void Reset(bool RSI,int id);
	void SetGoal(np::ndarray np_array,int id);
	bool IsTerminalState(int id);
	np::ndarray GetState(int id);
	void SetAction(np::ndarray np_array,int id);
	double GetReward(int id);

	//For all slaves
	void StepsAtOnce();
	void Steps(int num);
	void Resets(bool RSI);
	np::ndarray IsTerminalStates();
	np::ndarray GetStates();
	void SetActions(np::ndarray np_array);
	np::ndarray GetRewards();

	//For Muscle Transitions
	int GetNumTotalMuscleRelatedDofs(){return mSlaves[0]->GetNumTotalRelatedDofs();};
	int GetNumMuscles(){return mSlaves[0]->GetCharacter()->GetMuscles().size();}
	np::ndarray GetMuscleTorques();
	np::ndarray GetDesiredTorques();
	void SetActivationLevels(np::ndarray np_array);
	
	p::list GetMuscleTuples();
private:
	std::vector<MSS::Environment*> mSlaves;
	int mNumSlaves;
	int mNumState;
	int mNumAction;
};
#endif