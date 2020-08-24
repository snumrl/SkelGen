#include "EnvironmentPython.h"
#include <omp.h>
#include "dart/math/math.hpp"
#include <iostream>

EnvironmentPython::
EnvironmentPython(int simulationHz)
	:mNumSlaves(16)
{
	dart::math::seedRand();
	omp_set_num_threads(mNumSlaves);
	for(int i =0;i<mNumSlaves;i++){
		mSlaves.push_back(new MSS::Environment(30,simulationHz));
	}
	mNumState = mSlaves[0]->GetNumState();
	mNumAction = mSlaves[0]->GetNumAction();
}
//For general properties
int
EnvironmentPython::
GetNumState()
{
	return mNumState;
}
int
EnvironmentPython::
GetNumAction()
{
	return mNumAction;
}
int
EnvironmentPython::
GetNumDofs()
{
	return mSlaves[0]->GetCharacter()->GetSkeleton()->getNumDofs();
}
//For each slave
void 
EnvironmentPython::
Step(int id)
{
	mSlaves[id]->Step();
}
void 
EnvironmentPython::
Reset(bool RSI,int id)
{
	mSlaves[id]->Reset(RSI);
}
bool 
EnvironmentPython::
IsTerminalState(int id)
{
	return mSlaves[id]->IsTerminalState(); 
	
}
np::ndarray
EnvironmentPython::
GetState(int id)
{
	return toNumPyArray(mSlaves[id]->GetState());
}
void 
EnvironmentPython::
SetAction(np::ndarray np_array,int id)
{
	mSlaves[id]->SetAction(toEigenVector(np_array));
}
void
EnvironmentPython::
SetGoal(np::ndarray np_array,int id)
{
	mSlaves[id]->SetGoal(toEigenVector(np_array));
}
double 
EnvironmentPython::
GetReward(int id)
{
	return mSlaves[id]->GetReward();
}

//For all slaves
void
EnvironmentPython::
StepsAtOnce()
{
	int num = GetSimulationHz()/GetControlHz();
#pragma omp parallel for
	for (int id = 0; id < mNumSlaves; ++id)
	{
		for(int j =0;j<num;j++)
			this->Step(id);
	}
}

void
EnvironmentPython::
Steps(int num)
{
#pragma omp parallel for
	for (int id = 0; id < mNumSlaves; ++id)
	{
		for(int i=0;i<num;i++)
			this->Step(id);
	}
}
void
EnvironmentPython::
Resets(bool RSI)
{
	for (int id = 0; id < mNumSlaves; ++id)
	{
		this->Reset(RSI,id);
	}
}
np::ndarray
EnvironmentPython::
IsTerminalStates()
{
	std::vector<bool> is_terminate_vector(mNumSlaves);

	for (int id = 0; id < mNumSlaves; ++id)
		is_terminate_vector[id] = IsTerminalState(id);

	return toNumPyArray(is_terminate_vector);
}
np::ndarray
EnvironmentPython::
GetStates()
{
	Eigen::MatrixXd states(mNumSlaves,mNumState);

	for (int id = 0; id < mNumSlaves; ++id)
		states.row(id) = mSlaves[id]->GetState().transpose();

	return toNumPyArray(states);
}
void
EnvironmentPython::
SetActions(np::ndarray np_array)
{
	Eigen::MatrixXd action = toEigenMatrix(np_array);

	for (int id = 0; id < mNumSlaves; ++id){
		mSlaves[id]->SetAction(action.row(id).transpose());
	}
}
np::ndarray
EnvironmentPython::
GetRewards()
{
	std::vector<float> rewards(mNumSlaves);
	for (int id = 0; id < mNumSlaves; ++id)
		rewards[id] = this->GetReward(id);

	return toNumPyArray(rewards);
}

np::ndarray
EnvironmentPython::
GetMuscleTorques()
{
	std::vector<Eigen::VectorXd> mt(mNumSlaves);

#pragma omp parallel for
	for (int id = 0; id < mNumSlaves; ++id)
	{
		mt[id] = mSlaves[id]->GetMuscleTorques();
	}
	return toNumPyArray(mt);
}
np::ndarray
EnvironmentPython::
GetDesiredTorques()
{
	std::vector<Eigen::VectorXd> tau_des(mNumSlaves);
	
#pragma omp parallel for
	for (int id = 0; id < mNumSlaves; ++id)
	{
		tau_des[id] = mSlaves[id]->GetDesiredTorques();
	}
	return toNumPyArray(tau_des);
}

void
EnvironmentPython::
SetActivationLevels(np::ndarray np_array)
{
	std::vector<Eigen::VectorXd> activations =toEigenVectorVector(np_array);
	for (int id = 0; id < mNumSlaves; ++id)
		mSlaves[id]->SetActivationLevels(activations[id]);
}

p::list
EnvironmentPython::
GetMuscleTuples()
{
	p::list all;
	for (int id = 0; id < mNumSlaves; ++id)
	{
		auto& tps = mSlaves[id]->GetMuscleTuples();
		for(int j=0;j<tps.size();j++)
		{
			p::list t;
			t.append(toNumPyArray(tps[j].JtA));
			t.append(toNumPyArray(tps[j].tau_des));
			t.append(toNumPyArray(tps[j].L));
			t.append(toNumPyArray(tps[j].b));
			all.append(t);
		}
		tps.clear();
	}

	return all;
}
using namespace boost::python;

BOOST_PYTHON_MODULE(pymss)
{
	Py_Initialize();
	np::initialize();

	class_<EnvironmentPython>("Env",init<int>())
		.def("GetNumState",&EnvironmentPython::GetNumState)
		.def("GetNumAction",&EnvironmentPython::GetNumAction)
		.def("GetNumDofs",&EnvironmentPython::GetNumDofs)
		.def("GetSimulationHz",&EnvironmentPython::GetSimulationHz)
		.def("GetControlHz",&EnvironmentPython::GetControlHz)
		.def("Reset",&EnvironmentPython::Reset)
		.def("IsTerminalState",&EnvironmentPython::IsTerminalState)
		.def("GetState",&EnvironmentPython::GetState)
		.def("SetAction",&EnvironmentPython::SetAction)
		.def("SetGoal",&EnvironmentPython::SetGoal)
		.def("GetReward",&EnvironmentPython::GetReward)
		.def("StepsAtOnce",&EnvironmentPython::StepsAtOnce)
		.def("Steps",&EnvironmentPython::Steps)
		.def("Resets",&EnvironmentPython::Resets)
		.def("IsTerminalStates",&EnvironmentPython::IsTerminalStates)
		.def("GetStates",&EnvironmentPython::GetStates)
		.def("SetActions",&EnvironmentPython::SetActions)
		.def("GetRewards",&EnvironmentPython::GetRewards)
		.def("GetNumTotalMuscleRelatedDofs",&EnvironmentPython::GetNumTotalMuscleRelatedDofs)
		.def("GetNumMuscles",&EnvironmentPython::GetNumMuscles)
		.def("GetMuscleTorques",&EnvironmentPython::GetMuscleTorques)
		.def("GetDesiredTorques",&EnvironmentPython::GetDesiredTorques)
		.def("SetActivationLevels",&EnvironmentPython::SetActivationLevels)
		.def("GetMuscleTuples",&EnvironmentPython::GetMuscleTuples);
}


