//
// Created by hoseok on 11/3/18.
//

#ifndef EXAM_SIMPLEMOTION_H
#define EXAM_SIMPLEMOTION_H

#include <string>
#include <vector>

const int NUM_MUSCLE = 100;
const int DOF = 50;
class SimpleMotion {
public:
	SimpleMotion(){
		for (int i=0;i<DOF;i++) startConfig[i] = endConfig[i] = 0;
//		startConfig[33] = endConfig[33] = 90;
//		startConfig[41] = endConfig[41] = -90;
		for (int i=0;i<NUM_MUSCLE;i++){
			for (int j=0;j<DOF;j++){
				monotonicityFlag[i][j] = 0;
			}
		}
	}
	double startConfig[DOF], endConfig[DOF];
	int monotonicityFlag[NUM_MUSCLE][DOF]; // 1: increasing, 0: Don't Care(DC), -1: decreasing

	std::string motionName; // ex) "knee flexion", "hip flexion"
	std::vector<int> relatedMuscleIndices;

	void execute();
};


void configInterpolation(double *startConfig, double *endConfig, double *config, double ratio, int DOF);


#endif //EXAM_SIMPLEMOTION_H
