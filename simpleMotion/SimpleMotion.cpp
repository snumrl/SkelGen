//
// Created by hoseok on 11/3/18.
//

#include "SimpleMotion.h"


void configInterpolation(double *startConfig, double *endConfig, double *config, double ratio, int DOF){
	for (int i=0;i<DOF;i++){
		config[i] = startConfig[i] * (1.0-ratio) + endConfig[i] * ratio;
	}
}
