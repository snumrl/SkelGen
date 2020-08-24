//
// Created by hoseok on 10/29/18.
//

#include "SkeletonInfo.h"


void SkeletonInfo::setHeight(double height) {


	SkeletonInfo::height = height;


	R_Femur.alpha_lengthening = height / 175.0 ;         // (cm), 0.4453391 cm in zygote
	R_Tibia.alpha_lengthening = height / 175.0;         // (cm), 0.4249466 cm in zygote
//	R.setHumerusLength(33.24388 / 175.0 * height);      // (cm), 0.3324388 cm in zygote
//	R.setRadiusLength(26.98825 / 175.0 * height);       // (cm), 0.2698825 cm in zygote

	L_Femur.alpha_lengthening = height / 175.0;         // (cm), 0.4453391 cm in zygote
	L_Tibia.alpha_lengthening = height / 175.0;         // (cm), 0.4249466 cm in zygote
//	L.setHumerusLength(33.24388 / 175.0 * height);      // (cm), 0.3324388 cm in zygote
//	L.setRadiusLength(26.98825 / 175.0 * height);       // (cm), 0.2698825 cm in zygote
}

void SkeletonInfo::setWeight(double weight) {
	SkeletonInfo::weight = weight;
}

void SkeletonInfo::clear() {
	setHeight(175.0);
}
