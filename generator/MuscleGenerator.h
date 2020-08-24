//
// Created by hoseok on 11/8/18.
//

#ifndef SKELGEN_MUSCLEGENERATOR_H
#define SKELGEN_MUSCLEGENERATOR_H

#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"

#include "SkeletonInfo.h"
#include "../model/DART_helper.h"
#include "../model/MusculoSkeletalSystem.h"
#include "../functions/functions.h"
#include "../simpleMotion/SimpleMotion.h"

void MakeMuscles(const std::string& path,std::shared_ptr<MusculoSkeletalSystem>& ms);

double calculateSamplingMetric(std::shared_ptr<MusculoSkeletalSystem> &stdMSS,
                               std::shared_ptr<MusculoSkeletalSystem> &rtgMSS,
                               std::vector<SimpleMotion> &simpleMotions);

double calculateSamplingMetricDerivative(std::shared_ptr<MusculoSkeletalSystem> &stdMSS,
                                         std::shared_ptr<MusculoSkeletalSystem> &rtgMSS,
                                         std::vector<SimpleMotion> &simpleMotions, int waypointIdx, int direction);

double calculateMetric(std::shared_ptr<MusculoSkeletalSystem> &stdMSS,
                       std::shared_ptr<MusculoSkeletalSystem> &rtgMSS,
                       std::vector<SimpleMotion> &simpleMotions);

double calculateMetricDerivative(std::shared_ptr<MusculoSkeletalSystem> &stdMSS,
                                 std::shared_ptr<MusculoSkeletalSystem> &rtgMSS,
                                 std::vector<SimpleMotion> &simpleMotions, int waypointIdx, int direction);

/// 이미 load 된 muscle을 그대로 사용!
void retargetMusclesWaypointsLoading(std::shared_ptr<MusculoSkeletalSystem>& rtgMSS);

/// stdMSS에 맞도록 rtgMSS의 waypoint 를 Initial Guess 결과 로 업데이트하는 함수
void retargetMusclesWaypointsInitialGuessing(std::shared_ptr<MusculoSkeletalSystem>& stdMSS,
                                             std::shared_ptr<MusculoSkeletalSystem>& rtgMSS);


/// stdMSS에 맞도록 rtgMSS의 waypoint 를 Calibrate 결과 로 업데이트하는 함수
void retargetTotalMusclesWaypointsCalibrating(std::shared_ptr<MusculoSkeletalSystem> &stdMSS,
                                              std::shared_ptr<MusculoSkeletalSystem> &rtgMSS,
                                              std::vector<SimpleMotion> &simpleMotions);

/// stdMSS에 맞도록 rtgMSS의 parameter 를 바꾸는 함수
void retargetMusclesParameters(std::shared_ptr<MusculoSkeletalSystem>& stdMSS,
                               std::shared_ptr<MusculoSkeletalSystem>& rtgMSS,
                               std::vector<SimpleMotion> &simpleMotions);


/// bvh에 맞춰서 parameter initial guess
void retargetMusclesParametersInitialGuessing(std::shared_ptr<MusculoSkeletalSystem>& stdMSS,
                                              std::shared_ptr<MusculoSkeletalSystem>& rtgMSS);

/// stdMSS에 맞도록 rtgMSS의 waypoint 와 parameter를 바꾸는 함수
void retargetMuscles(std::shared_ptr<MusculoSkeletalSystem>& stdMSS,
                     std::shared_ptr<MusculoSkeletalSystem>& rtgMSS,
                     std::vector<SimpleMotion> &simpleMotions);

/// 현재 rtgMSS의 Waypoint를 저장한다
void setWaypointsAsInitial(std::shared_ptr<MusculoSkeletalSystem> &rtgMSS);

///
void repairWaypoints(std::shared_ptr<MusculoSkeletalSystem> &rtgMSS);

#endif //SKELGEN_MUSCLEGENERATOR_H
