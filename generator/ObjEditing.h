//
// Created by hoseok on 11/19/18.
//

#ifndef SKELGEN_OBJEDITING_H
#define SKELGEN_OBJEDITING_H

#include "Eigen/Dense"
#include <stdio.h>

#include <vector>
#include <string>
#include <string.h>
#include <iostream>

#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"
#include "../functions/functions.h"

#include "SkeletonInfo.h"
#include <tinyxml.h>

double interpolation(double x, double y, double ratio);
void getValues(std::string obj_name, double *maxX, double *maxY, double *maxZ, double *minX, double *minY, double *minZ);
void boneMeshEditing(const std::string& stdSkeletonXmlPath, const std::string& rtgSkeletonXmlPath, const std::string& userInput);
void makeObjVisualConsensus(dart::dynamics::SkeletonPtr skel, const std::string& filename);


#endif //SKELGEN_OBJEDITING_H
