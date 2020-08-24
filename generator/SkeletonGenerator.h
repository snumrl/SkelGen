//
// Created by hoseok on 10/30/18.
//

#ifndef SKELGEN_SKELETONGENERATOR_H
#define SKELGEN_SKELETONGENERATOR_H

#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"

#include "SkeletonInfo.h"
#include "../model/DART_helper.h"
#include "../model/MusculoSkeletalSystem.h"
#include "../functions/functions.h"

/*
 * Input  : SKeleton parameter input
 * Output : Dart::SkeletonPtr
 */

void makeSkeletonMayaTextfile(SkeletonInfo skeletonInfo, std::string fileName = "skeleton.txt",
                              Eigen::Vector3d rootPosition = Eigen::Vector3d(0, 0, 0));
void makeSkeletonXml(std::string mayaPath, std::string xmlPath, std::string prefix="");
dart::dynamics::SkeletonPtr BuildFromFile(const std::string& filename, bool obj_visual_consensus,
											Eigen::Vector3d rootTranslation = Eigen::Vector3d(0,0,0));

void BuildFromSkeleton(const std::string& inputSkeletonXmlPath, const std::string& userInput, const std::string& outputSkeletonXmlPath,
                       std::vector<MetaBoneInfo>& mbis);



#endif //SKELGEN_SKELETONGENERATOR_H
