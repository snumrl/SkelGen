//
// Created by hoseok on 12/4/18.
//

#ifndef SKELGEN_FUNCTIONS_H
#define SKELGEN_FUNCTIONS_H

#include <vector>
#include <string>
#include <Eigen/Core>
#include "Eigen/Dense"
#include "../generator/SkeletonInfo.h"


std::vector<double> split_to_double(const std::string& input, int num);
std::vector<int> split_to_int(const std::string& input);
Eigen::Vector3d string_to_vector3d(const std::string& input);
std::vector<int> string_to_int_array(const std::string& input);

std::string toString(const std::vector<int> v);
std::string toString(const Eigen::VectorXd& v);
std::string toString(const Eigen::Vector3d& v);
std::string toString(const Eigen::Isometry3d& v);

Eigen::Matrix3d string_to_matrix3d(const std::string& input);

Eigen::Vector3d Proj(const Eigen::Vector3d& u,const Eigen::Vector3d& v);
Eigen::Isometry3d Orthonormalize(const Eigen::Isometry3d& T_old);

void ReadUserInput(std::vector<MetaBoneInfo>& mbis, std::string path);

Eigen::AngleAxisd makeAngleAxis(double theta, double phi, double roll);
Eigen::Quaterniond makeConicQuaternion(double theta, double phi, Eigen::Vector3d thetaAxis, Eigen::Vector3d phiAxis);
Eigen::Quaterniond makeConicQuaternion(double theta, double phi);
Eigen::Quaterniond makeAxialQuaternion(Eigen::Vector3d axis, double roll);
Eigen::Quaterniond makeQuaternion(double theta, double phi, double roll);

Eigen::Vector3d rotate(Eigen::Vector3d p, double theta, double phi, double roll);

#endif //SKELGEN_FUNCTIONS_H
