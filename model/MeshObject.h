//
// Created by hoseok on 11/8/18.
//

#ifndef SKELGEN_MESHOBJECT_H
#define SKELGEN_MESHOBJECT_H

#include <vector>
#include <string>
#include <Eigen/Dense>
#include "MusculoSkeletalSystem.h"

typedef Eigen::Vector3d Vertex;
class Face{
public:
	Face(){
		vertices.clear();
	}
	std::vector<int> vertices;

	void push(int x){
		vertices.push_back(x);
	}
};

class MeshObject {
public:
	MeshObject(){
		name = "";
		vertices.push_back(Vertex());
		faces.push_back(Face());
	}
	MeshObject(std::string meshName):MeshObject(){
		readFile(meshName);
	}
	std::string name;
	std::vector<Vertex> vertices;
	std::vector<Face> faces;

	void readFile(std::string fileName);
	void calculateGlobalCoordinate(std::shared_ptr<MusculoSkeletalSystem> &mss);
	int findClosestFace(Eigen::Vector3d p);
	Vertex getCenter(Face &face);
};


#endif //SKELGEN_MESHOBJECT_H
