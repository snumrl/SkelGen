//
// Created by hoseok on 11/8/18.
//

#include "MeshObject.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

using namespace std;

FILE *out=fopen("glbalCoord.txt","w");


void MeshObject::calculateGlobalCoordinate(std::shared_ptr<MusculoSkeletalSystem> &mss){
	for (int i=1;i<vertices.size();i++){
		auto bn = mss->getSkeleton()->getBodyNode(name);
		assert(bn != nullptr);
		vertices[i] = bn->getTransform() * vertices[i];
//		fprintf(out,"%lf %lf %lf\n",vertices[i][0],vertices[i][1],vertices[i][2]);
	}

}

void MeshObject::readFile(std::string meshName) {
	name = meshName;
	if (meshName.find("rtg") != string::npos){
		name = meshName.substr(3);
	}
	FILE *in = fopen((string(MSS_ROOT_DIR)+"/model/standard/skeleton/"+meshName+".obj").c_str(), "r");

	char line[1005];
	while (fgets(line, 100, in) != NULL) {
		string str = string(line);
		if (line[0] == 'v' && line[1] == ' ') { // vertex
			char *p = strtok(line, " ");
			double x = 0, y = 0, z = 0;
			int cnt = 0;
			while (p) {
				if (cnt == 1) x = atof(p);
				if (cnt == 2) y = atof(p);
				if (cnt == 3) z = atof(p);
				cnt++;
				p = strtok(nullptr, " ");
			}
			vertices.emplace_back(Vertex(x, y, z));
		} else if (line[0] == 'f') { // face
			char *p = strtok(line, " ");
			p = strtok(nullptr, " ");
			Face face;
			while (p) {
				face.push(atoi(p));
				p = strtok(nullptr, " ");
			}
			faces.emplace_back(face);
		}
	}
}

int MeshObject::findClosestFace(Eigen::Vector3d p) {
	double minDist = 100000;
	int minIdx = 0;
	for (int i=1;i<faces.size();i++) {
		Vertex center = getCenter(faces[i]);
		double dist = (center - p).norm();
		if (minDist > dist) minDist = dist, minIdx = i;
	}
	return minIdx;
}

Vertex MeshObject::getCenter(Face &face) {
	Vertex center = Vertex(0, 0, 0);
	for (auto &vIdx: face.vertices) {
		center += vertices[vIdx];
	}
	return center / face.vertices.size();
}
