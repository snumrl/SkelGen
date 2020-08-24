//
// Created by hoseok on 11/19/18.
//

#include "ObjEditing.h"
using namespace std;
using namespace dart::constraint;
using namespace dart::dynamics;
using namespace dart::simulation;

double interpolation(double x, double y, double ratio) {
	// return z s.t. (z-x) : (y-z) = ratio : (1.0-ratio)

	return x * (1 - ratio) + y * ratio;
}

void getValues(string obj_name, double *maxX, double *maxY, double *maxZ, double *minX, double *minY, double *minZ){
	FILE *in = fopen(obj_name.c_str(), "r");

	*maxX = *maxY = *maxZ = -100000;
	*minX = *minY = *minZ =  100000;

	char line[1005];
	while (fgets(line, 100, in) != NULL) {
		string str = string(line);
		if (line[0] == 'v' && line[1] == ' ') {
			char *p = strtok(line, " ");
			double x, y, z;
			int cnt = 0;
			while (p) {
				if (cnt == 1) x = atof(p);
				if (cnt == 2) y = atof(p);
				if (cnt == 3) z = atof(p);
				cnt++;
				p = strtok(NULL, " ");
			}

			*maxX = max(*maxX, x);
			*maxY = max(*maxY, y);
			*maxZ = max(*maxZ, z);

			*minX = min(*minX, x);
			*minY = min(*minY, y);
			*minZ = min(*minZ, z);

		}
	}

	fclose(in);
}

void boneMeshLengthening(string input_name, string output_name,
                     vector<Eigen::Isometry3d>& p, vector<Eigen::Isometry3d>& q) {

	FILE *in = fopen(input_name.c_str(), "r");
	FILE *out = fopen(output_name.c_str(), "w");

	const int n = p.size();
	const double threshold = 0.0;


	char line[1005];
 	while (fgets(line, 100, in) != NULL) {
		string str = string(line);
		if (line[0] == 'v' && line[1] == ' ') {
			char *l = strtok(line, " ");
			double x, y, z;
			int cnt = 0;
			while (l) {
				if (cnt == 1) x = atof(l);
				if (cnt == 2) y = atof(l);
				if (cnt == 3) z = atof(l);
				cnt++;
				l = strtok(NULL, " ");
			}

			Eigen::Vector3d point(x,y,z);


			vector<Eigen::Vector3d> points(n);
			vector<double> r(n), w(n);

			double sum=0;
			for (int i=0;i<n;i++){
				points[i] = p[i].inverse() * point;
				r[i] = points[i].norm();
				w[i] = 1.0/pow(r[i], 1.7);
				sum += w[i];
			}

			double alpha = 1.0/sum;
			for (int i=0;i<n;i++){
				w[i] *= alpha;
			}

			point = Eigen::Vector3d(0,0,0);
			for (int i=0;i<n;i++){
				point = point + w[i] * (q[i] * points[i]);
			}



			fprintf(out, "v %lf %lf %lf\n", point[0], point[1], point[2]);
		} else {
			fprintf(out, "%s", str.c_str());
		}
	}

	fclose(in);
	fclose(out);
}
typedef pair<pair<string,string>, pair<Eigen::Isometry3d,Eigen::Isometry3d> > NodeInfo;
void boneMeshEditing(const std::string& stdSkeletonXmlPath, const std::string& rtgSkeletonXmlPath, const std::string& userInput){
	vector< NodeInfo > stdJoints, rtgJoints;

	TiXmlDocument stdDoc, rtgDoc;
	if(!stdDoc.LoadFile(stdSkeletonXmlPath)){
		std::cout << "Can't open file : " << stdSkeletonXmlPath << std::endl;
		return;
	}
	if(!rtgDoc.LoadFile(rtgSkeletonXmlPath)){
		std::cout << "Can't open file : " << rtgSkeletonXmlPath << std::endl;
		return;
	}

	TiXmlElement *stdSkelDoc = stdDoc.FirstChildElement("Skeleton");


	for(TiXmlElement *body = stdSkelDoc->FirstChildElement("Joint"); body != nullptr; body = body->NextSiblingElement("Joint")){
		// type
		std::string jointType = body->Attribute("type");
		// name
		std::string name = body->Attribute("name");
		// parent name
		std::string parentName = body->Attribute("parent_name");
		std::string objName = "None";
		if(body->Attribute("obj")!=nullptr)
			objName = body->Attribute("obj");
		// size
		Eigen::Vector3d size = string_to_vector3d(std::string(body->Attribute("size")));
		// body position
		TiXmlElement *bodyPosElem = body->FirstChildElement("BodyPosition");
		Eigen::Isometry3d bodyPosition;
		bodyPosition.setIdentity();
		bodyPosition.linear() = string_to_matrix3d(bodyPosElem->Attribute("linear"));
		bodyPosition.translation() = string_to_vector3d(bodyPosElem->Attribute("translation"));
		bodyPosition = Orthonormalize(bodyPosition);
		// joint position
		TiXmlElement *jointPosElem = body->FirstChildElement("JointPosition");
		Eigen::Isometry3d jointPosition;
		jointPosition.setIdentity();
		if(jointPosElem->Attribute("linear")!=nullptr)
			jointPosition.linear() = string_to_matrix3d(jointPosElem->Attribute("linear"));
		else jointPosition.linear() = bodyPosition.linear();
		jointPosition.translation() = string_to_vector3d(jointPosElem->Attribute("translation"));
		jointPosition = Orthonormalize(jointPosition);

		stdJoints.push_back({{name, parentName}, {bodyPosition,jointPosition}});

	}

	TiXmlElement *rtgSkelDoc = rtgDoc.FirstChildElement("Skeleton");


	for(TiXmlElement *body = rtgSkelDoc->FirstChildElement("Joint"); body != nullptr; body = body->NextSiblingElement("Joint")){
		// type
		std::string jointType = body->Attribute("type");
		// name
		std::string name = body->Attribute("name");
		// parent name
		std::string parentName = body->Attribute("parent_name");
		std::string objName = "None";
		if(body->Attribute("obj")!=nullptr)
			objName = body->Attribute("obj");
		// size
		Eigen::Vector3d size = string_to_vector3d(std::string(body->Attribute("size")));
		// body position
		TiXmlElement *bodyPosElem = body->FirstChildElement("BodyPosition");
		Eigen::Isometry3d bodyPosition;
		bodyPosition.setIdentity();
		bodyPosition.linear() = string_to_matrix3d(bodyPosElem->Attribute("linear"));
		bodyPosition.translation() = string_to_vector3d(bodyPosElem->Attribute("translation"));
		bodyPosition = Orthonormalize(bodyPosition);
		// joint position
		TiXmlElement *jointPosElem = body->FirstChildElement("JointPosition");
		Eigen::Isometry3d jointPosition;
		jointPosition.setIdentity();
		if(jointPosElem->Attribute("linear")!=nullptr)
			jointPosition.linear() = string_to_matrix3d(jointPosElem->Attribute("linear"));
		else jointPosition.linear() = bodyPosition.linear();
		jointPosition.translation() = string_to_vector3d(jointPosElem->Attribute("translation"));
		jointPosition = Orthonormalize(jointPosition);

		rtgJoints.push_back({{name, parentName}, {bodyPosition,jointPosition}});
	}

	vector<MetaBoneInfo> mbis;
	ReadUserInput(mbis, "../model/userInput.txt");

	for (auto& mbi: mbis){
		Eigen::Isometry3d p1, p2, q1, q2;
		vector<Eigen::Isometry3d> p, q;
		/* INFO
		 * p1, q1: transformation of parent joint of target bone.
		 * p2, q2: transformation of child joint of target bone. if the bone is torso, choose neck's one.
		 */

		if (mbi.name != "Torso") {
			// 1. first, find child joint
			for (int i = 0; i < stdJoints.size(); i++) {
				if (stdJoints[i].first.second == mbi.name) {
					//				p2 = stdJoints[i].second.first.inverse() * stdJoints[i].second.second;
					//				q2 = rtgJoints[i].second.first.inverse() * rtgJoints[i].second.second;
					p2 = stdJoints[i].second.second;
					q2 = rtgJoints[i].second.second;
					break;
				}
			}
			for (int i = 0; i < stdJoints.size(); i++) {
				if (stdJoints[i].first.first == mbi.name) {
					//				p1 = stdJoints[i].second.first.inverse() * stdJoints[i].second.second;
					//				q1 = rtgJoints[i].second.first.inverse() * rtgJoints[i].second.second;
					p1 = stdJoints[i].second.second;
					q1 = rtgJoints[i].second.second;
					//				p1.translation() -= stdJoints[i].second.first.translation();
					//				q1.translation() -= rtgJoints[i].second.first.translation();
					//				p2.translation() -= stdJoints[i].second.first.translation();
					//				q2.translation() -= rtgJoints[i].second.first.translation();
					p1 = stdJoints[i].second.first.inverse() * p1;
					q1 = rtgJoints[i].second.first.inverse() * q1;
					p2 = stdJoints[i].second.first.inverse() * p2;
					q2 = rtgJoints[i].second.first.inverse() * q2;
					break;
				}
			}
			p.emplace_back(p1);p.emplace_back(p2);
			q.emplace_back(q1);q.emplace_back(q2);
			boneMeshLengthening(string(MSS_ROOT_DIR) + "/model/standard/skeleton/"+mbi.name+".obj",
			                    string(MSS_ROOT_DIR) + "/model/standard/skeleton/rtg"+mbi.name+".obj",
			                    p, q);
		}else{
			// FOR TORSO
			Eigen::Isometry3d stdTorsoBody, rtgTorsoBody, p3, q3;
			for (int i = 0; i < stdJoints.size(); i++) {
				if (stdJoints[i].first.first == "Torso") {
					stdTorsoBody = stdJoints[i].second.first;
					rtgTorsoBody = rtgJoints[i].second.first;
					p3 = stdJoints[i].second.second;
					q3 = rtgJoints[i].second.second;
					break;
				}
			}
			for (int i = 0; i < stdJoints.size(); i++) {
				if (stdJoints[i].first.first == "L_Arm") {
					p2 = stdJoints[i].second.second;
					q2 = rtgJoints[i].second.second;
					break;
				}
			}
			for (int i = 0; i < stdJoints.size(); i++) {
				if (stdJoints[i].first.first == "R_Arm") {
					p1 = stdJoints[i].second.second;
					q1 = rtgJoints[i].second.second;
					p1 = stdTorsoBody.inverse() * p1;
					q1 = rtgTorsoBody.inverse() * q1;
					p2 = stdTorsoBody.inverse() * p2;
					q2 = rtgTorsoBody.inverse() * q2;
					p3 = stdTorsoBody.inverse() * p3;
					q3 = rtgTorsoBody.inverse() * q3;
					break;
				}
			}
			p.emplace_back(p1);p.emplace_back(p2);p.emplace_back(p3);
			q.emplace_back(q1);q.emplace_back(q2);q.emplace_back(q3);
			boneMeshLengthening(string(MSS_ROOT_DIR) + "/model/standard/skeleton/"+mbi.name+".obj",
			                    string(MSS_ROOT_DIR) + "/model/standard/skeleton/rtg"+mbi.name+".obj",
			                    p, q);
		}
	}
//	skel->getBodyNode("Pelvis")->getParentJoint()->setPosition(3,rootTranslation[0]);
//	skel->setPosition(3,1.0);
//	skel->getJoint(0)->setPosition(0,rootTranslation[0]);
}

void boneMeshTransform(string input_name, string output_name,Eigen::Isometry3d T) {

	FILE *in = fopen(input_name.c_str(), "r");
	FILE *out = fopen(output_name.c_str(), "w");

//	cout << botShaft << " " << topShaft << endl;

	char line[1005];
	while (fgets(line, 100, in) != NULL) {
		string str = string(line);
		if (line[0] == 'v' && line[1] == ' ') {
			char *p = strtok(line, " ");
			double x, y, z;
			int cnt = 0;
			while (p) {
				if (cnt == 1) x = atof(p);
				if (cnt == 2) y = atof(p);
				if (cnt == 3) z = atof(p);
				cnt++;
				p = strtok(NULL, " ");
			}

			Eigen::Vector3d point(x,y,z);

			point = point * 0.01;
			point = T * point;

			fprintf(out, "v %lf %lf %lf\n", point[0], point[1], point[2]);
		}else if (line[0] == 'v' && line[1] == 'n') {
			char *p = strtok(line, " ");
			double x, y, z;
			int cnt = 0;
			while (p) {
				if (cnt == 1) x = atof(p);
				if (cnt == 2) y = atof(p);
				if (cnt == 3) z = atof(p);
				cnt++;
				p = strtok(NULL, " ");
			}

			Eigen::Vector3d point(x,y,z);

//			point = point * 0.01;
			Eigen::Isometry3d _T = T;
			_T.translation() = Eigen::Vector3d(0,0,0);
			point = _T * point;

			fprintf(out, "vn %lf %lf %lf\n", point[0], point[1], point[2]);
		} else {
			fprintf(out, "%s", str.c_str());
		}
	}

	fclose(in);
	fclose(out);
}


void makeObjVisualConsensus(SkeletonPtr skel, const std::string& filename){
	TiXmlDocument doc;
	if(!doc.LoadFile(filename)){
		std::cout << "Can't open file : " << filename << std::endl;
		return;
	}

	TiXmlElement *skeldoc = doc.FirstChildElement("Skeleton");

	std::string skelname = skeldoc->Attribute("name");

	for(TiXmlElement *body = skeldoc->FirstChildElement("Joint"); body != nullptr; body = body->NextSiblingElement("Joint")) {
		std::string objName = "None";
		if (body->Attribute("obj") != nullptr)
			objName = body->Attribute("obj");
		// body position
		TiXmlElement *bodyPosElem = body->FirstChildElement("BodyPosition");
		Eigen::Isometry3d bodyPosition;
		bodyPosition.setIdentity();
		bodyPosition.linear() = string_to_matrix3d(bodyPosElem->Attribute("linear"));
		bodyPosition.translation() = string_to_vector3d(bodyPosElem->Attribute("translation"));
		bodyPosition = Orthonormalize(bodyPosition);

		std::string input_obj_path = string(MSS_ROOT_DIR)+"/model/standard/OBJ_reduced/"+objName;
		std::string output_obj_path = string(MSS_ROOT_DIR)+"/model/standard/skeleton/"+objName;

		boneMeshTransform(input_obj_path, output_obj_path, bodyPosition.inverse());

	}
}