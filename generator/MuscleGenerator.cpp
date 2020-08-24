//
// Created by hoseok on 11/8/18.
//

#include "MuscleGenerator.h"
#include <tinyxml.h>
#include "../model/MeshObject.h"
#include "./motionAdjust/MotionAdjust.h"
#include <string>

using namespace std;
using namespace dart::constraint;
using namespace dart::dynamics;
using namespace dart::simulation;


int nDirection = 0;
const int numIteration = 10000;
const int numSampling = 10;
const double learning_rate = 0.01;
const double lambdaShape = 0.1, lambdaLengthCurve = 0.1, lambdaRegular = 1.0;

vector<BlendedWaypoint> initialRetagertedBlendedWaypoints, newRetagertedBlendedWaypoints;

void MakeMuscles(const std::string& path,std::shared_ptr<MusculoSkeletalSystem>& ms)
{
	auto& skel = ms->getSkeleton();
	TiXmlDocument doc;
	if(!doc.LoadFile(path)){
		std::cout << "Can't open file : " << path << std::endl;
		return;
	}

	TiXmlElement *muscledoc = doc.FirstChildElement("Muscle");



	ms->originFlag.resize(1250);
	ms->insertionFlag.resize(1250);


	int waypointIdx = 0;
	for(TiXmlElement* unit = muscledoc->FirstChildElement("Unit");unit!=nullptr;unit = unit->NextSiblingElement("Unit"))
	{
		std::string name = unit->Attribute("name");
		double f_m_o = std::stod(unit->Attribute("f0"));
		double l_m_o = std::stod(unit->Attribute("lm"));
		double l_t_sl = std::stod(unit->Attribute("lt"));
		double pen_angle = std::stod(unit->Attribute("pen_angle"));
		std::vector<int> waypoint_indices;

		for(TiXmlElement* waypoint = unit->FirstChildElement("Waypoint");waypoint!=nullptr;waypoint = waypoint->NextSiblingElement("Waypoint"))
		{
//		int index = std::stoi(waypoint->Attribute("index"));
			std::string body = waypoint->Attribute("body");
			Eigen::Vector3d glob_pos = string_to_vector3d(waypoint->Attribute("p"));
//		auto T = skel->getBodyNode(body.c_str())->getShapeNodesWith<VisualAspect>()[0]->getRelativeTransform();
			auto T = skel->getBodyNode(body.c_str())->getTransform();
			Eigen::Vector3d body_local_coord;
			body_local_coord = T.inverse() * glob_pos;
			ms->mWaypoints.emplace_back(AnchorPoint(skel->getBodyNode(body.c_str()),body_local_coord));
			ms->mBlendedWaypoints.emplace_back(BlendedWaypoint(ms->mWaypoints.back()));

			waypoint_indices.emplace_back(waypointIdx++);

//		// Temporarily setting
//		auto& bwp = ms->mBlendedWaypoints.back();
//		bwp.relatedBodyNodes.emplace_back(ms->mWaypoints.back().first);
//		bwp.relativeCoords.emplace_back(ms->mWaypoints.back().second);
//		bwp.weights.emplace_back(1.0);
		}

		ms->addMuscle(name, waypoint_indices, f_m_o, l_m_o, l_t_sl, pen_angle);

		ms->originFlag[waypoint_indices.front()] = true;
		ms->insertionFlag[waypoint_indices.back()] = true;

	}
}

void retargetMusclesWaypointsLoading(shared_ptr<MusculoSkeletalSystem>& rtgMSS){

	cout << "Waypoint Loading Starting" << endl;
	auto& rtgWaypoints = rtgMSS->getWaypoints();
	for (int wpIdx = 0; wpIdx < rtgWaypoints.size(); wpIdx++) {
		auto& rtgWP = rtgWaypoints[wpIdx];

		auto rtgWPGlobalCoord = getAnchorPointGlobalCoord(rtgWP); // global coordinate of standard model's waypoint

		auto& bodynodeName = rtgWP.first->getName();
		int meshIdx=0;

//		int faceIdx                         = stdMeshObjects[meshIdx].findClosestFace(stdWPGlobalCoord);
//		Eigen::Vector3d vec                 = stdWPGlobalCoord - stdMeshObjects[meshIdx].getCenter(stdMeshObjects[meshIdx].faces[faceIdx]);
//		Eigen::Vector3d rtgWPGlobalCoord    = rtgMeshObjects[meshIdx].getCenter(rtgMeshObjects[meshIdx].faces[faceIdx]) + vec;
		rtgWP.second                        = rtgWP.first->getTransform().inverse() * rtgWPGlobalCoord;

		rtgMSS->getBlendedWaypoints()[wpIdx].ap = rtgWP;
	}


	rtgMSS->updateTotalWaypoints();
	cout << "Success" << endl;


	initialRetagertedBlendedWaypoints = rtgMSS->mBlendedWaypoints;

	cout << "Waypoint Loading Ending" << endl << endl;
}

/// Initial Guessing waypoints
void retargetMusclesWaypointsInitialGuessing(shared_ptr<MusculoSkeletalSystem>& stdMSS,
                              shared_ptr<MusculoSkeletalSystem>& rtgMSS) {
	cout << "Waypoint Initial Guessing Starting" << endl;

	// Goal: calculate closest face of mesh and following that.

	// 1. read objects
	cout << "Read Objects ----- ";
	vector<MeshObject> stdMeshObjects, rtgMeshObjects;
	TiXmlDocument stdDoc;
	if(!stdDoc.LoadFile(string(MSS_ROOT_DIR) + "/model/stdSkeleton.xml")){
		std::cout << "Can't open file : " << string(MSS_ROOT_DIR) + "/model/stdSkeleton.xml" << std::endl;
		exit(-1);
	}

	TiXmlElement *skeldoc = stdDoc.FirstChildElement("Skeleton");
	for(TiXmlElement *body = skeldoc->FirstChildElement("Joint"); body != nullptr; body = body->NextSiblingElement("Joint")){
		std::string objName = body->Attribute("obj");
		stdMeshObjects.emplace_back(MeshObject(objName.substr(0, objName.length()-4)));
	}

	TiXmlDocument rtgDoc;
	if(!rtgDoc.LoadFile(string(MSS_ROOT_DIR) + "/model/rtgSkeleton.xml")){
		std::cout << "Can't open file : " << string(MSS_ROOT_DIR) + "/model/rtgSkeleton.xml" << std::endl;
		exit(-1);
	}

	skeldoc = rtgDoc.FirstChildElement("Skeleton");
	for(TiXmlElement *body = skeldoc->FirstChildElement("Joint"); body != nullptr; body = body->NextSiblingElement("Joint")){
		std::string objName = body->Attribute("obj");
		rtgMeshObjects.emplace_back(MeshObject(objName.substr(0, objName.length()-4)));
	}

	int meshN = stdMeshObjects.size();
	// At this point, stdMeshObjects[i].vertices[j] is expressed in obj file format.
	for (int i=0;i<meshN;i++){
		stdMeshObjects[i].calculateGlobalCoordinate(stdMSS);
		rtgMeshObjects[i].calculateGlobalCoordinate(rtgMSS);
	}

	cout << "Success" << endl;

	// At this point, stdMeshObjects[i].vertices[j] is expressed in global coordinate.

	// 2. read blendedWaypoints

	cout << "Read Waypoints ----- ";
	auto& stdWaypoints = stdMSS->getWaypoints();
	auto& rtgWaypoints = rtgMSS->getWaypoints();
	cout << "Success" << endl;

	// 3. found closest face and calculate relative distance from it's center

	cout << "Initial Guessing ----- \n";
	for (int wpIdx = 0; wpIdx < stdWaypoints.size(); wpIdx++) {
		auto& stdWP = stdWaypoints[wpIdx];
		auto& rtgWP = rtgWaypoints[wpIdx];

		auto stdWPGlobalCoord = getAnchorPointGlobalCoord(stdWP); // global coordinate of standard model's waypoint

		auto& bodynodeName = stdWP.first->getName();
		int meshIdx=0;
		for (;meshIdx<meshN;meshIdx++){
			if (stdMeshObjects[meshIdx].name == bodynodeName){
				break;
			}
		}

		int faceIdx                         = stdMeshObjects[meshIdx].findClosestFace(stdWPGlobalCoord);
		Eigen::Vector3d vec                 = stdWPGlobalCoord - stdMeshObjects[meshIdx].getCenter(stdMeshObjects[meshIdx].faces[faceIdx]);
		Eigen::Vector3d rtgWPGlobalCoord    = rtgMeshObjects[meshIdx].getCenter(rtgMeshObjects[meshIdx].faces[faceIdx]) + vec;
		rtgWP.second                        = rtgWP.first->getTransform().inverse() * rtgWPGlobalCoord;

		rtgMSS->getBlendedWaypoints()[wpIdx].ap = rtgWP;
	}
	rtgMSS->updateTotalWaypoints();
	cout << "Success" << endl;


	initialRetagertedBlendedWaypoints = rtgMSS->mBlendedWaypoints;

	cout << "Waypoint Initial Guessing Ending" << endl << endl;

	return;

}

double getMuscleLength(shared_ptr<MusculoSkeletalSystem> &mss, double *config, shared_ptr<Muscle> m){
	double saveConfig[DOF];
	for (int i = 0; i < DOF; i++) {
		saveConfig[i] = mss->getSkeleton()->getPosition(i);
	}

	mss->setPose(config);

	double ret = m->getLength();

	for (int i = 0; i < DOF; i++) {
		mss->getSkeleton()->setPosition(i, saveConfig[i]);
	}

	return ret;
}

double getMuscleLengthDelta(shared_ptr<MusculoSkeletalSystem> &mss, SimpleMotion sm, double phase, shared_ptr<Muscle> m){
	double config[DOF] = {0,};
	configInterpolation(sm.startConfig, sm.endConfig, config, phase, DOF);
	
	return getMuscleLength(mss, config, m)-getMuscleLength(mss, sm.startConfig, m);
}

double fShape(shared_ptr<MusculoSkeletalSystem> &stdMSS,
                   shared_ptr<MusculoSkeletalSystem> &rtgMSS, SimpleMotion& sm, int muscleIdx) {
	double ret = 0.0;
	int cnt = 0;
	double config[DOF];

	auto &stdMuscle = stdMSS->getMuscles()[muscleIdx];
	auto &rtgMuscle = rtgMSS->getMuscles()[muscleIdx];
	int numWaypoint = stdMuscle->blendedWaypoints.size();

/*	for (int rep = 0; rep <= numSampling; rep++) {
		double phase = 1.0 * rep / numSampling;
		configInterpolation(sm.startConfig, sm.endConfig, config, phase, DOF);
		stdMSS->setPose(config);
		rtgMSS->setPose(config);

		// origin force direction
		Eigen::Vector3d stdVector, rtgVector;
		stdVector = getBlendedWaypointGlobalCoord(stdMuscle->blendedWaypoints[1]) -
		            getBlendedWaypointGlobalCoord(stdMuscle->blendedWaypoints[0]);
		rtgVector = getBlendedWaypointGlobalCoord(rtgMuscle->blendedWaypoints[1]) -
		            getBlendedWaypointGlobalCoord(rtgMuscle->blendedWaypoints[0]);
		stdVector.normalize();
		rtgVector.normalize();
		ret += (stdVector.cross(rtgVector)).norm();

		// insertion force direction
		stdVector = getBlendedWaypointGlobalCoord(stdMuscle->blendedWaypoints[numWaypoint-2]) -
		            getBlendedWaypointGlobalCoord(stdMuscle->blendedWaypoints[numWaypoint-1]);
		rtgVector = getBlendedWaypointGlobalCoord(rtgMuscle->blendedWaypoints[numWaypoint-2]) -
		            getBlendedWaypointGlobalCoord(rtgMuscle->blendedWaypoints[numWaypoint-1]);
		stdVector.normalize();
		rtgVector.normalize();
		ret += (stdVector.cross(rtgVector)).norm();
		cnt++;
	}*/
	for (int rep = 0; rep <= numSampling; rep++) {
		double phase = 1.0 * rep / numSampling;
		configInterpolation(sm.startConfig, sm.endConfig, config, phase, DOF);
		stdMSS->setPose(config);
		rtgMSS->setPose(config);

		for (int i=0;i<stdMuscle->blendedWaypoints.size()-1;i++){
			Eigen::Vector3d stdVector, rtgVector;
			stdVector = getBlendedWaypointGlobalCoord(stdMuscle->blendedWaypoints[i+1]) -
			            getBlendedWaypointGlobalCoord(stdMuscle->blendedWaypoints[i]);
			rtgVector = getBlendedWaypointGlobalCoord(rtgMuscle->blendedWaypoints[i+1]) -
			            getBlendedWaypointGlobalCoord(rtgMuscle->blendedWaypoints[i]);
			stdVector.normalize();
			rtgVector.normalize();
			ret += (stdVector.cross(rtgVector)).norm();
			cnt++;
		}
	}
	if (cnt == 0) cnt = 1;
	ret /= cnt;
	return ret;
}

double fLengthCurve(shared_ptr<MusculoSkeletalSystem> &stdMSS,
                   shared_ptr<MusculoSkeletalSystem> &rtgMSS, SimpleMotion& sm, int muscleIdx, bool flag = false) {
	double ret = 0.0;
	double config[DOF];

	auto &stdMuscle = stdMSS->getMuscles()[muscleIdx];
	auto &rtgMuscle = rtgMSS->getMuscles()[muscleIdx];

	double stdMinDelta = 10000, stdMaxDelta = -10000;
	double stdMinPhase, stdMaxPhase;

	double rtgMinDelta = 10000, rtgMaxDelta = -10000;
	double rtgMinPhase, rtgMaxPhase;

	for (int rep = 0; rep <= numSampling; rep++) {
		double phase = 1.0 * rep / numSampling;
//		configInterpolation(sm.startConfig, sm.endConfig, config, phase, DOF);
//		stdMSS->setPose(config);
//		rtgMSS->setPose(config);

		double stdDelta = getMuscleLengthDelta(stdMSS, sm, phase, stdMuscle);
		double rtgDelta = getMuscleLengthDelta(rtgMSS, sm, phase, rtgMuscle);

		if (stdDelta < stdMinDelta){
			stdMinDelta = stdDelta;
			stdMinPhase = phase;
		}
		if (stdDelta > stdMaxDelta){
			stdMaxDelta = stdDelta;
			stdMaxPhase = phase;
		}

		if (rtgDelta < rtgMinDelta){
			rtgMinDelta = rtgDelta;
			rtgMinPhase = phase;
		}
		if (rtgDelta > rtgMaxDelta){
			rtgMaxDelta = rtgDelta;
			rtgMaxPhase = phase;
		}
	}

	if (flag) {
//		cout << "-------------------------------------------\n";
//		cout << stdMinPhase << " / " << rtgMinPhase << endl;
//		cout << stdMaxPhase << " / " << rtgMaxPhase << endl;
//		cout << (stdMaxDelta - stdMinDelta) << " / " << (rtgMaxDelta - rtgMinDelta) << endl;
	}

//	ret = 0.5 * pow(stdMinPhase - rtgMinPhase, 2)
//		+ 0.5 * pow(stdMaxPhase - rtgMaxPhase, 2)
//		+ 5 * pow((stdMaxDelta - stdMinDelta) - (rtgMaxDelta - rtgMinDelta), 2);

	ret = 0.007 * pow(stdMinPhase - rtgMinPhase, 2)
		+ 0.007 * pow(stdMaxPhase - rtgMaxPhase, 2)
		+ 0.5 * pow((stdMaxDelta - stdMinDelta) - (rtgMaxDelta - rtgMinDelta), 2);

	return ret;
}

double calculateMetric(shared_ptr<MusculoSkeletalSystem> &stdMSS,
                       shared_ptr<MusculoSkeletalSystem> &rtgMSS,
                       vector<SimpleMotion> &simpleMotions,
                       int muscleIdx){
	int numTotalWaypoints = stdMSS->getWaypoints().size();
	int numMuscles = stdMSS->getNumMuscles();

	double shapeTerm = 0.0;
	int shapeCnt = 0;

	for (auto& sm: simpleMotions){
		bool relatedFlag = false;
		for (auto &relatedMuscleIdx : sm.relatedMuscleIndices)
			if (muscleIdx == relatedMuscleIdx)
				relatedFlag = true;

		if (relatedFlag) {
			shapeTerm += fShape(stdMSS, rtgMSS, sm, muscleIdx);
			shapeCnt++;
		}
	}

	double lengthCurveTerm = 0.0;
	int lengthCurveCnt = 0;

	for (auto& sm: simpleMotions){
		bool relatedFlag = false;
		for (auto &relatedMuscleIdx : sm.relatedMuscleIndices)
			if (muscleIdx == relatedMuscleIdx)
				relatedFlag = true;

		if (relatedFlag) {
			lengthCurveTerm += fLengthCurve(stdMSS, rtgMSS, sm, muscleIdx, true);
			lengthCurveCnt++;
		}
	}


	if (shapeCnt == 0) shapeCnt = 1;
	if (lengthCurveCnt == 0) lengthCurveCnt = 1;
//	cout << "shape : " << lambdaShape * shapeTerm / shapeCnt << " ; length curve : " << lambdaLengthCurve * lengthCurveTerm / lengthCurveCnt << endl;

	double ret = lambdaShape * shapeTerm / shapeCnt + lambdaLengthCurve * lengthCurveTerm / lengthCurveCnt;

	return ret;
}

double calculateMetricDerivative(shared_ptr<MusculoSkeletalSystem> &stdMSS,
                                 shared_ptr<MusculoSkeletalSystem> &rtgMSS,
                                 vector<SimpleMotion> &simpleMotions, int waypointIdx, int direction){
	double eps = 5e-3;

	int numTotalWaypoints = stdMSS->getWaypoints().size();
	int numMuscles = stdMSS->getNumMuscles();

	double shapeTerm = 0.0;
	int shapeCnt = 0;

	for (auto& sm: simpleMotions){
		for (int muscleIdx: sm.relatedMuscleIndices){
			double v = 0;
			rtgMSS->getWaypoints()[waypointIdx].second[direction] += eps, rtgMSS->updateSingleWaypoints(waypointIdx);
			v = fShape(stdMSS, rtgMSS, sm, muscleIdx);

			rtgMSS->getWaypoints()[waypointIdx].second[direction] -= eps * 2, rtgMSS->updateSingleWaypoints(waypointIdx);
			v -= fShape(stdMSS, rtgMSS, sm, muscleIdx);

			rtgMSS->getWaypoints()[waypointIdx].second[direction] += eps, rtgMSS->updateSingleWaypoints(waypointIdx);

			shapeTerm += v / (eps * 2.0);
			shapeCnt++;
		}
	}

	double lengthCurveTerm = 0.0;
	int lengthCurveCnt = 0;

	for (auto& sm: simpleMotions){
		for (int muscleIdx: sm.relatedMuscleIndices){
			double v = 0;
			rtgMSS->getWaypoints()[waypointIdx].second[direction] += eps, rtgMSS->updateSingleWaypoints(waypointIdx);
			v = fLengthCurve(stdMSS, rtgMSS, sm, muscleIdx);

			rtgMSS->getWaypoints()[waypointIdx].second[direction] -= eps * 2, rtgMSS->updateSingleWaypoints(waypointIdx);
			v -= fLengthCurve(stdMSS, rtgMSS, sm, muscleIdx);

			rtgMSS->getWaypoints()[waypointIdx].second[direction] += eps, rtgMSS->updateSingleWaypoints(waypointIdx);

			lengthCurveTerm += v / (eps * 2.0);
			lengthCurveCnt++;
		}
	}

//	cout << "shape : " << lambdaShape * shapeTerm / shapeCnt << " ; length curve : " << lambdaLengthCurve * lengthCurveTerm / lengthCurveCnt << endl;

	if (shapeCnt == 0) shapeCnt = 1;
	if (lengthCurveCnt == 0) lengthCurveCnt = 1;

	double ret = lambdaShape * shapeTerm / shapeCnt + lambdaLengthCurve * lengthCurveTerm / lengthCurveCnt;

	return ret;
}

void retargetSingleMusclesWaypointsCalibrating(std::shared_ptr<MusculoSkeletalSystem> &stdMSS,
                                               std::shared_ptr<MusculoSkeletalSystem> &rtgMSS,
                                               std::vector<SimpleMotion> &simpleMotions,
                                               int muscleIdx){

	auto stdMuscle = stdMSS->getMuscles()[muscleIdx];
	auto rtgMuscle = rtgMSS->getMuscles()[muscleIdx];
	cout << "Muscle : " << rtgMuscle->name << endl;
	int numWaypoints = rtgMuscle->waypoint_indices.size();
	auto &rtgMuscles = rtgMSS->getMuscles();

	// optimize W_r*
	for (int rep = 0; rep < numIteration; rep++) {

		double currentDifference = calculateMetric(stdMSS, rtgMSS,
		                                           simpleMotions, muscleIdx);

		vector<double> derivative(numWaypoints * 3);
		int derivativeIdx=0;
		for (int waypointIdx : rtgMuscle->waypoint_indices){
			// if waypoint is origin or insertion of any muscle, then it must be fixed.
			if (rtgMSS->originFlag[waypointIdx] || rtgMSS->insertionFlag[waypointIdx]) {
				derivativeIdx += 3;
				continue;
			}
			for (int dir = 0; dir < 3; dir++, derivativeIdx++) { // 0:x, 1:y, 2;z
				derivative[derivativeIdx] = calculateMetricDerivative(stdMSS, rtgMSS,
				                                                      simpleMotions, waypointIdx, dir);
			}
		}

		std::vector<AnchorPoint> currentWayPoints = rtgMSS->getWaypoints();
		double alpha = 0.1;
		int k;

		/* Line Search */
		if (OPT_MODE == "LINE_SEARCH") {
			for (k = 0; k < 16; k++) // I tried k<100, but result was pretty same as k<16
			{
				std::vector<AnchorPoint> nextWayPoints = currentWayPoints;
				int derivativeIdx=0;
				for (int waypointIdx : rtgMuscle->waypoint_indices){
					if (rtgMSS->originFlag[waypointIdx] || rtgMSS->insertionFlag[waypointIdx]) {
						derivativeIdx += 3;
						continue;
					}
					for (int dir = 0; dir < 3; dir++, derivativeIdx++) { // 0:x, 1:y, 2;z
						nextWayPoints[waypointIdx].second[dir] -= alpha * derivative[derivativeIdx];
					}
					rtgMSS->mWaypoints[waypointIdx] = nextWayPoints[waypointIdx];
					rtgMSS->updateSingleWaypoints(waypointIdx);
				}

//				rtgMSS->updateTotalWaypoints();

				double nextDifference = calculateMetric(stdMSS, rtgMSS,
				                                        simpleMotions, muscleIdx);

				if (nextDifference < currentDifference * 0.99)
					break;

				rtgMSS->mWaypoints = currentWayPoints;
				rtgMSS->updateTotalWaypoints();
				alpha *= 0.5;
			}
		}
			/***************/
		else {

			/* Gradient Method */

			std::vector<AnchorPoint> nextWayPoints = currentWayPoints;

			for (int waypointIdx=0, derivativeIdx=0 ; waypointIdx<numWaypoints ; waypointIdx++) {
				if (rtgMSS->originFlag[waypointIdx] || rtgMSS->insertionFlag[waypointIdx]) {
					derivativeIdx += 3;
					continue;
				}
				for (int dir = 0; dir < 3; dir++, derivativeIdx++) { // 0:x, 1:y, 2;z
					nextWayPoints[waypointIdx].second[dir] -= learning_rate * derivative[derivativeIdx];
				}
			}

			rtgMSS->mWaypoints = nextWayPoints;
			rtgMSS->updateTotalWaypoints();
		}



		/*******************/



		double nextDifference = calculateMetric(stdMSS, rtgMSS,
		                                        simpleMotions, muscleIdx);
		if(k==16) {
			if (rep>0) cout << "Iteration # " << rep << " Ends with diff " << nextDifference << endl;
			break;
		}

		if (rep%100==0 || rep==numIteration-1){
			cout << "@ " << rep + 1 << " / " << numIteration << endl;
		}
		cout << "cur Diff: " << currentDifference << " next Diff: " << nextDifference << endl;

	}

}

/// Calibrating waypoints
void retargetTotalMusclesWaypointsCalibrating(shared_ptr<MusculoSkeletalSystem> &stdMSS,
                                              shared_ptr<MusculoSkeletalSystem> &rtgMSS,
                                              vector<SimpleMotion> &simpleMotions) {

	cout << "Waypoint Calibrating Starting" << endl;
	int numMuscles = stdMSS->getNumMuscles();

	for (int i=0;i<numMuscles;i++){
		retargetSingleMusclesWaypointsCalibrating(stdMSS, rtgMSS, simpleMotions, i);
	}
	
	cout << "Waypoint Calibrating Ending" << endl << endl;
}
void retargetMusclesWaypointsInitialGuess(shared_ptr<MusculoSkeletalSystem> &stdMSS,
                                          shared_ptr<MusculoSkeletalSystem> &rtgMSS) {

	retargetMusclesWaypointsInitialGuessing(stdMSS,
	                                        rtgMSS);


//	retargetTotalMusclesWaypointsCalibrating(stdMSS,
//	                                    rtgMSS,
//	                                    simpleMotions);
}
void retargetMusclesParameters(std::shared_ptr<MusculoSkeletalSystem>& stdMSS,
                               std::shared_ptr<MusculoSkeletalSystem>& rtgMSS,
                               vector<SimpleMotion> &simpleMotions)
{
	retargetMusclesParametersInitialGuessing(stdMSS, rtgMSS);
}
void retargetMusclesParametersInitialGuessing(std::shared_ptr<MusculoSkeletalSystem>& stdMSS,
                                              std::shared_ptr<MusculoSkeletalSystem>& rtgMSS){
	// MotionAdjust* ma_std = new MotionAdjust("../generator/motionAdjust/PosePriorDataset/bvh", stdMSS);
	MotionAdjust* ma_rtg = new MotionAdjust("../generator/motionAdjust/PosePriorDataset/bvh", rtgMSS);
	// ma_std->SetAdjustmentValueForMotionFile(false, true, "STD");
	ma_rtg->SetAdjustmentValueForMotionFile(true, true, "RTG");
}

void retargetMuscles(shared_ptr<MusculoSkeletalSystem>& stdMSS,
                     shared_ptr<MusculoSkeletalSystem>& rtgMSS,
                     vector<SimpleMotion> &simpleMotions){
	retargetMusclesWaypointsInitialGuess(stdMSS, rtgMSS);
	retargetMusclesParameters(stdMSS, rtgMSS,  simpleMotions);
}

void setWaypointsAsInitial(shared_ptr<MusculoSkeletalSystem> &rtgMSS){
	newRetagertedBlendedWaypoints = rtgMSS->mBlendedWaypoints;
	rtgMSS->mBlendedWaypoints = initialRetagertedBlendedWaypoints;
	rtgMSS->updateBlendedWaypoints();

}

void repairWaypoints(shared_ptr<MusculoSkeletalSystem> &rtgMSS){
	rtgMSS->mBlendedWaypoints = newRetagertedBlendedWaypoints;
	rtgMSS->updateBlendedWaypoints();
}