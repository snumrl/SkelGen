//
// Created by hoseok on 9/16/18.
//

#include "ExamWindow.h"
#include "../world/IntegratedWorld.h"
#include "../model/MusculoSkeletalSystem.h"
#include "../generator/MuscleGenerator.h"
#include "../generator/SkeletonInfo.h"
#include "../generator/ObjEditing.h"
#include <cstdlib>
using namespace std;
using muscle_vector = std::vector<std::shared_ptr<Muscle>>;




ExamWindow::ExamWindow():SimWindow(),test_type(test_type), smStep(30), posX(2.4), posZ(0) {

	readJointMap();

	rtgSkeletonInfo = SkeletonInfo();
//	FemurLength = 44.53391; // (cm), 0.4453391 cm in zygote
//  TibiaLength = 42.49466; // (cm), 0.4249466 cm in zygote
	rtgSkeletonInfo.R_Femur.alpha_lengthening = 2.0;
	rtgSkeletonInfo.L_Femur.alpha_lengthening = 1.0;
	rtgSkeletonInfo.R_Tibia.alpha_lengthening = 1.2;
	rtgSkeletonInfo.L_Tibia.alpha_lengthening = 1.0;
	smIdx = stepIdx = 0;

	/* Load Muscle */
//	mWorld->Initialize(POSE_TYPE::STAND, "Gorilla_opt.xml");
//	mWorld->mRtgMusculoSkeletalSystem->updateTotalWaypoints();

	/* New Skeleton */
	mWorld->Initialize();

	if (mUseMuscle) {
		retargetMusclesWaypointsInitialGuessing(mWorld->mStdMusculoSkeletalSystem,
		                                        mWorld->mRtgMusculoSkeletalSystem);
	}


	readSimpleMotion();


}

void ExamWindow::Keyboard(unsigned char key, int x, int y) {
	auto& waypoints = mWorld->GetCmpMusculoSkeletalSystem()->getWaypoints();
	auto& muscle = mWorld->GetCmpMusculoSkeletalSystem()->getMuscles()[0];
	auto& waypoint_indices= muscle->waypoint_indices;
    switch(key)
    {
		case 'd':
    		drawGraph();
    		break;
	    case 'n':
    		smIdx++;
    		stepIdx=0;
    		if (smIdx == simpleMotions.size()) smIdx=0;
    		break;
    	case 'm':
		    stepIdx++;
		    if (stepIdx == smStep) stepIdx=0;
		    break;
    	case 'v':
    		mRenderDetail = !mRenderDetail;
    		break;
    	case 'c':
		    if (mUseMuscle) {
			    retargetTotalMusclesWaypointsCalibrating(mWorld->mStdMusculoSkeletalSystem,
			                                             mWorld->mRtgMusculoSkeletalSystem,
			                                             simpleMotions);
			    drawGraph();
		    }
		    break;
    	case '=':
		    mInitialGuessViewFlag = !mInitialGuessViewFlag;
		    break;
    	case 's':
    		printf("File Name? (include \'.xml\') : ");
    		char fileName[100];
    		scanf("%s", fileName);
    		mWorld->mRtgMusculoSkeletalSystem->saveMuscleXML(fileName);
    		break;
	    case 'i':
		    posX+=0.1;
		    break;
	    case 'o':
		    posX-=0.1;
		    break;
	    case 'k':
		    posZ+=0.1;
		    break;
	    case 'l':
		    posZ-=0.1;
		    break;
	    case 'y':
		    highlightIdx++;
		    highlightIdx %= mWorld->mRtgMusculoSkeletalSystem->getMuscles().size();
		    break;
	    case 'u':
		    highlightIdx--;
		    highlightIdx += mWorld->mRtgMusculoSkeletalSystem->getMuscles().size();
		    highlightIdx %= mWorld->mRtgMusculoSkeletalSystem->getMuscles().size();
		    break;
    	case ' ':
    		printf("%s\n",mWorld->mRtgMusculoSkeletalSystem->getMuscles()[highlightIdx]->name.c_str());
    		break;

	    default :
            SimWindow::Keyboard(key, x, y);
            break;
    }
}


void ExamWindow::setStdPose(double *config){
	auto& stdSkel = mWorld->GetMusculoSkeletalSystem()->getSkeleton();
	for (int i=0;i<stdSkel->getNumDofs();i++){
		stdSkel->setPosition(i, config[i] * M_PI / 180);
	}
}

void ExamWindow::setRtgPose(double *config){
	auto& rtgSkel = mWorld->GetCmpMusculoSkeletalSystem()->getSkeleton();
	for (int i=0;i<rtgSkel->getNumDofs();i++){
		rtgSkel->setPosition(i, config[i] * M_PI / 180);
	}
//	rtgSkel->setPosition(3,0.7);
	rtgSkel->setPosition(3,posX);
	rtgSkel->setPosition(5,posZ);
}


void ExamWindow::Timer(int value) {
	double config[DOF];
	configInterpolation(simpleMotions[smIdx].startConfig, simpleMotions[smIdx].endConfig, config, 1.0*stepIdx/smStep, DOF);
	setStdPose(config);
	setRtgPose(config);
    SimWindow::Timer(value);
}

void ExamWindow::readJointMap() {
	FILE *in=fopen((string(MSS_ROOT_DIR) + "/simpleMotion/JointMap.txt").c_str(), "r");
	char line[1005];
	while (fgets(line, 100, in) != NULL) {
		string str = string(line);
		char *p = strtok(line, " \n");
		string name = p;
		p = strtok(NULL, " \n");
		int idx = atoi(p);
		jointMap[name]=idx;
	}

}

void ExamWindow::drawGraph(){
//	for (int i=0;i<simpleMotions.size();i++){
//		for (int j=0;j< mWorld->GetMusculoSkeletalSystem()->getNumMuscles();j++){
//			executePrintingNormalizedLmt(simpleMotions[i].motionName, mWorld->GetMusculoSkeletalSystem()->getMuscles()[j]->name);
//		}
//	}
}

void ExamWindow::executePrintingNormalizedLmt(string sm_name, string muscle_name) { // print (current l_mt) / (original l_mt)
    auto& stdMSS = mWorld->GetMusculoSkeletalSystem();
    auto& rtgMSS = mWorld->GetCmpMusculoSkeletalSystem();

    int smIdx = getSimpleMotionIdx(sm_name);
    assert(smIdx!=-1);
    auto& sm = simpleMotions[smIdx];

    int muscleIdx = getMuscleIdx(muscle_name);

	int step=smStep;
	// File format
	// Line 1: Graph name; Test name
	// Line 2: x-axis label
	// Line 3: y-axis label
	// Line 4: number of data, number of patches
	// Line 5 ~: data; x y
	string filePath = string(MSS_ROOT_DIR) + "/log/" + sm_name + " for " + muscle_name + ".txt";
	FILE *out=fopen(filePath.c_str(),"w");

//	printf("Normalized Lmt of %s Standard model and Retargeted model; SM: %s\n", muscle_name.c_str(), sm_name.c_str());

	fprintf(out,"Normalized Lmt of %s Standard model and Retargeted model; SM: %s\n", muscle_name.c_str(), sm_name.c_str());
    fprintf(out,"Angle\n");
    fprintf(out,"Normalized Lmt\n");
    fprintf(out,"%d %d\n", step, 3);

	double config[DOF];

    double stdLmt0 = 0, rtgLmt0 = 0, rtgLmtOrigin0 = 0;
	for (int i=0;i<step;i++) {
		configInterpolation(sm.startConfig, sm.endConfig, config, 1.0*i/step, DOF);
		setStdPose(config);
		setRtgPose(config);

        double stdLmt = 0, rtgLmt = 0, rtgLmtOrigin = 0;

        stdLmt = stdMSS->getMuscles()[muscleIdx]->getLength();
        rtgLmt = rtgMSS->getMuscles()[muscleIdx]->getLength();

		setWaypointsAsInitial(mWorld->mRtgMusculoSkeletalSystem);
		rtgLmtOrigin = rtgMSS->getMuscles()[muscleIdx]->getLength();
		repairWaypoints(mWorld->mRtgMusculoSkeletalSystem);

        if (i==0) stdLmt0 = stdLmt, rtgLmt0 = rtgLmt, rtgLmtOrigin0 = rtgLmtOrigin;

//		fprintf(out,"%lf %lf %lf %lf\n", 1.0*i/step, stdLmt, rtgLmt, rtgLmtOrigin);
		fprintf(out,"%lf %lf %lf %lf\n", 1.0*i/step, (stdLmt-stdLmt0)/stdLmt0, (rtgLmt-rtgLmt0)/rtgLmt0, (rtgLmtOrigin-rtgLmtOrigin0)/rtgLmtOrigin0);

    }

	fclose(out);

	system(("python3 "+string(MSS_ROOT_DIR)+"/pycode/plot.py \"" + filePath + "\"").c_str());
}


int ExamWindow::getMuscleIdx(string muscle_name){
	auto& stdMSS = mWorld->GetMusculoSkeletalSystem();
	for (int i=0;i< stdMSS->getNumMuscles();i++){
		if (muscle_name == stdMSS->getMuscles()[i]->name){
			return i;
		}
	}
//	assert(false && printf("Cannot find muscle name %s" , muscle_name.c_str()));
	return 0;
}

void ExamWindow::readSimpleMotion() {
	FILE *in=fopen((string(MSS_ROOT_DIR) + "/exams/SimpleMotionSet.txt").c_str(), "r");
	char line[1005];
	vector<SimpleMotion> _simpleMotions;
	while (fgets(line, 100, in) != NULL) {
		string str = string(line);
		if (line[0] == 'n') {
			char *p = strtok(line, " \n");
			p = strtok(NULL, " \n");
			_simpleMotions.emplace_back(SimpleMotion());
			_simpleMotions.back().motionName = p;
		} else if (line[0] == 'c'){
			char *p = strtok(line, " \n");
			p = strtok(NULL, " \n");
			int idx = jointMap[p];

			double s, e;
			p = strtok(NULL, " \n");
			s = atof(p);
			p = strtok(NULL, " \n");
			e = atof(p);

			_simpleMotions.back().startConfig[idx] = s;
			_simpleMotions.back().endConfig[idx] = e;
		} else if (line[0]=='p'){
			char *p = strtok(line, " \n");
			p = strtok(NULL, " \n");
			string motionName = p;
			for (auto& _sm: _simpleMotions){
				if (_sm.motionName == motionName){
					simpleMotions.emplace_back(_sm);
					break;
				}
			}
		}
	}
}

int ExamWindow::getSimpleMotionIdx(std::string sm_name) {
	for (int i=0;i<simpleMotions.size();i++){
		if (simpleMotions[i].motionName == sm_name) return i;
	}
	return -1;
}

void ExamWindow::updateTargetSkeleton() {

//	printUserInput(rtgSkeletonInfo);

	mWorld->updateTargetSkeleton();

	if (mUseMuscle) {
		retargetMuscles(mWorld->mStdMusculoSkeletalSystem,
		                mWorld->mRtgMusculoSkeletalSystem,
		                simpleMotions);
	}
}
