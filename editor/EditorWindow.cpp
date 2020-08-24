//
// Created by hoseok on 9/16/18.
//

#include "EditorWindow.h"
#include "../world/IntegratedWorld.h"
#include "../model/MusculoSkeletalSystem.h"
#include "../generator/MuscleGenerator.h"
#include "../generator/SkeletonInfo.h"
#include "../generator/ObjEditing.h"
#include "../functions/functions.h"
#include <cstdlib>
using namespace std;
using namespace GUI;
using namespace dart;
using namespace dart::dynamics;
using muscle_vector = std::vector<std::shared_ptr<Muscle>>;
 



EditorWindow::EditorWindow():SimWindow(), smStep(30), mode(MODE::SKELETON_EDIT), selectedSkeletonID(-1), isEditingSkeleton(false),
													selectedROMID(-1), isEditingROM(false), romSphereDrag(false){

	ID2Name[ID_Pelvis]="Pelvis";
	ID2Name[ID_R_Femur]="R_Femur";
	ID2Name[ID_R_Tibia]="R_Tibia";
	ID2Name[ID_R_Foot]="R_Foot";
	ID2Name[ID_L_Femur]="L_Femur";
	ID2Name[ID_L_Tibia]="L_Tibia";
	ID2Name[ID_L_Foot]="L_Foot";
	ID2Name[ID_Spine]="Spine";
	ID2Name[ID_Torso]="Torso";
	ID2Name[ID_Neck]="Neck";
	ID2Name[ID_Head]="Head";
	ID2Name[ID_L_Shoulder]="L_Shoulder";
	ID2Name[ID_L_Arm]="L_Arm";
	ID2Name[ID_L_ForeArm]="L_ForeArm";
	ID2Name[ID_L_Hand]="L_Hand";
	ID2Name[ID_R_Shoulder]="R_Shoulder";
	ID2Name[ID_R_Arm]="R_Arm";
	ID2Name[ID_R_ForeArm]="R_ForeArm";
	ID2Name[ID_R_Hand]="R_Hand";

	name2ID["Pelvis"]=ID_Pelvis;
	name2ID["R_Femur"]=ID_R_Femur;
	name2ID["R_Tibia"]=ID_R_Tibia;
	name2ID["R_Foot"]=ID_R_Foot;
	name2ID["L_Femur"]=ID_L_Femur;
	name2ID["L_Tibia"]=ID_L_Tibia;
	name2ID["L_Foot"]=ID_L_Foot;
	name2ID["Spine"]=ID_Spine;
	name2ID["Torso"]=ID_Torso;
	name2ID["Neck"]=ID_Neck;
	name2ID["Head"]=ID_Head;
	name2ID["L_Shoulder"]=ID_L_Shoulder;
	name2ID["L_Arm"]=ID_L_Arm;
	name2ID["L_ForeArm"]=ID_L_ForeArm;
	name2ID["L_Hand"]=ID_L_Hand;
	name2ID["R_Shoulder"]=ID_R_Shoulder;
	name2ID["R_Arm"]=ID_R_Arm;
	name2ID["R_ForeArm"]=ID_R_ForeArm;
	name2ID["R_Hand"]=ID_R_Hand;


	smIdx = stepIdx = 0;

	/* Load Muscle */
//	mWorld->Initialize(POSE_TYPE::STAND, "Gorilla_opt.xml");
//	mWorld->mRtgMusculoSkeletalSystem->updateTotalWaypoints();

	/* New Skeleton */
	cout << "New Skeleton (new, n) or Load Skeleton (load, l)? ";
	string reply;
	cin >> reply;

	loadFlag = false;

	if (reply == "load" || reply=="l"){
		string fileName;
		cout << "File Name(ex, Gorilla)? ";
		cin >> fileName;
		ReadUserInput(mWorld->mbis, "../model/userInput_" + fileName + ".txt");

		cout << "New Muscle (new, n) or Load Muscle (load, l)? ";
		cin >> reply;
		if (reply == "load" || reply=="l") {
			string fileName;
			cout << "File Name(ex, Gorilla.xml)? ";
			cin >> fileName;
			mWorld->Initialize(POSE_TYPE::STAND, fileName);
			retargetMusclesWaypointsLoading(mWorld->mRtgMusculoSkeletalSystem);
			loadFlag = true;
		}else{
			mWorld->Initialize();
//			retargetMusclesWaypointsInitialGuessing(mWorld->mStdMusculoSkeletalSystem,
//													mWorld->mRtgMusculoSkeletalSystem);
		}
	}else {
		mWorld->Initialize();
//		retargetMusclesWaypointsInitialGuessing(mWorld->mStdMusculoSkeletalSystem,
//												mWorld->mRtgMusculoSkeletalSystem);
	}
	// baby
//	mWorld->mRtgMusculoSkeletalSystem->saveSkeletonText(0.4, mWorld->mRtgMusculoSkeletalSystem->getSkeleton()->getBodyNode(0)->getTransform().translation());
//	mWorld->mRtgMusculoSkeletalSystem->saveMuscleXML("baby_muscle.xml", 0.4, mWorld->mRtgMusculoSkeletalSystem->getSkeleton()->getBodyNode(0)->getTransform().translation());
//	mWorld->mRtgMusculoSkeletalSystem->saveSkeletonXml("baby_skeleton.xml", 0.4, mWorld->mRtgMusculoSkeletalSystem->getSkeleton()->getBodyNode(0)->getTransform().translation());
	// normal
//	mWorld->mRtgMusculoSkeletalSystem->saveSkeletonText(1.0, mWorld->mRtgMusculoSkeletalSystem->getSkeleton()->getBodyNode(0)->getTransform().translation());
	// teen
//	mWorld->mRtgMusculoSkeletalSystem->saveSkeletonText(0.7, mWorld->mRtgMusculoSkeletalSystem->getSkeleton()->getBodyNode(0)->getTransform().translation());


	EditInfoDB *edb = new EditInfoDB();

	ROM = new MSSROM(mWorld->mRtgMusculoSkeletalSystem, edb);

	readSimpleMotion();

//	findSimpleMotionRelatedMuscleIndices();

//	exit(1);

	// TEMP
	mIsDrawingSkeleton = true;
	nonMuscleOn = true;

	R_Femur =0.0;
	R_Tibia =0.0;
}

void EditorWindow::setInitialCameraView()
{
	mCamera->Zoom(0,0,0,430);
	// mCamera->Rotate(0, 300, 0, 0);
}

void EditorWindow::Keyboard(unsigned char key, int x, int y) {
	auto &waypoints = mWorld->GetCmpMusculoSkeletalSystem()->getWaypoints();
	auto &muscle = mWorld->GetCmpMusculoSkeletalSystem()->getMuscles()[0];
	auto &waypoint_indices = muscle->waypoint_indices;
	switch (key) {
		case 'g':
			R_Tibia = M_PI/2.0;
			break;
		case 'h':
			R_Tibia = 0.0;
			break;
		case 'q':
			nonMuscleOn = !nonMuscleOn;
			break;
		case 'd':
			drawGraph();
			break;
		case 'n':
			smIdx++;
			stepIdx = 0;
			if (smIdx == simpleMotions.size()) smIdx = 0;
			break;
		case 'm':
			stepIdx++;
			if (stepIdx == smStep) stepIdx = 0;
/*			for(auto& muscle : ROM->ms->getMuscles())
			{
				if(muscle->name == "R_Psoas_Major" ||
				   muscle->name == "L_Bicep_Brachii_Short_Head" )
				{
					cout<<muscle->name<<" "<<muscle->getLength()<<endl;g
				}
			}*/
			break;
		case 'v':
			mIsDrawingSkeleton = !mIsDrawingSkeleton;
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
		case '[':
			if (mode == MODE::SKELETON_EDIT && isEditingSkeleton) {
				for (int i = 0; i < mWorld->mbis.size(); i++) {
					if (mWorld->mbis[i].name == ID2Name[editingSkeletonID]) {
						mWorld->mbis[i].alpha_lengthening += 0.1;
						cout << "alpha_lengthening : " << mWorld->mbis[i].alpha_lengthening << endl;
						updateTargetSkeleton();
					}
				}
			}else if (mode == MODE::ROM_EDIT && isEditingROM && editingROM->type==JOINT_TYPE::BALL_AND_SOCKET){
				// editingROM->conicScale += 0.1;
				// if (editingROM->conicScale >= 1.0) editingROM->conicScale = 1.0;
			}else if (mode == MODE::ROM_EDIT && isEditingROM && editingROM->type==JOINT_TYPE::REVOLUTE){
				editingROM->revoluteEditInfo.scale += 0.1;
				if (editingROM->revoluteEditInfo.scale  >= 1.0) editingROM->revoluteEditInfo.scale  = 1.0;
			}
			break;
		case ']':
			if (mode == MODE::SKELETON_EDIT && isEditingSkeleton) {
				for (int i = 0; i < mWorld->mbis.size(); i++) {
					if (mWorld->mbis[i].name == ID2Name[editingSkeletonID]) {
						mWorld->mbis[i].alpha_lengthening -= 0.1;
						if (mWorld->mbis[i].alpha_lengthening < 0.1) mWorld->mbis[i].alpha_lengthening = 0.1;
						cout << "alpha_lengthening : " << mWorld->mbis[i].alpha_lengthening << endl;
						updateTargetSkeleton();
					}
				}
			}else if (mode == MODE::ROM_EDIT && isEditingROM && editingROM->type==JOINT_TYPE::BALL_AND_SOCKET){
				// editingROM->conicScale -= 0.1;
				// if (editingROM->conicScale <= 0.1) editingROM->conicScale = 0.1;
			}else if (mode == MODE::ROM_EDIT && isEditingROM && editingROM->type==JOINT_TYPE::REVOLUTE){
				editingROM->revoluteEditInfo.scale -= 0.1;
				if (editingROM->revoluteEditInfo.scale  <= 0.1) editingROM->revoluteEditInfo.scale  = 0.1;
			}
			break;
		case ';':
			if (mode == MODE::SKELETON_EDIT && isEditingSkeleton) {
				for (int i = 0; i < mWorld->mbis.size(); i++) {
					if (mWorld->mbis[i].name == ID2Name[editingSkeletonID]) {
						mWorld->mbis[i].theta_distal += 5;
						cout << "alpha_lengthening : " << mWorld->mbis[i].alpha_lengthening << endl;
						updateTargetSkeleton();
					}
				}
			}else if (mode == MODE::ROM_EDIT && isEditingROM && editingROM->type==JOINT_TYPE::BALL_AND_SOCKET){
				editingROM->conicTranslate = Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d(1,0,0))) * editingROM->conicTranslate;
			}else if (mode == MODE::ROM_EDIT && isEditingROM && editingROM->type==JOINT_TYPE::REVOLUTE){
				editingROM->revoluteEditInfo.translate += 0.1;
			}
			break;
		case '\'':
			if (mode == MODE::SKELETON_EDIT && isEditingSkeleton) {
				for (int i = 0; i < mWorld->mbis.size(); i++) {
					if (mWorld->mbis[i].name == ID2Name[editingSkeletonID]) {
						mWorld->mbis[i].theta_distal -= 5;
						cout << "alpha_lengthening : " << mWorld->mbis[i].alpha_lengthening << endl;
						updateTargetSkeleton();
					}
				}
			}else if (mode == MODE::ROM_EDIT && isEditingROM && editingROM->type==JOINT_TYPE::BALL_AND_SOCKET){
				editingROM->conicTranslate = Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d(0,1,0))) * editingROM->conicTranslate;
			}else if (mode == MODE::ROM_EDIT && isEditingROM && editingROM->type==JOINT_TYPE::REVOLUTE){
				editingROM->revoluteEditInfo.translate -= 0.1;
			}
			break;
		case 'y':
			highlightIdx++;
			highlightIdx %= mWorld->mRtgMusculoSkeletalSystem->getMuscles().size();
			cout << "Highlight : " << mWorld->mRtgMusculoSkeletalSystem->getMuscles()[highlightIdx]->name << endl;
			break;
		case 'u':
			highlightIdx--;
			highlightIdx += mWorld->mRtgMusculoSkeletalSystem->getMuscles().size();
			highlightIdx %= mWorld->mRtgMusculoSkeletalSystem->getMuscles().size();
			cout << "Highlight : " << mWorld->mRtgMusculoSkeletalSystem->getMuscles()[highlightIdx]->name<< endl;
			break;
		case ' ':
			switch (mode) {
				case MODE::SKELETON_EDIT:
					mode = MODE::MUSCLE_GENERATE;
					mUseMuscle = true;
					if (!loadFlag)
						retargetMusclesWaypointsInitialGuessing(mWorld->mStdMusculoSkeletalSystem,
						                                        mWorld->mRtgMusculoSkeletalSystem);

					break;
				case MODE::MUSCLE_GENERATE: {
					mode = MODE::MUSCLE_CALIBRATE;
					cout << "Want to calibrate? (y/n)" << endl;
					string ans;
					cin >> ans;
					if (ans == "y") {
						retargetTotalMusclesWaypointsCalibrating(mWorld->mStdMusculoSkeletalSystem,
																 mWorld->mRtgMusculoSkeletalSystem,
																 simpleMotions);
						mWorld->mRtgMusculoSkeletalSystem->saveMuscleXML("AUTOSAVE.xml");
						drawGraph();
					}
					break;
				}
				case MODE::MUSCLE_CALIBRATE:
					mode = MODE::ROM_EDIT;
					retargetMusclesParameters(mWorld->mStdMusculoSkeletalSystem,
					                          mWorld->mRtgMusculoSkeletalSystem,
					                          simpleMotions);
					ROM->ms =  mWorld->mRtgMusculoSkeletalSystem;
					ROM->InitROM();
					// ROM->setComfortablePosition();
					computeROMs();
					cout<<"q : Non-Muscle cuting ON/OF"<<endl;
/*					for(auto& muscle : ROM->ms->getMuscles())
					{
						if(muscle->name == "R_Psoas_Major" ||
							muscle->name == "L_Bicep_Brachii_Short_Head" )
						{
							cout<<muscle->name<<" "<<muscle->max_l_mt<<endl;
						}
					}*/

					break;
				case MODE::ROM_EDIT:
					mode = MODE::ROM_EDIT;
					convertUserInputIntoEDB();
					ROMEditor::editMPwithROM(ROM->ms, ROM->edb);
					// ROM->setComfortablePosition();
					computeROMs();
					break;
			}
			break;

		default :
			SimWindow::Keyboard(key, x, y);
			break;
	}
}

void EditorWindow::setStdPose(double *config){
	auto& stdSkel = mWorld->GetMusculoSkeletalSystem()->getSkeleton();
	for (int i=0;i<stdSkel->getNumDofs();i++){
		stdSkel->setPosition(i, config[i] * M_PI / 180);
	}
}

void EditorWindow::setRtgPose(double *config){
	auto& rtgSkel = mWorld->GetCmpMusculoSkeletalSystem()->getSkeleton();
	for (int i=0;i<rtgSkel->getNumDofs();i++){
		rtgSkel->setPosition(i, config[i] * M_PI / 180);
	}
}


void EditorWindow::Display() {
	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	initLights();
	mCamera->Apply();
	glDisable(GL_LIGHTING);
	glLineWidth(2.0);
	glLineWidth(1.0);
	glColor3f(0,0,0);
	glLineWidth(1.0);


	Eigen::Vector3d clr[5] =
			{
					Eigen::Vector3d(0.8,0.2,0.2),
					Eigen::Vector3d(0.2,0.8,0.2),
					Eigen::Vector3d(0.2,0.2,0.8),
					Eigen::Vector3d(0.8,0.8,0.2),
					Eigen::Vector3d(0.2,0.8,0.8)
			};
	int ball_index = 0;
	if (mIsDrawingSkeleton) GUI::DrawSkeleton(mWorld->GetRigidWorld()->getSkeleton(1), name2ID, Eigen::Vector3d(0.1, 1.0, 0.1), !mRenderDetail);
	// Draw muscles
	if((mRenderDetail || !mRenderDetail) && mUseMuscle)
	{
		/*for (int i=0;i<mWorld->GetCmpMusculoSkeletalSystem()->getMuscles().size();i++){
			auto& mus = mWorld->GetCmpMusculoSkeletalSystem()->getMuscles()[i];
			GUI::DrawMuscleWayPoints(mus, qobj, false, false);
		}*/

//		auto& mus = mWorld->GetCmpMusculoSkeletalSystem()->getMuscles()[highlightIdx];
//		GUI::DrawMuscleWayPoints(mus, qobj, false, false);

		for (int i=0;i<mWorld->GetCmpMusculoSkeletalSystem()->getMuscles().size();i++){
			auto& mus = mWorld->GetCmpMusculoSkeletalSystem()->getMuscles()[i];
			int flag=0;

			for (int muscleIdx: simpleMotions[smIdx].relatedMuscleIndices){
				if (i == muscleIdx) flag=1;
			}

//			if (flag==1) GUI::DrawMuscleWayPoints(mus, qobj, false, highlightIdx == i);
			GUI::DrawMuscleWayPoints(mus, qobj, false, highlightIdx == i);
		}

		setWaypointsAsInitial(mWorld->mRtgMusculoSkeletalSystem);
		for (auto &mus: mWorld->GetCmpMusculoSkeletalSystem()->getMuscles()) {
			DrawMuscleWayPoints(mus, qobj, true);
		}
		repairWaypoints(mWorld->mRtgMusculoSkeletalSystem);
	}
	switch (mode){
		case MODE::SKELETON_EDIT:
//			GUI::DrawStringOnScreen(0.8,0.8,"Skeleton Editing",true,Eigen::Vector3d(0,0,0));
			if (isEditingSkeleton) {
				for (int i=0;i<mWorld->mbis.size();i++){
					if (mWorld->mbis[i].name == ID2Name[editingSkeletonID]){
						GUI::DrawStringOnScreen(0.8,0.75,ID2Name[editingSkeletonID],true,Eigen::Vector3d(0,0,0));
						GUI::DrawStringOnScreen(0.8,0.70,"length : " + std::to_string(mWorld->mbis[i].alpha_lengthening),true,Eigen::Vector3d(0,0,0));
						GUI::DrawStringOnScreen(0.8,0.65,"twist  : " + std::to_string(mWorld->mbis[i].theta_distal),true,Eigen::Vector3d(0,0,0));

					}
				}

//				GUI::DrawStringOnScreen(0.8,0.65,"twist  : " + std::to_string(mWorld->mbis[i].theta_distal),true,Eigen::Vector3d(0,0,0));
			}
			break;
		case MODE::MUSCLE_GENERATE:
//			GUI::DrawStringOnScreen(0.8,0.8,"Muscle Initialize",true,Eigen::Vector3d(0,0,0));
			break;
		case MODE::MUSCLE_CALIBRATE:
//			GUI::DrawStringOnScreen(0.8,0.8,"Muscle Calibrate",true,Eigen::Vector3d(0,0,0));
			break;
		case MODE::ROM_EDIT:
//			GUI::DrawStringOnScreen(0.8,0.8,"ROM Editing",true,Eigen::Vector3d(0,0,0));
			loadROMs();
			if (isEditingROM){
				GUI::DrawStringOnScreen(0.8, 0.75, editingROM->name, true, Eigen::Vector3d(0, 0, 0));
				if (editingROM->type == JOINT_TYPE::BALL_AND_SOCKET)
					GUI::DrawStringOnScreen(0.8, 0.70, "scale : " + std::to_string(editingROM->conicScale), true,
					                        Eigen::Vector3d(0, 0, 0));

				if (editingROM->type == JOINT_TYPE::REVOLUTE) {
					GUI::DrawStringOnScreen(0.8, 0.70, "scale : " + std::to_string(editingROM->revoluteEditInfo.scale),
					                        true, Eigen::Vector3d(0, 0, 0));

					GUI::DrawStringOnScreen(0.8, 0.65, "translate : " + std::to_string(editingROM->revoluteEditInfo.translate),
					                        true, Eigen::Vector3d(0, 0, 0));
				}
			}
			break;
		case MODE::MUSCLE_PARAMETER_ADAPTING:
			GUI::DrawStringOnScreen(0.8,0.8,"Muscle Parameter Adapting",true,Eigen::Vector3d(0,0,0));
			loadROMs();
			break;

	}





	// const unsigned char text[10] = "Hello";
	// RenderString(-1.0, 1.0, GLUT_BITMAP_TIMES_ROMAN_24, text, Eigen::Vector3d(1.0f, 0.0f, 0.0f));
	glutSwapBuffers();
}

void EditorWindow::Timer(int value) {
	// double config[DOF];
	// configInterpolation(simpleMotions[smIdx].startConfig, simpleMotions[smIdx].endConfig, config, 1.0*stepIdx/smStep, DOF);
	// setStdPose(config);
	// setRtgPose(config);
	// cout << mWorld->GetCmpMusculoSkeletalSystem()->getMuscles()[0]->getLength() << endl;

	    Eigen::VectorXd zeroPosition;
	zeroPosition = ROM->ms->getSkeleton()->getPositions();
	zeroPosition.setZero();

	int indexRTibia = ROM->ms->getSkeleton()->getIndexOf(ROM->ms->getSkeleton()->getBodyNode("R_Tibia")->getParentJoint()->getDof(0));

	zeroPosition[indexRTibia] = R_Tibia;

	ROM->ms->getSkeleton()->setPositions(zeroPosition);


	SimWindow::Timer(value);
}

void EditorWindow::readJointMap() {
	FILE *in=fopen((string(MSS_ROOT_DIR) + "/simpleMotion/JointMap.txt").c_str(), "r");
	char line[1005];
	while (fgets(line, 100, in) != NULL) {
		string str = string(line);
		char *p = strtok(line, " \n");
		string name = p;
		p = strtok(NULL, " \n");
		string bnName = p;
		p = strtok(NULL, " \n");
		int idx = atoi(p);
		dart::dynamics::BodyNode* bn = mWorld->mRtgMusculoSkeletalSystem->getSkeleton()->getBodyNode(bnName);
		int offset = bn->getParentJoint()->getDof(0)->getIndexInSkeleton();
		jointMap[name] = offset + idx;
	}

}

void EditorWindow::drawGraph(){
	for (int i=0;i<simpleMotions.size();i++){
		if (simpleMotions[i].motionName == "T_Pose") continue;
		if (simpleMotions[i].motionName == "Stand_Pose") continue;
		for (int j=0;j< mWorld->GetMusculoSkeletalSystem()->getNumMuscles();j++){
			int flag=0;

			for (int muscleIdx: simpleMotions[i].relatedMuscleIndices){
				if (j == muscleIdx) flag=1;
			}
			if (flag)
				executePrintingNormalizedLmt(simpleMotions[i].motionName,
			                             mWorld->GetMusculoSkeletalSystem()->getMuscles()[j]->name);
		}
	}
}

void EditorWindow::executePrintingNormalizedLmt(string sm_name, string muscle_name) { // print (current l_mt) / (original l_mt)
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
	fprintf(out,"%d %d\n", step+1, 3);

	double config[DOF];

	double stdLmt0 = 0, rtgLmt0 = 0, rtgLmtOrigin0 = 0;
	for (int i=0;i<=step;i++) {
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
		fprintf(out,"%lf %lf %lf %lf\n", 1.0*i/step, stdLmt-stdLmt0, rtgLmt-rtgLmt0, rtgLmtOrigin-rtgLmtOrigin0);

	}

	fclose(out);

	system(("python3 "+string(MSS_ROOT_DIR)+"/pycode/plot.py \"" + filePath + "\"").c_str());
}


int EditorWindow::getMuscleIdx(string muscleName){
	auto& stdMSS = mWorld->GetMusculoSkeletalSystem();
	for (int i=0;i< stdMSS->getNumMuscles();i++){
		if (muscleName == stdMSS->getMuscles()[i]->name){
			return i;
		}
	}
//	assert(false && printf("Cannot find muscle name %s" , muscleName.c_str()));
	return 0;
}

void EditorWindow::readSimpleMotion() {
	readJointMap();
	FILE *in = fopen((string(MSS_ROOT_DIR) + "/simpleMotion/SimpleMotionSet.txt").c_str(), "r");
	char line[1005];
	vector<SimpleMotion> _simpleMotions;
	while (fgets(line, 100, in) != NULL) {
		string str = string(line);
		if (line[0] == 'n') {
			char *p = strtok(line, " \n");
			p = strtok(NULL, " \n");
			_simpleMotions.emplace_back(SimpleMotion());
			_simpleMotions.back().motionName = p;

			if (p=="T_Pose" || p=="Stand_Pose"){
				for (int i=0;i<mWorld->mStdMusculoSkeletalSystem->getNumMuscles();i++)
					_simpleMotions.back().relatedMuscleIndices.emplace_back(i);
			}
		} else if (line[0] == 'c') {
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
		} else if (line[0] == 'm') {
			char *p = strtok(line, " \n");
			p = strtok(NULL, " \n");
			string muscleName = p;

			_simpleMotions.back().relatedMuscleIndices.emplace_back(getMuscleIdx(muscleName));
		} else if (line[0] == 'p') {
			char *p = strtok(line, " \n");
			p = strtok(NULL, " \n");
			string motionName = p;
			for (auto &_sm: _simpleMotions) {
				if (_sm.motionName == motionName) {
					simpleMotions.emplace_back(_sm);
					break;
				}
			}
		}
	}
}

void EditorWindow::findSimpleMotionRelatedMuscleIndices() {
	auto &stdMSS = mWorld->mStdMusculoSkeletalSystem;
	for (SimpleMotion &sm: simpleMotions) {
		cout << "@ " << sm.motionName << endl;
		for (int muscleIdx = 0; muscleIdx < stdMSS->getNumMuscles(); muscleIdx++) {
			double minLen = 10000, maxLen = 0;
			for (int i = 0; i <= 5; i++) {
				double config[DOF];
				configInterpolation(sm.startConfig, sm.endConfig, config,
				                    1.0 * i / 5, DOF);
				setStdPose(config);

				double len = stdMSS->mMuscles[muscleIdx]->getLength();
				minLen = min(minLen, len);
				maxLen = max(maxLen, len);
			}
			if (maxLen - minLen > 1e-3){
				cout << "\t" << stdMSS->mMuscles[muscleIdx]->name << endl;
				sm.relatedMuscleIndices.emplace_back(muscleIdx);
			}
		}
		cout << "------------" << endl;
	}

	for (int i = 0; i < DOF; i++) stdMSS->getSkeleton()->setPosition(i, 0);
}

int EditorWindow::getSimpleMotionIdx(std::string sm_name) {
	for (int i=0;i<simpleMotions.size();i++){
		if (simpleMotions[i].motionName == sm_name) return i;
	}
	return -1;
}

void EditorWindow::updateTargetSkeleton() {

	mWorld->updateTargetSkeleton();

	if (mUseMuscle) {
		retargetMuscles(mWorld->mStdMusculoSkeletalSystem,
		                mWorld->mRtgMusculoSkeletalSystem,
		                simpleMotions);
	}
}


int EditorWindow::selectObject(GLint x, GLint y)
{
	GLuint selectBuff[64];
	GLint hits, viewport[4];

	glSelectBuffer(63, selectBuff);
	glGetIntegerv(GL_VIEWPORT, viewport);
	GLint w = glutGet(GLUT_WINDOW_WIDTH);
	GLint h = glutGet(GLUT_WINDOW_HEIGHT);

	glMatrixMode(GL_PROJECTION);
	glRenderMode(GL_SELECT);
	glLoadIdentity();
	gluPickMatrix(x, viewport[3]-y, 2, 2, viewport);
	gluPerspective(mCamera->fovy, (GLfloat)w / (GLfloat)h, 0.01, 1000);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(mCamera->eye.x(), mCamera->eye.y(), mCamera->eye.z(),
	          mCamera->lookAt.x(),mCamera->lookAt.y(), mCamera->lookAt.z(),
	          mCamera->up.x(), mCamera->up.y(), mCamera->up.z());

	glClearColor(0.95, 0.95, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glInitNames();
	switch(mode){
		case MODE::SKELETON_EDIT:
			GUI::DrawSkeleton(mWorld->GetRigidWorld()->getSkeleton(1), name2ID, Eigen::Vector3d(0.1, 1.0, 0.1), true);
			break;
		case MODE::ROM_EDIT:
			loadROMs();
			break;
	}
//	ROM->LoadROM("R_Femur");
//	ROM->LoadROM("R_Tibia");
//	ROM->LoadROM("R_Foot");
//	ROM->LoadROM("L_Femur");
//	ROM->LoadROM("L_Tibia");
//	ROM->LoadROM("L_Foot");
//
//	ROM->LoadROM("R_Arm");
//	ROM->LoadROM("R_ForeArm");
//	ROM->LoadROM("R_Hand");
//	ROM->LoadROM("L_Arm");
//	ROM->LoadROM("L_ForeArm");
//	ROM->LoadROM("L_Hand");

	hits = glRenderMode(GL_RENDER);

	cout << "HIT! : " << ID2Name[selectBuff[3]] << endl;

	if(hits == 0)
		return -1;
	return selectBuff[3];
}

void
EditorWindow::
Motion(int x, int y)
{
	if (!mIsDrag)
		return;
	if (romSphereDrag)
	{
		if(isBallROM(selectedROMID))
			moveROMSphere(x, y);
		else if(isRevoluteROM(selectedROMID))
			moveRevoluteArc(x,y);
	}
	if(!romSphereDrag)
		SimWindow::Motion(x, y);


}

void EditorWindow::Mouse(int button, int state, int x, int y)
{
	int mod = glutGetModifiers();


	if(mod == GLUT_ACTIVE_SHIFT)
	{


		if (state == GLUT_DOWN)
		{
			if (mode == MODE::SKELETON_EDIT) {
				// picking
				selectedSkeletonID = selectObject(x, y);
				if (selectedSkeletonID == -1) { // 빈 공간 shift + 클릭 시
					if (isEditingSkeleton) { // 기존의 editing 종료
						isEditingSkeleton = false;
					}
					cout << "EMPTY\n";
				} else { // Visualization 된 ROM 중 하나를 클릭
					if (isEditingSkeleton) { // 이미 다른 ROM 을 editing 중이었다면 무시
						// ignore
					} else if (isEditableSkeleton(selectedSkeletonID)) {
						isEditingSkeleton = true;
						editingSkeletonID = selectedSkeletonID;

					}
				}
			}else if (mode == MODE::ROM_EDIT || mode == MODE::MUSCLE_PARAMETER_ADAPTING){
				// picking
				selectedROMID = selectObject(x, y);
				if(selectedROMID != -1)
				{
					tempSaveCurrent();
					romSphereDrag = true;
					mIsDrag = true;
				}

				// if (selectedROMID == -1) {
				// 	if (isEditingROM) {
				// 		isEditingROM = false;
				// 	}
				// 	cout << "EMPTY\n";
				// }else{
				// 	if (isEditingROM){
				// 		// ignore
				// 	}else if (isEditableROM(selectedROMID)){
				// 		isEditingROM = true;
				// 		editingROM = ROM->getROM(ID2Name[selectedROMID]);
				// 	}
				// }

				if ((button ==3) || (button == 4))
				{
					if (isBallROM(selectedROMID))
					{
						if(button == 3)
							scaleROMSphere(true);
						if(button == 4)
							scaleROMSphere(false);
					}
					else if(isRevoluteROM(selectedROMID))
					{
						if(button == 3)
							scaleRevoluteArc(true);
						if(button == 4)
							scaleRevoluteArc(false);
					}

				}
			}

		}

		else
		{
			romSphereDrag = false;
			mIsDrag = false;
			selectedROMID = -1;
	//			mShiftMouseType = -1;
		}
	}
		// const unsigned char text[10] = "Hello";
		// RenderString(-1.0, 1.0, GLUT_BITMAP_TIMES_ROMAN_24, text, Eigen::Vector3d(1.0f, 0.0f, 0.0f));
	else
	{
		if ((button ==3) || (button == 4))
		{
			if(state == GLUT_UP) return;
			if(button == 3)
				mCamera->Zoom(0,0,-30,-30);
			if(button == 4)
				mCamera->Zoom(0,0,30,30);
			mIsDrag = false;
		}


		if (state == GLUT_DOWN)
		{
			mIsDrag = true;
			mMouseType = button;
			mPrevX = x;
			mPrevY = y;
		}
		else
		{
			mIsDrag = false;
			mMouseType = -1;
			selectedROMID = -1;
	//			mShiftMouseType = -1;
		}
		// const unsigned char text[10] = "Hello";
		// RenderString(-1.0, 1.0, GLUT_BITMAP_TIMES_ROMAN_24, text, Eigen::Vector3d(1.0f, 0.0f, 0.0f));
		romSphereDrag = false;
		glutPostRedisplay();
	}
	prevX = x;
	prevY = y;
}

void EditorWindow::computeROMs() {
	ROM->ComputeROM("R_Femur", nonMuscleOn);
	ROM->ComputeROM("R_Tibia", nonMuscleOn);
	ROM->ComputeROM("R_Foot", nonMuscleOn);
	ROM->ComputeROM("L_Femur", nonMuscleOn);
	ROM->ComputeROM("L_Tibia", nonMuscleOn);
	ROM->ComputeROM("L_Foot", nonMuscleOn);
//
	ROM->ComputeROM("R_Arm", nonMuscleOn);
	ROM->ComputeROM("R_ForeArm", nonMuscleOn);
	ROM->ComputeROM("R_Hand", nonMuscleOn);
	ROM->ComputeROM("L_Arm", nonMuscleOn);
	ROM->ComputeROM("L_ForeArm", nonMuscleOn);
	ROM->ComputeROM("L_Hand", nonMuscleOn);
}

void EditorWindow::loadROMs(){
	ROM->LoadROM("R_Femur");
	ROM->LoadROM("R_Tibia");
	ROM->LoadROM("R_Foot");
	 ROM->LoadROM("L_Femur");
	 ROM->LoadROM("L_Tibia");
	 ROM->LoadROM("L_Foot");
//
	ROM->LoadROM("R_Arm");
	ROM->LoadROM("R_ForeArm");
	ROM->LoadROM("R_Hand");
	 ROM->LoadROM("L_Arm");
	 ROM->LoadROM("L_ForeArm");
	 ROM->LoadROM("L_Hand");
}

bool EditorWindow::isEditableSkeleton(int ID){
	for (int i=0;i<mWorld->mbis.size();i++)
		if (mWorld->mbis[i].name == ID2Name[ID])
			return true;
	return false;
}
bool EditorWindow::isEditableROM(int ID) {
	switch (ID){
		case ID_R_Femur:
		case ID_L_Femur:
		case ID_R_Tibia:
		case ID_L_Tibia:
		case ID_R_Foot:
		case ID_L_Foot:
		case ID_R_Arm:
		case ID_L_Arm:
		case ID_R_ForeArm:
		case ID_L_ForeArm:
		case ID_R_Hand:
		case ID_L_Hand:
			return true;
			break;
		default:
			return false;
			break;
	}
	return false;
}
bool EditorWindow::isBallROM(int ID) {
	switch (ID){
		case ID_R_Femur:
		case ID_L_Femur:
		case ID_R_Foot:
		case ID_L_Foot:
		case ID_R_Arm:
		case ID_L_Arm:
		case ID_R_Hand:
		case ID_L_Hand:
			return true;
			break;
		default:
			return false;
			break;
	}
	return false;
}
bool EditorWindow::isRevoluteROM(int ID) {
	switch (ID){
		case ID_R_Tibia:
		case ID_L_Tibia:
		case ID_R_ForeArm:
		case ID_L_ForeArm:
			return true;
			break;
		default:
			return false;
			break;
	}
	return false;
}

double getBetweenAngle(Eigen::Vector3d v1, Eigen::Vector3d v2)
{
	Eigen::Vector3d v1_norm = v1.normalized();
	Eigen::Vector3d v2_norm = v2.normalized();
	return atan2(v1_norm.cross(v2_norm).norm(), v1_norm.dot(v2_norm));
}


void EditorWindow::moveROMSphere(int x, int y)
{
	//radius of the sphere
	double radius = 0.15;

	Eigen::Vector3d prev_ray_vector = getMouseRayVector(prevX, prevY);
	Eigen::Vector3d cur_ray_vector = getMouseRayVector(x, y);

	BodyNodePtr body = ROM->ms->getSkeleton()->getBodyNode(ID2Name[selectedROMID]);

	Eigen::Vector3d originPos = body->getParentJoint()->getPositions();
	body->getParentJoint()->setPositions(Eigen::Vector3d::Zero());
	Eigen::Isometry3d zeroParentJointT = body->getTransform() * body->getParentJoint()->getTransformFromChildBodyNode();
	body->getParentJoint()->setPositions(originPos);

	//Technicaly, normal is not correct. we approximate to prey_ray_vector;
	Eigen::Vector3d center = zeroParentJointT.translation();
	Eigen::Vector3d normal = prev_ray_vector;

	Eigen::Vector3d prev_ray_point_on_plane = pointOnPlane(prev_ray_vector, center, normal);
	Eigen::Vector3d cur_ray_point_on_plane = pointOnPlane(cur_ray_vector, center, normal);

	if((prev_ray_point_on_plane - center).norm()>radius)
		prev_ray_point_on_plane = center + (prev_ray_point_on_plane - center).normalized() * radius;
	if((cur_ray_point_on_plane - center).norm()>radius)
		cur_ray_point_on_plane = center + (cur_ray_point_on_plane - center).normalized() * radius;

	double prev_sphere_depth = radius * sqrt(1 - (prev_ray_point_on_plane-center).norm()*(prev_ray_point_on_plane-center).norm());
	double cur_sphere_depth = radius * sqrt(1 - (cur_ray_point_on_plane-center).norm()*(cur_ray_point_on_plane-center).norm());

	Eigen::Vector3d prev_ray_point_on_sphere = prev_ray_point_on_plane - prev_sphere_depth * prev_ray_vector;
	Eigen::Vector3d cur_ray_point_on_sphere = cur_ray_point_on_plane - cur_sphere_depth * cur_ray_vector;

	Eigen::Vector3d prev_vector = (prev_ray_point_on_sphere - center).normalized();
	Eigen::Vector3d cur_vector = (cur_ray_point_on_sphere - center).normalized();


	double between_angle = getBetweenAngle(prev_vector, cur_vector);
	Eigen::Vector3d rot_axis = prev_vector.cross(cur_vector);
	if(rot_axis.norm()<1e-6)
	{
		between_angle = 0;
		rot_axis = Eigen::Vector3d::UnitX();
	}
	else
		rot_axis.normalize();

	Eigen::Matrix3d userRotation = Eigen::AngleAxisd(between_angle, rot_axis).toRotationMatrix();

	BodyROM* bodyROM = ROM->getROM(ID2Name[selectedROMID]);

	Eigen::Vector3d world_cur_centerJ2B = zeroParentJointT.linear()* bodyROM->cur_centerJ2B;

	bodyROM->cur_centerJ2B = zeroParentJointT.linear().inverse() * userRotation * zeroParentJointT.linear()*prev_cur_centerJ2B;
}

void EditorWindow::moveRevoluteArc(int x, int y)
{
	//radius of the sphere
	double radius = 0.15;

	Eigen::Vector3d prev_ray_vector = getMouseRayVector(prevX, prevY);
	Eigen::Vector3d cur_ray_vector = getMouseRayVector(x, y);

	BodyNodePtr body = ROM->ms->getSkeleton()->getBodyNode(ID2Name[selectedROMID]);

	double originPos = body->getParentJoint()->getPositions()[0];
	body->getParentJoint()->setPosition(0, 0);
	Eigen::Isometry3d zeroParentJointT = body->getTransform() * body->getParentJoint()->getTransformFromChildBodyNode();
	body->getParentJoint()->setPosition(0, originPos);

	//Technicaly, normal is not correct. we approximate to prey_ray_vector;
	Eigen::Vector3d center = zeroParentJointT.translation();

	Eigen::Vector3d normal;

	RevoluteJoint* revJoint = dynamic_cast<RevoluteJoint*>(body->getParentJoint());
	normal = revJoint->getAxis();
	normal = zeroParentJointT.linear() * normal;

	Eigen::Vector3d prev_ray_point_on_plane = pointOnPlane(prev_ray_vector, center, normal);
	Eigen::Vector3d cur_ray_point_on_plane = pointOnPlane(cur_ray_vector, center, normal);

	Eigen::Vector3d center_to_prev = (prev_ray_point_on_plane-center).normalized();
	Eigen::Vector3d center_to_cur = (cur_ray_point_on_plane - center).normalized();

	Eigen::Vector3d axis = center_to_prev.cross(center_to_cur);
	double angle = atan2(axis.norm(), center_to_prev.dot(center_to_cur));
	if(axis.dot(normal) <0)
		angle *= -1;
	
	BodyROM* bodyROM = ROM->getROM(ID2Name[selectedROMID]);

	bodyROM->revoluteEditInfo.translate = prev_translateRev + angle;
	// cout<<cur_ray_point_on_plane.transpose()<<endl;


}


Eigen::Vector3d EditorWindow::getMouseRayVector(int x, int y)
{
	float width = glutGet(GLUT_WINDOW_WIDTH);
	float height = glutGet(GLUT_WINDOW_HEIGHT);
	/// normed point -> point on the screen x : -1.0 ~ 1.0, y : -1.0 ~ 1.0
	Eigen::Vector2d normed_point;
	/// real_diff -> point on the picked plane
	Eigen::Vector2d real_normed_point;

	normed_point[0] = (width / 2.0f - x) / (width / 2.0);
	normed_point[1] = (height / 2.0f - y) / (height / 2.0);
	double viewDistance = (mCamera->lookAt - mCamera->eye).norm();
	real_normed_point[1] = normed_point[1] * viewDistance * tan(mCamera->fovy * M_PI / 180.0 / 2.0);
	real_normed_point[0] = normed_point[0] * viewDistance * tan(mCamera->fovy * M_PI / 180.0 / 2.0) * width/height;

	// cout<<viewUp.transpose()<<endl;

	Eigen::Vector3d viewLeft;
	viewLeft = (mCamera->up).normalized().cross((mCamera->lookAt - mCamera->eye).normalized());
	// The norm of viewRight is already 1. For safe implement, we will normailze.
	viewLeft.normalize();
	Eigen::Vector3d real_point_on_lookAt_normed_plane;
	real_point_on_lookAt_normed_plane = mCamera->lookAt + real_normed_point[0] * viewLeft + real_normed_point[1] * mCamera->up;
	return (real_point_on_lookAt_normed_plane - mCamera->eye).normalized();
}


Eigen::Vector3d EditorWindow::pointOnPlane(Eigen::Vector3d ray, Eigen::Vector3d center, Eigen::Vector3d normal)
{
	// 2D plane on 3d : ax + by +cz +d = 0 => normal.dot(center)+d = 0
	double d = - normal.dot(center);
	double t = -(d + normal.dot(mCamera->eye))/(normal.dot(ray));

	return  mCamera->eye+ray*t;
}

void EditorWindow::tempSaveCurrent()
{
	BodyROM* bodyROM = ROM->getROM(ID2Name[selectedROMID]);
	prev_cur_centerJ2B = bodyROM->cur_centerJ2B;
	prev_translateRev = bodyROM->revoluteEditInfo.translate;
}
void EditorWindow::scaleROMSphere(bool up)
{
	BodyROM* bodyROM = ROM->getROM(ID2Name[selectedROMID]);
	if(up)
		bodyROM->conicScale += 0.1;
	else
		bodyROM->conicScale -= 0.1;
}

void EditorWindow::scaleRevoluteArc(bool up)
{
	BodyROM* bodyROM = ROM->getROM(ID2Name[selectedROMID]);
	if(up)
		bodyROM->revoluteEditInfo.scale += 0.1;
	else
		bodyROM->revoluteEditInfo.scale -= 0.1;
}

void EditorWindow::convertUserInputIntoEDB()
{
	for(auto& bodyROM : ROM->BodyROMs)
	{
		if(IsBallJoint(bodyROM->name))
		{
			// cout<<bodyROM->conicScale<<endl;
			int body_index_edb = ROM->edb->findBodyIndex(bodyROM->name);
			ROM->edb->setEditInfo(bodyROM->name, 
				bodyROM->cur_centerJ2B, bodyROM->conicScale, 0.0, 1.0);
		}
		else
		{
			int body_index_edb = ROM->edb->findBodyIndex(bodyROM->name);
			ROM->edb->setEditInfo(bodyROM->name, 
				bodyROM->revoluteEditInfo.prev_center-bodyROM->revoluteEditInfo.center+ bodyROM->revoluteEditInfo.translate, 
				bodyROM->revoluteEditInfo.scale);
		}
	}
}