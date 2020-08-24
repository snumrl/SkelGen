//
// Created by minseok on 11/22/18.
//
#include "APTWindow.h"
#include "dart/dart.hpp"
#include "../model/DART_helper.h"
#include "../generator/motionAdjust/BVHparser.h"
#include "../romEditor/ROMEditor.h"
using namespace GUI;
using namespace dart;
using namespace dart::simulation;
using namespace dart::dynamics;
using namespace std;

std::string curSimpleMotion = "right_hip_flexion";
std::string curDof = "R_Femur";

APTWindow::APTWindow():SimWindow(), 
smStep(30), ratioCnt(0), bvhPositionOffset(Eigen::Vector3d(1.0, 0.0, 0.0)){
	InitSkelRertargeting();
	// InitAPTRetargeting();
}

void APTWindow::InitSkelRertargeting()
{
	rtgSkeletonInfo = SkeletonInfo();
	//	FemurLength = 44.53391; // (cm), 0.4453391 cm in zygote
	//  TibiaLength = 42.49466; // (cm), 0.4249466 cm in zygote
	rtgSkeletonInfo.R_Femur.alpha_lengthening = 2.0;
	rtgSkeletonInfo.L_Femur.alpha_lengthening = 1.0;
	rtgSkeletonInfo.R_Tibia.alpha_lengthening = 1.2;
	rtgSkeletonInfo.L_Tibia.alpha_lengthening = 1.0;
	smIdx = stepIdx = 0;

	mWorld->Initialize();

	readSimpleMotion();

	// retargetTotalMusclesWaypointsCalibrating(mWorld->mStdMusculoSkeletalSystem,
 //                                    mWorld->mRtgMusculoSkeletalSystem,
 //                                    simpleMotions);
	if (true) {
		retargetMuscles(mWorld->mStdMusculoSkeletalSystem,
		                mWorld->mRtgMusculoSkeletalSystem,
		                simpleMotions);
	}


	// Made standard, initial rtg muscle parameters.
	mWorld->mStdMusculoSkeletalSystem->updateMuscleParametersByMaxLmtFast();
	mWorld->mRtgMusculoSkeletalSystem->updateMuscleParametersByMaxLmtFast();

}

void APTWindow::InitAPTRetargeting()
{
	aptOptimizer = new APTOptimizer(mWorld->GetMusculoSkeletalSystem(), mWorld->GetCmpMusculoSkeletalSystem());
	aptOptimizer->retargetAPT();
}


void APTWindow::Keyboard(unsigned char key, int x, int y) {
	switch (key) {
		case ']':
			ratioCnt = min(20, ratioCnt+1);
			break;
		case '[':
			ratioCnt = max(0, ratioCnt-1);
			break;
		default:
			SimWindow::Keyboard(key, x, y);
			break;
	}
}
/*
n right_knee_flexion_with_0_hip_angle
c R_Knee_Flex 0 90
*/

void APTWindow::drawAPTGraph(std::shared_ptr<MusculoSkeletalSystem> ms, std::string label)
{
	std::string filePath = "KneeAngleTorque";
    filePath = filePath + label+ ".txt";
    FILE *out=fopen(filePath.c_str(),"w");
    std::cout<<"# writing file "<<filePath<<std::endl;
    fprintf(out, "Angle\\Torque\n");

	double config[DOF];

	SkeletonPtr skel = ms->getSkeleton();
	Eigen::VectorXd activationLevels(ms->getNumMuscles());
	activationLevels.setOnes();

	//set to max activation.
	ms->setActivationLevels(activationLevels);
	int index = getSimpleMotionIdx(curSimpleMotion);
	// for(auto& muscle : muscles)
	// {
	// 	cout<<muscle->name<<endl;
	// }

	// cout<<"@##################"<<endl;
	// for(auto& muscle : mWorld->GetMusculoSkeletalSystem()->getMuscles())
	// {
	// 	if(muscle->name == "R_Bicep_Femoris_Short")
	// 	{
	// 		for(int i=0;i<muscle->blendedWaypoints.size();i++)
	// 		{
	// 			for(int j=0;j< muscle->blendedWaypoints[i].relatedBodyNodes.size();j++)
	// 			{
	// 				cout<<muscle->blendedWaypoints[i].relatedBodyNodes[j]->getName()<<" "<<muscle->blendedWaypoints[i].weights[j]<<endl;
	// 			}
	// 			cout<<endl;

	// 		}
	// 	}
	// }
	// exit(0);
	// for(auto& muscle : mWorld->GetMusculoSkeletalSystem()->getMuscles())
	// {
	// 	if(muscle->name == "R_Gastrocnemius_Medial_Head")
	// 	{
	// 		cout<<muscle->name<<
	// 		" "<<muscleJacobianTranspose.block(R_Tibia_index, j*3, 
	// 		1, 3)<<endl;
	// 	}
	// }

	// int R_Tibia_index = skel->getIndexOf(skel->getBodyNode(curDof)->getParentJoint()->getDof(0));
	int R_Tibia_index = skel->getIndexOf(skel->getBodyNode("Spine")->getParentJoint()->getDof(0));
	for(int i=0;i<1;i++)
	{
		cout<<i/100.0<<endl;
		configInterpolation(simpleMotions[index].startConfig, simpleMotions[index].endConfig, config, i/100.0, DOF);
		Eigen::VectorXd pos(DOF);
		for(int j=0;j<DOF;j++)
		{
			pos[j] = config[j] * M_PI / 180.0;
			pos[j] = 0.0;
		}

		skel->setPositions(pos);
		auto muscleForceActive =              ms->computeForceActive(mWorld->GetRigidWorld());
		auto muscleForcePassive =              ms->computeForcePassive(mWorld->GetRigidWorld());
        auto muscleJacobianTranspose =  ms->getJacobianTranspose();
        // auto torque =                   muscleJacobianTranspose * muscleForce;

        double R_Tibia_torque = 0.0;
       	// Eigen::VectorXd R_Tibia_torque = muscleJacobianTranspose.block(R_Tibia_index, 0, 
        // 		1, mWorld->GetMusculoSkeletalSystem()->getNumMuscleForces() * 3) * muscleForce;


        bool applyPassive= true;
        bool activeForcePositive = false;

        vector<int> relativeMuscleIndex;
       	for(int index=0, j=0;j<ms->getNumMuscles();j++)
       	{
  			int muscleNumForces = ms->getMuscles()[j]->getNumForces();
  			if(activeForcePositive)
  			{
  				if((muscleJacobianTranspose.block(R_Tibia_index, index*3, 
        			1, muscleNumForces*3) * muscleForceActive.segment(index*3,muscleNumForces*3))[0]>1e-2)
	       		{
	       			R_Tibia_torque += (muscleJacobianTranspose.block(R_Tibia_index, index*3, 
	        		1, muscleNumForces*3) * muscleForceActive.segment(index*3,muscleNumForces*3))[0];

	        		cout<<"active  : "<<ms->getMuscles()[j]->name<<
	        		" "<<(muscleJacobianTranspose.block(R_Tibia_index, index*3, 
	        		1, muscleNumForces*3) * muscleForceActive.segment(index*3,muscleNumForces*3))[0]<<endl;
	        		relativeMuscleIndex.push_back(j);
	   			}
  			}
  			else
  			{
  				if((muscleJacobianTranspose.block(R_Tibia_index, index*3, 
        			1, muscleNumForces*3) * muscleForceActive.segment(index*3,muscleNumForces*3))[0]<-1e-2)
	       		{
	       			R_Tibia_torque += (muscleJacobianTranspose.block(R_Tibia_index, index*3, 
	        		1, muscleNumForces*3) * muscleForceActive.segment(index*3,muscleNumForces*3))[0];

	        		cout<<"active  : "<<ms->getMuscles()[j]->name<<
	        		" "<<(muscleJacobianTranspose.block(R_Tibia_index, index*3, 
	        		1, muscleNumForces*3) * muscleForceActive.segment(index*3,muscleNumForces*3))[0]<<endl;
	        		relativeMuscleIndex.push_back(j);
	   			}
  			}
       		

   			if(applyPassive && abs((muscleJacobianTranspose.block(R_Tibia_index, index*3, 
        		1, muscleNumForces*3) * muscleForcePassive.segment(index*3,muscleNumForces*3))[0])>1e-2)
       		{
       			R_Tibia_torque += (muscleJacobianTranspose.block(R_Tibia_index, index*3, 
        		1, muscleNumForces*3) * muscleForcePassive.segment(index*3,muscleNumForces*3))[0];

        		cout<<"Passive : "<<ms->getMuscles()[j]->name<<
        		" "<<(muscleJacobianTranspose.block(R_Tibia_index, index*3, 
        		1, muscleNumForces*3) * muscleForcePassive.segment(index*3,muscleNumForces*3))[0]<<endl;
        		
        		// if(relativeMuscleIndex[relativeMuscleIndex.size()-1]!=j)
        		// {
        		// 	relativeMuscleIndex.push_back(j);
        		// }
   			}

   			// if(mWorld->GetMusculoSkeletalSystem()->getMuscles()[j]->name == "R_Bicep_Femoris_Short")
   			// {
   			// 	for(int k=0;k<mWorld->GetMusculoSkeletalSystem()->getMuscles()[j]->getNumForces();k++)
   			// 	{
		    // 		cout<<mWorld->GetMusculoSkeletalSystem()->getMuscles()[j]->name<<
		    // 		" "<<muscleForce.segment((index+k)*3,3).transpose()<<"/"
		    // 		<<muscleJacobianTranspose.block(R_Tibia_index, (index+k)*3, 
		    // 		1, 3)<<endl;
		    // 		cout<<"l_m_tilda : "<<mWorld->GetMusculoSkeletalSystem()->getMuscles()[j]->l_m_tilda<<endl;
   			// 	}
   			// 	cout<<endl;
   			// }

       		index += muscleNumForces;
       		
       	}
       	// cout<<R_Tibia_torque<<" ";
        // R_Tibia_torque += (muscleJacobianTranspose * muscleForcePassive)[R_Tibia_index];
        // cout<<R_Tibia_torque<<endl;
       	cout<<"total : "<<R_Tibia_torque<<" at "<<config[R_Tibia_index]<<endl;
       	cout<<"-------------------------------------------"<<endl;
        // cout<<torque.transpose()<<endl;
        // cout<<torque[R_Tibia_index]<<endl;
        // cout<<R_Tibia_torque<<endl;
        // cout<<endl;
        fprintf(out,"%lf %lf\n",config[R_Tibia_index], R_Tibia_torque);
	}
	fclose(out);
}


void
APTWindow::
checkTorque(std::shared_ptr<MusculoSkeletalSystem> ms)
{

	SkeletonPtr skel = ms->getSkeleton();
	Eigen::VectorXd activationLevels(ms->getNumMuscles());
	activationLevels.setOnes();
	//set to max activation.
	ms->setActivationLevels(activationLevels);


	int R_Tibia_index = skel->getIndexOf(skel->getBodyNode("Neck")->getParentJoint()->getDof(0));
	Eigen::VectorXd pos(DOF);
	pos.setZero();
	skel->setPositions(pos);
	auto muscleForceActive =              ms->computeForceActive(mWorld->GetRigidWorld());
	auto muscleForcePassive =              ms->computeForcePassive(mWorld->GetRigidWorld());
    auto muscleJacobianTranspose =  ms->getJacobianTranspose();

    double R_Tibia_torque_positive = 0.0;
    double R_Tibia_torque_negative = 0.0;


    bool applyPassive= true;
    bool activeForcePositive = true;

    vector<int> relativeMuscleIndex;
   	cout<<"------------------------------------"<<endl;
    cout<<"Positive Forces : "<<endl;
   	for(int index=0, j=0;j<ms->getNumMuscles();j++)
   	{
		int muscleNumForces = ms->getMuscles()[j]->getNumForces();
		if(activeForcePositive)
		{
			if((muscleJacobianTranspose.block(R_Tibia_index, index*3, 
			1, muscleNumForces*3) * muscleForceActive.segment(index*3,muscleNumForces*3))[0]>1e-2)
   		{
   			R_Tibia_torque_positive += (muscleJacobianTranspose.block(R_Tibia_index, index*3, 
    		1, muscleNumForces*3) * muscleForceActive.segment(index*3,muscleNumForces*3))[0];

    		cout<<"active  : "<<ms->getMuscles()[j]->name<<
    		" "<<(muscleJacobianTranspose.block(R_Tibia_index, index*3, 
    		1, muscleNumForces*3) * muscleForceActive.segment(index*3,muscleNumForces*3))[0]<<endl;
    		relativeMuscleIndex.push_back(j);
			}
		}
		else
		{
			if((muscleJacobianTranspose.block(R_Tibia_index, index*3, 
			1, muscleNumForces*3) * muscleForceActive.segment(index*3,muscleNumForces*3))[0]<-1e-2)
   		{
   			R_Tibia_torque_positive += (muscleJacobianTranspose.block(R_Tibia_index, index*3, 
    		1, muscleNumForces*3) * muscleForceActive.segment(index*3,muscleNumForces*3))[0];

    		cout<<"active  : "<<ms->getMuscles()[j]->name<<
    		" "<<(muscleJacobianTranspose.block(R_Tibia_index, index*3, 
    		1, muscleNumForces*3) * muscleForceActive.segment(index*3,muscleNumForces*3))[0]<<endl;
    		relativeMuscleIndex.push_back(j);
			}
		}
		if(applyPassive && abs((muscleJacobianTranspose.block(R_Tibia_index, index*3, 
		1, muscleNumForces*3) * muscleForcePassive.segment(index*3,muscleNumForces*3))[0])>1e-2)
   		{
   			R_Tibia_torque_positive += (muscleJacobianTranspose.block(R_Tibia_index, index*3, 
    		1, muscleNumForces*3) * muscleForcePassive.segment(index*3,muscleNumForces*3))[0];

    		cout<<"Passive : "<<ms->getMuscles()[j]->name<<
    		" "<<(muscleJacobianTranspose.block(R_Tibia_index, index*3, 
    		1, muscleNumForces*3) * muscleForcePassive.segment(index*3,muscleNumForces*3))[0]<<endl;

			}
   		index += muscleNumForces;
   	}

   	cout<<"------------------------------------"<<endl;
    cout<<"Negative Forces : "<<endl;
   	activeForcePositive = false;
   	for(int index=0, j=0;j<ms->getNumMuscles();j++)
   	{
		int muscleNumForces = ms->getMuscles()[j]->getNumForces();
		if(activeForcePositive)
		{
			if((muscleJacobianTranspose.block(R_Tibia_index, index*3, 
			1, muscleNumForces*3) * muscleForceActive.segment(index*3,muscleNumForces*3))[0]>1e-2)
   		{
   			R_Tibia_torque_negative += (muscleJacobianTranspose.block(R_Tibia_index, index*3, 
    		1, muscleNumForces*3) * muscleForceActive.segment(index*3,muscleNumForces*3))[0];

    		cout<<"active  : "<<ms->getMuscles()[j]->name<<
    		" "<<(muscleJacobianTranspose.block(R_Tibia_index, index*3, 
    		1, muscleNumForces*3) * muscleForceActive.segment(index*3,muscleNumForces*3))[0]<<endl;
    		relativeMuscleIndex.push_back(j);
			}
		}
		else
		{
			if((muscleJacobianTranspose.block(R_Tibia_index, index*3, 
			1, muscleNumForces*3) * muscleForceActive.segment(index*3,muscleNumForces*3))[0]<-1e-2)
   		{
   			R_Tibia_torque_negative += (muscleJacobianTranspose.block(R_Tibia_index, index*3, 
    		1, muscleNumForces*3) * muscleForceActive.segment(index*3,muscleNumForces*3))[0];

    		cout<<"active  : "<<ms->getMuscles()[j]->name<<
    		" "<<(muscleJacobianTranspose.block(R_Tibia_index, index*3, 
    		1, muscleNumForces*3) * muscleForceActive.segment(index*3,muscleNumForces*3))[0]<<endl;
    		relativeMuscleIndex.push_back(j);
			}
		}
		if(applyPassive && abs((muscleJacobianTranspose.block(R_Tibia_index, index*3, 
		1, muscleNumForces*3) * muscleForcePassive.segment(index*3,muscleNumForces*3))[0])>1e-2)
   		{
   			R_Tibia_torque_negative += (muscleJacobianTranspose.block(R_Tibia_index, index*3, 
    		1, muscleNumForces*3) * muscleForcePassive.segment(index*3,muscleNumForces*3))[0];

    		cout<<"Passive : "<<ms->getMuscles()[j]->name<<
    		" "<<(muscleJacobianTranspose.block(R_Tibia_index, index*3, 
    		1, muscleNumForces*3) * muscleForcePassive.segment(index*3,muscleNumForces*3))[0]<<endl;

			}
   		index += muscleNumForces;
   	}
   	cout<<"total : positive "<<R_Tibia_torque_positive<<" // negative "<<R_Tibia_torque_negative<<endl;
   	cout<<"-------------------------------------------"<<endl;

}

void 
APTWindow::
Timer(int value) {
    SimWindow::Timer(value);
    SkeletonPtr stdSkel = mWorld->GetRigidWorld()->getSkeleton(0);
    SkeletonPtr	rtgSkel = mWorld->GetRigidWorld()->getSkeleton(1);
    Eigen::VectorXd stdSimplePosition(stdSkel->getNumDofs());
    Eigen::VectorXd rtgSimplePosition(stdSkel->getNumDofs());
    double config[DOF+4] = {0,};
	int index = getSimpleMotionIdx(curSimpleMotion);
	configInterpolation(simpleMotions[index].startConfig, simpleMotions[index].endConfig, config, ratioCnt/20.0, DOF);
	// cout<<ratioCnt/20.0<<endl;
	// cout<<DOF<<" "<<skel->getNumDofs();
    for(int i =0;i<stdSkel->getNumDofs();i++)
    {
    	stdSimplePosition[i] = config[i] * M_PI/180.0;
    	rtgSimplePosition[i] = config[i] * M_PI/180.0;
    }

    rtgSimplePosition.segment(3,3) += bvhPositionOffset;

    stdSkel->setPositions(stdSimplePosition);
    rtgSkel->setPositions(rtgSimplePosition);
}

void
APTWindow::
Display() 
{
	glClearColor(0.95, 0.95, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable( GL_BLEND );
	initLights();
	mCamera->Apply();
	glDisable(GL_LIGHTING);
	glLineWidth(2.0);
	DrawLine(Eigen::Vector3d(0,0,0),Eigen::Vector3d(100,0,0),Eigen::Vector3d(1,0,0));
	DrawLine(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,100,0),Eigen::Vector3d(0,1,0));
	DrawLine(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,100),Eigen::Vector3d(0,0,1));
	glLineWidth(1.0);
	glColor3f(0,0,0);
	glLineWidth(3.0);

    DrawSkeleton(mWorld->GetRigidWorld()->getSkeleton(0), Eigen::Vector3d(0.3, 0.3, 0.3), !mRenderDetail);
    DrawSkeleton(mWorld->GetRigidWorld()->getSkeleton(1), Eigen::Vector3d(0.3, 0.3, 0.3), !mRenderDetail);

    // Draw muscles
	if(true)
	{
		glDisable(GL_LIGHTING);

		for(auto& mus: mWorld->GetMusculoSkeletalSystem()->getMuscles()){
			if(mus->name == "R_Gluteus_Maximus" ||
				mus->name == "R_Gluteus_Maximus4" ||
				// mus->name == "R_Semimembranosus" ||
				// mus->name == "R_Adductor_Magnus1"||
				mus->name == "R_Bicep_Femoris_Longus")
			{
				// cout<<mus->name<<" "<<endl;
				// for(int i =0;i<mus->blendedWaypoints[3].relatedBodyNodes.size();i++)
				// {
				// 	cout<<mus->blendedWaypoints[3].relatedBodyNodes[i]->getName()<<" "<<mus->blendedWaypoints[3].weights[i]<<endl;
				// }
				DrawMuscleWayPoints(mus, qobj);
			}
		}
		for(auto& mus: mWorld->GetCmpMusculoSkeletalSystem()->getMuscles()){
			if(mus->name == "R_Bicep_Femoris_Longus" ||
				mus->name == "R_Semimembranosus" ||
				mus->name == "R_Gastrocnemius_Medial_Head")
			{
				// cout<<mus->name<<" "<<endl;
				// for(int i =0;i<mus->blendedWaypoints[3].relatedBodyNodes.size();i++)
				// {
				// 	cout<<mus->blendedWaypoints[3].relatedBodyNodes[i]->getName()<<" "<<mus->blendedWaypoints[3].weights[i]<<endl;
				// }
				DrawMuscleWayPoints(mus, qobj);
			}
		}
		

		// for(auto& mus: mWorld->GetCmpMusculoSkeletalSystem()->getMuscles()){
		// 	DrawMuscleWayPoints(mus, qobj);
		// }

	}
	glutSwapBuffers();
}

void APTWindow::readJointMap() {
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



int APTWindow::getMuscleIdx(string muscle_name){
	auto& stdMSS = mWorld->GetMusculoSkeletalSystem();
	for (int i=0;i< stdMSS->getNumMuscles();i++){
		if (muscle_name == stdMSS->getMuscles()[i]->name){
			return i;
		}
	}
//	assert(false && printf("Cannot find muscle name %s" , muscle_name.c_str()));
	return 0;
}

void APTWindow::readSimpleMotion() {
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


int APTWindow::getSimpleMotionIdx(std::string sm_name) {
	for (int i=0;i<simpleMotions.size();i++){
		if (simpleMotions[i].motionName == sm_name) return i;
	}
	return -1;
}

