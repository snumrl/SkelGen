#include "SimWindow.h"
#include "dart/external/lodepng/lodepng.h"
#include "SkeletonBuilder.h"
#include <algorithm>
#include <fstream>
#include <boost/filesystem.hpp>
#include <GL/glut.h>
using namespace GUI;
using namespace dart::simulation;
using namespace dart::dynamics;

SimWindow::
SimWindow()
	:GLUTWindow(),mIsRotate(false),mIsAuto(false),mIsCapture(false),mIsFocusing(false),mIsNNLoaded(false),mIsMuscleNNLoaded(false)
	,mIsWriteOutput(false),mOutputCount(0),save_index(1), mShowMotion(true)
{
	// program = CreateShader("../shaders/phong.vert","../shaders/phong.frag");
	// depth_program = CreateShader("../shaders/depth.vert","../shaders/depth.frag");
	mWorld = new MSS::Environment(30,600);
	mAction =Eigen::VectorXd::Zero(mWorld->GetNumAction());
	mDisplayTimeout = 33;

	mm = p::import("__main__");
	mns = mm.attr("__dict__");
	sys_module = p::import("sys");
	
	p::str module_dir = (std::string(MSS_ROOT_DIR)+"/pymss").c_str();
	sys_module.attr("path").attr("insert")(1, module_dir);
	p::exec("import torch",mns);
	p::exec("import torch.nn as nn",mns);
	p::exec("import torch.optim as optim",mns);
	p::exec("import torch.nn.functional as F",mns);
	p::exec("import torchvision.transforms as T",mns);
	p::exec("import numpy as np",mns);
	p::exec("from Model import *",mns);

}
SimWindow::
SimWindow(const std::string& nn_path)
	:SimWindow()
{
	mIsNNLoaded = true;

	boost::python::str str = ("num_state = "+std::to_string(mWorld->GetNumState())).c_str();
	p::exec(str,mns);
	str = ("num_action = "+std::to_string(mWorld->GetNumAction())).c_str();
	p::exec(str,mns);

	nn_module = p::eval("SimulationNN(num_state,num_action)",mns);

	p::object load = nn_module.attr("load");
	load(nn_path);

}
SimWindow::
SimWindow(const std::string& nn_path,const std::string& muscle_nn_path)
	:SimWindow(nn_path)
{
	mIsMuscleNNLoaded = true;

	boost::python::str str = ("num_total_muscle_related_dofs = "+std::to_string(mWorld->GetNumTotalRelatedDofs())).c_str();
	p::exec(str,mns);
	str = ("num_dofs = "+std::to_string(mWorld->GetCharacter()->GetSkeleton()->getNumDofs())).c_str();
	p::exec(str,mns);
	str = ("num_muscles = "+std::to_string(mWorld->GetCharacter()->GetMuscles().size())).c_str();
	p::exec(str,mns);

	muscle_nn_module = p::eval("MuscleNN(num_total_muscle_related_dofs,num_dofs-6,num_muscles).cuda()",mns);

	p::object load = muscle_nn_module.attr("load");
	load(muscle_nn_path);
}
void
SimWindow::
Display() 
{
	glClearColor(1.0, 1.0, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	initLights();

	auto character = mWorld->GetCharacter();
	auto ground = mWorld->GetGround();
	
	Eigen::Vector3d v;
	if(mIsFocusing)
	{
		{
		Eigen::VectorXd p = character->GetSkeleton()->getPositions();
		Eigen::VectorXd action = mWorld->GetAction();
		action.setZero();
		auto target = mWorld->GetTargetPositionAndVelocities(action);
		// character->GetSkeleton()->setPositions(target.first);
		// character->GetSkeleton()->computeForwardKinematics(true,false,false);
		Eigen::Isometry3d T = character->GetSkeleton()->getRootBodyNode()->getTransform();
		if(!mShowMotion)
			v = T.translation();
		else
			v = target.first.segment(3,3);
		v[1] = 0.0;
		mCamera->SetLookAt(v);
		mCamera->eye = mCamera->lookAt + Eigen::Vector3d(-0.0, 0.0, 3.0);
		character->GetSkeleton()->setPositions(p);
		}
		
		// std::cout<<v.transpose()<<std::endl;
	}
	mCamera->Apply();
	
	glDisable(GL_LIGHTING);
	glColor3f(0.8,0.8,0.8);
	
	float y = 
		ground->getBodyNode(0)->getTransform().translation()[1] +
		dynamic_cast<const BoxShape*>(ground->getBodyNode(0)->getShapeNodesWith<dart::dynamics::VisualAspect>()[0]->getShape().get())->getSize()[1]*0.5;
	GUI::DrawGround(y);

	glPopMatrix();
	
	
	{
		Eigen::VectorXd action = mWorld->GetAction();
		action.setZero();
		
		auto target = mWorld->GetTargetPositionAndVelocities(action);
		SkeletonPtr skel = character->GetSkeleton();
		Eigen::VectorXd zeroPosition(skel->getNumDofs());

		zeroPosition.setZero();

		zeroPosition.segment(3,3) = target.first.segment(3,3);

		int R_Arm_index = skel->getIndexOf(skel->getJoint("ArmR")->getDof(2));
		int L_Arm_index = skel->getIndexOf(skel->getJoint("ArmL")->getDof(2));

		zeroPosition[R_Arm_index] = M_PI/2.0;
		zeroPosition[L_Arm_index] = -M_PI/2.0;


		skel->setPositions(zeroPosition);

		if(!mShowMotion)
			GUI::DrawSkeleton(character->GetSkeleton());

		// std::cout<<"Avg : "<<mWorld->GetAverageActivationLevels().transpose()<<std::endl;
		mWorld->SetActivationLevels(mWorld->GetAverageActivationLevels());
		// if(!mShowMotion)
		// 	GUI::DrawMuscles(character->GetMuscles());
	}

	// {
	// 	Eigen::VectorXd p = character->GetSkeleton()->getPositions();
	// 	Eigen::VectorXd action = mWorld->GetAction();
	// 	auto target = mWorld->GetTargetPositionAndVelocities(action);
	// 	target.first[3] += 2.0;
	// 	character->GetSkeleton()->setPositions(target.first);
	// 	character->GetSkeleton()->computeForwardKinematics(true,false,false);
	// 	GUI::DrawSkeleton(character->GetSkeleton(),Eigen::Vector3d(1.0,0.6,0.6));
	// 	character->GetSkeleton()->setPositions(p);
	// }
	{
		Eigen::VectorXd p = character->GetSkeleton()->getPositions();
		Eigen::VectorXd action = mWorld->GetAction();
		action.setZero();
		
		auto target = mWorld->GetTargetPositionAndVelocities(action);
		target.first[3] += 0.0;
		character->GetSkeleton()->setPositions(target.first);
		character->GetSkeleton()->computeForwardKinematics(true,false,false);
		if(mShowMotion)
			GUI::DrawSkeleton(character->GetSkeleton(),Eigen::Vector3d(0.6,1.0,0.6), true);
		character->GetSkeleton()->setPositions(p);
	}
	// for(int i =0;i<save_positions.size();i++)
	// {
	// 	if(i%save_index==0)
	// 	{
	// 		Eigen::VectorXd p = character->GetSkeleton()->getPositions();

	// 		character->GetSkeleton()->setPositions(save_positions[i]);
	// 		mWorld->SetActivationLevels(save_activations[i]);
	// 		character->GetSkeleton()->computeForwardKinematics(true,false,false);
	// 		GUI::DrawSkeleton(character->GetSkeleton());
	// 		// GUI::DrawMuscles(character->GetMuscles());		
	// 		character->GetSkeleton()->setPositions(p);
	// 	}
		
	// }
	glutSwapBuffers();
	if(mIsCapture){
		if(mIsAuto == false)
			mIsAuto=true;
		Screenshot();
		
	}
	glutPostRedisplay();
}

void
SimWindow::
Keyboard(unsigned char key,int x,int y) 
{
	auto muscle = mWorld->GetCharacter()->GetMuscles()[0];
	// ucs.at("R_Psoas_Major1").l_max = 0.96;
	switch(key)
	{
		case '`': mIsRotate= !mIsRotate;break;
		case 's': Step();break;
		case 'R': mWorld->Reset(false);break;
		case 'r': mWorld->Reset(true);break;
		case 'f': mIsFocusing=!mIsFocusing;break;
		case 'C': mIsCapture = true; break;
		case 'W': mIsWriteOutput = !mIsWriteOutput; break;
		case ' ': mIsAuto = !mIsAuto;break;
		case 't': mWorld->GetReward();break;
		case '5': save_index++;save_index%=save_positions.size();break;
		case 'T': Teaser();break;
		case 'h': mShowMotion = !mShowMotion;break;

		case 27 : exit(0);break;
		default : break;
	}
	glutPostRedisplay();
}

void
SimWindow::
Mouse(int button, int state, int x, int y) 
{
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
		mMouseType = 0;
	}

	glutPostRedisplay();
}
void
SimWindow::
Motion(int x, int y) 
{
	if (!mIsDrag)
		return;

	int mod = glutGetModifiers();
	if (mMouseType == GLUT_LEFT_BUTTON)
	{
		
		if(!mIsRotate)
		mCamera->Translate(x,y,mPrevX,mPrevY);
		else
		mCamera->Rotate(x,y,mPrevX,mPrevY);	
	
		
	}
	else if (mMouseType == GLUT_RIGHT_BUTTON)
	{
		switch (mod)
		{
		case GLUT_ACTIVE_SHIFT:
			mCamera->Zoom(x,y,mPrevX,mPrevY); break;
		default:
			mCamera->Pan(x,y,mPrevX,mPrevY); break;		
		}
	}
	mPrevX = x;
	mPrevY = y;
	glutPostRedisplay();
}
void
SimWindow::
Reshape(int w, int h) 
{
	glViewport(0, 0, w, h);
	mCamera->Apply();
}
void
SimWindow::
Step()
{
	// std::cout<<mWorld->GetCharacter()->GetMotionGraph()->GetPhase()<<std::endl;
	if(mIsWriteOutput)
		WriteOutput("output");
	if(mIsNNLoaded)
		GetActionFromNN();
	mWorld->SetAction(mAction);
	// double root_y = mWorld->GetCharacter()->GetSkeleton()->getBodyNode(0)->getTransform().translation()[1];
	// std::cout << root_y << std::endl;
	
	int sim_per_control = mWorld->GetSimulationHz()/mWorld->GetControlHz();
	for(int i =0;i<mWorld->GetCharacter()->GetMuscles().size();i++)
	{
		// std::cout<<mWorld->GetCharacter()->GetMuscles()[i]->name<<" "<<mWorld->GetCharacter()->GetMuscles()[i]->l_mt<<std::endl;
	}
	for(int i = 0;i<sim_per_control;i++)
	{
		Eigen::VectorXd mt = mWorld->GetMuscleTorques();
		mWorld->SetActivationLevels(GetActivationFromNN(mt));
		mWorld->Step();
	}
	mWorld->GetReward();
	std::ofstream os;
	
	// os.open("act.txt",std::ofstream::out|std::ofstream::app);
	// std::vector<std::string> name;
	// name.push_back("L_Soleus");
	// name.push_back("L_Gastrocnemius_Lateral_Head");
	// name.push_back("L_Gastrocnemius_Medial_Head");

// name.push_back("L_Gluteus_Maximus");
// name.push_back("L_Gluteus_Maximus1");
// name.push_back("L_Gluteus_Maximus2");
// name.push_back("L_Gluteus_Maximus3");
// name.push_back("L_Gluteus_Maximus4");
	// name.push_back("L_iliacus");
	// name.push_back("L_Rectus_Femoris");
	// name.push_back("L_Sartorius");
	// name.push_back("L_Pectineus");
	// name.push_back("L_Adductor_Brevis");
	// name.push_back("L_Adductor_Brevis1");
	// name.push_back("L_Adductor_Longus");
	// name.push_back("L_Adductor_Longus1");
	// name.push_back("L_Gracilis");
	
	// for(int j=0;j<mWorld->GetCharacter()->GetMuscles().size();j++)
	// 	std::cout<<mWorld->GetCharacter()->GetMuscles()[j]->name<<std::endl;

	// for(int i =0;i<name.size();i++)
	// {
	// 	for(int j=0;j<mWorld->GetCharacter()->GetMuscles().size();j++)
	// 	{
	// 		if(mWorld->GetCharacter()->GetMuscles()[j]->name==name[i]){
	// 			std::cout<<j<<std::endl;
	// 		}
	// 	}
	// }


	// os<<mWorld->GetCharacter()->GetMotionGraph()->GetPhase()<<" "<<mWorld->GetAverageActivationLevels().transpose()<<std::endl;
	// std::cout<<save_index<<std::endl;
	// if(mWorld->GetCharacter()->GetMotionGraph()->GetPhase()<0.0001)
	// 	save_index++;
	// if(save_index==20)
	// 	exit(0);
	// os.close();
}
void
SimWindow::
Timer(int value) 
{
	if( mIsAuto ) Step();
	
	glutTimerFunc(mDisplayTimeout, TimerEvent,1);
	glutPostRedisplay();
}


void SimWindow::
Screenshot() {
  static int count = 0;
  const char directory[8] = "frames";
  const char fileBase[8] = "Capture";
  char fileName[32];

  boost::filesystem::create_directories(directory);
  std::snprintf(fileName, sizeof(fileName), "%s%s%s%.4d.png",
                directory, "/", fileBase, count++);
  int tw = glutGet(GLUT_WINDOW_WIDTH);
  int th = glutGet(GLUT_WINDOW_HEIGHT);

  glReadPixels(0, 0,  tw, th, GL_RGBA, GL_UNSIGNED_BYTE, &mScreenshotTemp[0]);

  // reverse temp2 temp1
  for (int row = 0; row < th; row++) {
    memcpy(&mScreenshotTemp2[row * tw * 4],
           &mScreenshotTemp[(th - row - 1) * tw * 4], tw * 4);
  }

  unsigned result = lodepng::encode(fileName, mScreenshotTemp2, tw, th);

  // if there's an error, display it
  if (result) {
    std::cout << "lodepng error " << result << ": "
              << lodepng_error_text(result) << std::endl;
    return ;
  } else {
    std::cout << "wrote screenshot " << fileName << "\n";
    return ;
  }
}
np::ndarray toNumPyArray(const Eigen::VectorXd& vec)
{
	int n = vec.rows();
	p::tuple shape = p::make_tuple(n);
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray array = np::empty(shape,dtype);

	float* dest = reinterpret_cast<float*>(array.get_data());
	for(int i =0;i<n;i++)
	{
		dest[i] = vec[i];
	}

	return array;
}
void
SimWindow::
GetActionFromNN()
{
	p::object get_action;
	get_action= nn_module.attr("get_action");
	Eigen::VectorXd state = mWorld->GetState();
	p::tuple shape = p::make_tuple(state.rows());
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray state_np = np::empty(shape,dtype);
	
	float* dest = reinterpret_cast<float*>(state_np.get_data());
	for(int i =0;i<state.rows();i++)
		dest[i] = state[i];
	
	p::object temp = get_action(state_np);
	np::ndarray action_np = np::from_object(temp);

	float* srcs = reinterpret_cast<float*>(action_np.get_data());
	for(int i=0;i<mAction.rows();i++)
		mAction[i] = srcs[i];
}
Eigen::VectorXd
SimWindow::
GetActivationFromNN(const Eigen::VectorXd& mt)
{
	if(!mIsMuscleNNLoaded)
	{
		mWorld->GetDesiredTorques();
		return Eigen::VectorXd::Zero(mWorld->GetCharacter()->GetMuscles().size());
	}
	p::object get_activation = muscle_nn_module.attr("get_activation");
	// Eigen::VectorXd state = mWorld->GetState();
	// Eigen::VectorXd mt = mWorld->GetMuscleTorques();
	Eigen::VectorXd dt = mWorld->GetDesiredTorques();
	np::ndarray mt_np = toNumPyArray(mt);
	np::ndarray dt_np = toNumPyArray(dt);

	p::object temp = get_activation(mt_np,dt_np);
	np::ndarray activation_np = np::from_object(temp);

	Eigen::VectorXd activation(mWorld->GetCharacter()->GetMuscles().size());
	float* srcs = reinterpret_cast<float*>(activation_np.get_data());
	for(int i=0;i<activation.rows();i++)
		activation[i] = srcs[i];

	return activation;
}
#include <fstream>
void
SimWindow::
WriteOutput(const std::string& path)
{
	std::string real_path = path + "/"+ std::to_string(mOutputCount) +".tr";

	std::ofstream out(real_path);
	auto& skel = mWorld->GetCharacter()->GetSkeleton();
	for(int i =0;i<skel->getNumJoints();i++)
	{
		auto joint = skel->getJoint(i);
		if(joint->getType()=="FreeJoint")
		{
			out<<joint->getName()<<" ";
			out<<(joint->getPositions().segment<3>(3)*100.0).transpose()<<"\t"<<180.0/3.141592*dart::math::matrixToEulerXYZ(dart::math::expMapRot(joint->getPositions().segment<3>(0))).transpose()<<std::endl;
		}
		else if(joint->getType()=="BallJoint")
		{
			out<<joint->getName()<<" ";
			out<<180.0/3.141592*dart::math::matrixToEulerXYZ(dart::math::expMapRot(joint->getPositions())).transpose()<<std::endl;
		}
		else if(joint->getType()=="RevoluteJoint")
		{
			double p = joint->getPosition(0);
			Eigen::Vector3d axis = dynamic_cast<dart::dynamics::RevoluteJoint*>(joint)->getAxis();
			out<<joint->getName()<<" ";
			out<<180.0/3.141592*(axis*p).transpose()<<std::endl;
		}
	}

	out.close();

	mOutputCount++;

}
void
SimWindow::
Teaser()
{
	auto character = mWorld->GetCharacter();
	int count = 0;
	for(int i =0;i<200;i++)
	{
		
		
		save_positions.push_back(character->GetSkeleton()->getPositions());
		save_activations.push_back(mWorld->GetActivationLevels());	
		Step();
		if(mWorld->GetCharacter()->GetMotionGraph()->GetPhase()<0.001)
		{
			if(count<=1)
				count++;
			else
				break;
		} 
		
	}
	
}
