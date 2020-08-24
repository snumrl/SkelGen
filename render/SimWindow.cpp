#include "SimWindow.h"
#include "../world/IntegratedWorld.h"
#include "../model/MusculoSkeletalSystem.h"
#include "../model/DART_helper.h"
#include "../simpleMotion/SimpleMotion.h"
#include "../generator/MuscleGenerator.h"

#include <algorithm>
#include <fstream>
#include <boost/filesystem.hpp>
#include <GL/glut.h>
#include <GL/freeglut.h>
#include <stdio.h>

using namespace GUI;
using namespace dart::simulation;
using namespace dart::dynamics;

Eigen::VectorXd p_target;
Eigen::VectorXd p_initial;

SimWindow::
SimWindow()
	:GLUTWindow(),mIsRotate(true),mIsDrag(false),mIsPlay(false),mFrame(0),mDisplayRatio(1.0),mRenderDetail(false),mUseMuscle(false),mInitialGuessViewFlag(false),highlightIdx(0)
{
	mWorld = std::make_shared<IntegratedWorld>();
//	mWorld->initialize(NULL);
	//LoadFromFolder(record_path);
	// WriteTransformation("../output_trans/");
	//mDisplacement = mWorld->GetRecords()[0]->rigid_body_positions["HUMAN"];
//	mDisplacement.setZero();
//	mDisplayTimeout = 33;
//
//	p_target = mWorld->GetMusculoSkeletalSystem()->getSkeleton()->getPositions();
//	p_target[0] -= 1.4;
//	p_target[6] += 1.4;

    qobj = gluNewQuadric();
    gluQuadricNormals(qobj, GLU_SMOOTH);
}
void SimWindow::
RenderString(float x, float y, void *font, const unsigned char* string, Eigen::Vector3d colors)
{  
  char *c;

  glColor3f(colors[0], colors[1], colors[2]); 
  glRasterPos2f(x, y);

  glutBitmapString(font, string);
}
void
SimWindow::
Display() 
{
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

//	GUI::DrawStringOnScreen(0.8,0.2,std::to_string(mWorld->GetRigidWorld()->getTime()),true,Eigen::Vector3d(0,0,0));

    for(int i =0;i<mWorld->GetRigidWorld()->getNumSkeletons();i++)
	{
		if (i==0) { // Standard
			DrawSkeleton(mWorld->GetRigidWorld()->getSkeleton(i), Eigen::Vector3d(1.0, 0.1, 0.1), !mRenderDetail);
		}else if (i==1) { // Re-targeted
			DrawSkeleton(mWorld->GetRigidWorld()->getSkeleton(i), Eigen::Vector3d(0.1, 1.0, 0.1), !mRenderDetail);
		}else{
			DrawSkeleton(mWorld->GetRigidWorld()->getSkeleton(i), Eigen::Vector3d(0.3*i+0.3, 0.3*i+0.3, 0.3*i+0.3), !mRenderDetail);
		}
    }
    // Draw muscles
	if((mRenderDetail || !mRenderDetail) && mUseMuscle)
	{
		for (int i=0;i<mWorld->GetMusculoSkeletalSystem()->getMuscles().size();i++){
			auto& mus = mWorld->GetMusculoSkeletalSystem()->getMuscles()[i];
			DrawMuscleWayPoints(mus, qobj, false, highlightIdx == i);
		}

		for (int i=0;i<mWorld->GetCmpMusculoSkeletalSystem()->getMuscles().size();i++){
			auto& mus = mWorld->GetCmpMusculoSkeletalSystem()->getMuscles()[i];
			DrawMuscleWayPoints(mus, qobj, false, highlightIdx == i);
		}

		if (mInitialGuessViewFlag) {
			setWaypointsAsInitial(mWorld->mRtgMusculoSkeletalSystem);
			for (auto &mus: mWorld->GetCmpMusculoSkeletalSystem()->getMuscles()) {
				DrawMuscleWayPoints(mus, qobj, true);
			}
			repairWaypoints(mWorld->mRtgMusculoSkeletalSystem);
		}
	}


	// const unsigned char text[10] = "Hello";
	// RenderString(-1.0, 1.0, GLUT_BITMAP_TIMES_ROMAN_24, text, Eigen::Vector3d(1.0f, 0.0f, 0.0f));
	glutSwapBuffers();
}
void
SimWindow::
Keyboard(unsigned char key,int x,int y)
{
	switch(key)
	{
		case 'a' : mRenderDetail =!mRenderDetail;break;

		case 27: exit(0);break;
		default : break;
	}


	glutPostRedisplay();
}
void
SimWindow::
Mouse(int button, int state, int x, int y) 
{
	if ((button ==3) || (button == 4))
	{
		if(state == GLUT_UP) return;
		if(button == 3)
			mCamera->Zoom(0,0,-30,-30);
		if(button == 4)
			mCamera->Zoom(0,0,30,30);

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
		mMouseType = 0;
	}

	// const unsigned char text[10] = "Hello";
	// RenderString(-1.0, 1.0, GLUT_BITMAP_TIMES_ROMAN_24, text, Eigen::Vector3d(1.0f, 0.0f, 0.0f));

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
		mCamera->Rotate(x,y,mPrevX,mPrevY);
	}
	else if (mMouseType == GLUT_RIGHT_BUTTON)
	{
		mCamera->Translate(x,y,mPrevX,mPrevY);
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
int time_count=0;
void
SimWindow::
Timer(int value) 
{


//	p_initial = mWorld->GetMusculoSkeletalSystem()->getSkeleton()->getPositions();
//	Eigen::VectorXd p = (p_target-p_initial)/200.0*time_count+p_initial;
//	Eigen::VectorXd v = p;
//	v.setZero();

//	mWorld->TimeStepping();
	time_count++;

	glutPostRedisplay();
	glutTimerFunc(mDisplayTimeout, TimerEvent,1);
}
