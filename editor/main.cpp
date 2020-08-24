#include <boost/filesystem.hpp>
#include <stdlib.h>
#include <memory>

#include "EditorWindow.h"
#include "../APTOpt/APTWindow.h"
#include <stdio.h>
#include <algorithm>

#include <vector>
#include <string>
#include <GL/glut.h>
using namespace std;

void Render(int argc,char** argv,EditorWindow *editorWindow ){
	cout<<"q: wp.x++"<<endl;
	cout<<"w: wp.x--"<<endl;
	cout<<"a: wp.y++"<<endl;
	cout<<"s: wp.y--"<<endl;
	cout<<"z: wp.z++"<<endl;
	cout<<"x: wp.z--"<<endl;

	cout << "current waypoint index : " << 1 << endl;

	glutInit(&argc, argv);
	editorWindow->InitWindow(1080,1080,"Render");
	editorWindow->setInitialCameraView();
	glutMainLoop();
}

void Render(int argc,char** argv,APTWindow *aptWindow ){
	cout<<"ROM Representation"<<endl;
	glutInit(&argc, argv);
	aptWindow->InitWindow(800,800,"Render");
	glutMainLoop();
}

void Exam(APTWindow *aptWindow)
{
	aptWindow->checkTorque(aptWindow->mWorld->GetMusculoSkeletalSystem());
	// aptWindow->drawAPTGraph(aptWindow->mWorld->GetCmpMusculoSkeletalSystem(), "RTG");
}

int main(int argc,char** argv)
{
	// APTWindow *aptWindow = new APTWindow();
	// Exam(aptWindow);
	// Render(argc, argv, aptWindow);



	EditorWindow *editorWindow = new EditorWindow();
//	editorWindow->drawGraph();
	Render(argc, argv, editorWindow);

	return 0;
}