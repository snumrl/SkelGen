#include <boost/filesystem.hpp>
#include <stdlib.h>
#include <memory>

#include "world/IntegratedWorld.h"
#include "model/MusculoSkeletalSystem.h"
#include "ExamWindow.h"
#include "generator/ObjEditing.h"
#include <stdio.h>
#include <algorithm>

#include <vector>
#include <string>
#include <GL/glut.h>
using namespace std;

void Render(int argc,char** argv,ExamWindow *examWindow ){
	cout<<"q: wp.x++"<<endl;
	cout<<"w: wp.x--"<<endl;
	cout<<"a: wp.y++"<<endl;
	cout<<"s: wp.y--"<<endl;
	cout<<"z: wp.z++"<<endl;
	cout<<"x: wp.z--"<<endl;

	cout << "current waypoint index : " << 1 << endl;

	glutInit(&argc, argv);
	examWindow->InitWindow(1080,1080,"Render");
	glutMainLoop();
}

int main(int argc,char** argv)
{
	ExamWindow *examWindow = new ExamWindow();
	examWindow->drawGraph();
	Render(argc, argv, examWindow);

	return 0;
}