#include "SimWindow.h"
#include <vector>
#include <string>
#include <GL/glut.h>

namespace p = boost::python;
namespace np = boost::python::numpy;

int main(int argc,char** argv)
{
	Py_Initialize();
	np::initialize();

	glutInit(&argc, argv);
	SimWindow* simwindow;
	if(argc==1)
		simwindow = new SimWindow();
	else if (argc==2)
		simwindow = new SimWindow(argv[1]);
	else if (argc==3)
		simwindow = new SimWindow(argv[1],argv[2]);
	
	// simwindow->InitWindow(1920,1080,"Render");
	simwindow->InitWindow(1920,1080,"Render");
	// simwindow->InitWindow(1080,720,"Render");
	glutMainLoop();
}
