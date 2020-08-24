/*#include "SimWindow.h"
#include <vector>
#include <string>
#include <GL/glut.h>

int main(int argc,char** argv)
{
	std::string record_path;
	if(argc==2)
		record_path = argv[1];
	else
		record_path = "../output/";

	if(record_path.back()!='/')
		record_path.append("/");
	SimWindow simwindow(record_path);
	//SimWindow simwindow;

	std::cout<<"- : mDisplayRatio*=0.9"<<std::endl;
	std::cout<<"+ : mDisplayRatio*=1.1"<<std::endl;
	std::cout<<"q : mRenderDetail =!mRenderDetail"<<std::endl;
	std::cout<<"  : mIsPlay =!mIsPlay"<<std::endl;
	std::cout<<"r : mFrame = 0"<<std::endl;
	std::cout<<"[ : mFrame--"<<std::endl;
	std::cout<<"] : mFrame++"<<std::endl;

	glutInit(&argc, argv);
	simwindow.InitWindow(800,800,"-");
	glutMainLoop();
}
*/