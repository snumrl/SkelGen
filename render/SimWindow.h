#ifndef __VMCON_SIM_WINDOW_H__
#define __VMCON_SIM_WINDOW_H__
#include "Camera.h"
#include "GLUTWindow.h"
#include "GLfunctions.h"
#include "DART_interface.h"
#include <string>
class IntegratedWorld;
class SimWindow : public GUI::GLUTWindow
{
public:
//	SimWindow(const std::string& record_path);
	SimWindow();

	std::shared_ptr<IntegratedWorld> mWorld;
	Eigen::VectorXd					mDisplacement;
	int 						mFrame;

	bool 						mIsPlay;
	bool 						mIsRotate;
	bool 						mIsDrag;
	bool 						mRenderDetail;
	bool                        mUseMuscle;
	bool                        mInitialGuessViewFlag;

	double						mDisplayRatio;


	void RenderString(float x, float y, void *font, const unsigned char* string, Eigen::Vector3d colors);
protected:
	void Display() override;
	void Keyboard(unsigned char key,int x,int y) override;
	void Mouse(int button, int state, int x, int y) override;
	void Motion(int x, int y) override;
	void Reshape(int w, int h) override;
	void Timer(int value) override;

	GLUquadric* qobj;

	// temporarily variable
	int highlightIdx;
};

#endif