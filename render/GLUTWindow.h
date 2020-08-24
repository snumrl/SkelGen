#ifndef __GUI_GLUT_WINDOW_H__
#define __GUI_GLUT_WINDOW_H__
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <memory>

namespace GUI
{
class Camera;
class GLUTWindow
{
public:
	GLUTWindow();
	~GLUTWindow();

	virtual void InitWindow(int _w,int _h,const char* _name);
	
	static GLUTWindow* current();
	static void DisplayEvent();
	static void KeyboardEvent(unsigned char key,int x,int y);
	static void MouseEvent(int button, int state, int x, int y);
	static void MotionEvent(int x, int y);
	static void ReshapeEvent(int w, int h);
	static void TimerEvent(int value);

	static std::vector<GLUTWindow*> mWindows;
	static std::vector<int> mWinIDs;
	
protected:
	virtual void initLights();
	virtual void Display() = 0;
	virtual void Keyboard(unsigned char key,int x,int y) = 0;
	virtual void Mouse(int button, int state, int x, int y) = 0;
	virtual void Motion(int x, int y) = 0;
	virtual void Reshape(int w, int h) = 0;
	virtual void Timer(int value) = 0;
protected:
	std::unique_ptr<Camera> 		mCamera;
	bool 							mIsDrag;
	int 							mMouseType;
	int 							mPrevX,mPrevY;
	int 							mDisplayTimeout;
};

};
#endif