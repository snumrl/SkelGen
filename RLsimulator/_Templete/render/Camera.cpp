#include "Camera.h"
#include <GL/glut.h>
using namespace GUI;

Camera::
Camera()
	// :fovy(60.0),lookAt(Eigen::Vector3d(0,0.0,0)),eye(Eigen::Vector3d(-1.9,0.4,1.3)),up(Eigen::Vector3d(0,1,0))
	:fovy(60.0),lookAt(Eigen::Vector3d(0,0.0,0)),eye(Eigen::Vector3d(-2.2,0.4,0.0)),up(Eigen::Vector3d(0,1,0))
{

}
	
void
Camera::
SetCamera(const Eigen::Vector3d& lookAt,const Eigen::Vector3d& eye,const Eigen::Vector3d& up)
{
	this->lookAt = lookAt, this->eye = eye, this->up = up;
}
void
Camera::
Apply()
{
	GLint w = glutGet(GLUT_WINDOW_WIDTH);
	GLint h = glutGet(GLUT_WINDOW_HEIGHT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	// gluPerspective(fovy, (GLfloat)w / (GLfloat)h, 0.01, 1000);
	glOrtho(-8.0/3.0,8.0/3.0,-0.5,2.5,1.0,200.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(eye.x(), eye.y(), eye.z(),
		lookAt.x(), lookAt.y(), lookAt.z(),
		up.x(), up.y(), up.z());
}

void
Camera::
Pan(int x,int y,int prev_x,int prev_y)
{
	double delta = (double)prev_y - (double)y;
	delta = 1 - delta / 200.0;
	eye = lookAt - (lookAt - eye)*delta;
}
void
Camera::
Zoom(int x,int y,int prev_x,int prev_y)
{
	double delta = (double)prev_y - (double)y;
	fovy += delta/20.0;
}
void
Camera::
Rotate(int x,int y,int prev_x,int prev_y)
{
	GLint w = glutGet(GLUT_WINDOW_WIDTH);
	GLint h = glutGet(GLUT_WINDOW_HEIGHT);

	Eigen::Vector3d prevPoint = GetTrackballPoint(prev_x,prev_y,w,h);
	Eigen::Vector3d curPoint = GetTrackballPoint(x,y,w,h);
	Eigen::Vector3d rotVec = curPoint.cross(prevPoint);

	rotVec = UnProject(rotVec);
	double cosT = curPoint.dot(prevPoint) / (curPoint.norm()*prevPoint.norm());
	double sinT = (curPoint.cross(prevPoint)).norm() / (curPoint.norm()*prevPoint.norm());

	double angle = -atan2(sinT, cosT);

	Eigen::Vector3d n = this->lookAt - this->eye;
	rotVec[0] = 0.0;
	// rotVec[1] = 1.0;
	rotVec[2] = 0.0;
	rotVec.normalize();
	n = Rotateq(n, rotVec, angle);
	this->up = Rotateq(this->up, rotVec, angle);
	this->eye = this->lookAt - n;
}
void
Camera::
Translate(int x,int y,int prev_x,int prev_y)
{
	Eigen::Vector3d delta((double)x - (double)prev_x, (double)y - (double)prev_y, 0);
	delta = UnProject(delta) / 200.0;
	lookAt += delta; eye += delta;
}
#include <iostream>
void
Camera::
SetLookAt(const Eigen::Vector3d& lookAt)
{
	Eigen::Vector3d diff = lookAt - this->lookAt;
	this->lookAt =lookAt;
	this->eye = this->eye + diff;
	this->eye[1] = this->lookAt[1];
	Eigen::Vector3d dir = (this->eye-this->lookAt).normalized();
	// double cos = dir[2];
	// double sin = dir[0];
	// // std::cout<<dir.transpose()<<std::endl;
	// if(cos>1.0)
	// 	cos=1.0;
	// if(cos<-1.0)
	// 	cos=-1.0;

	// double angle = std::acos(cos);
	// if(sin<0)
	// 	angle = 2*3.141592 - angle;
	// // std::cout<<angle<<std::endl;
	// angle+=0.05;
	// double dangle = std::fmod(angle,2*3.141592/12.0);
	// angle -=dangle;
	// cos = std::cos(angle);
	// sin = std::sin(angle);
	// dir[2] = cos;
	// dir[0] = sin;

	double n = (this->eye-this->lookAt).norm();
	this->eye = this->lookAt+dir*n;
}
#include <iostream>
Eigen::Vector3d
Camera::
GetDeltaPosition(int x,int y,int prev_x,int prev_y)
{
	Eigen::Vector3d delta((double)x - (double)prev_x, (double)y - (double)prev_y, 0);
	delta = UnProject(delta) / 200.0;
	return delta;
}
Eigen::Vector3d
Camera::
Rotateq(const Eigen::Vector3d& target, const Eigen::Vector3d& rotateVector,double angle)
{
	Eigen::Vector3d rv = rotateVector.normalized();

	Eigen::Quaternion<double> rot(cos(angle / 2.0), sin(angle / 2.0)*rv.x(), sin(angle / 2.0)*rv.y(), sin(angle / 2.0)*rv.z());
	rot.normalize();
	Eigen::Quaternion<double> tar(0, target.x(), target.y(), target.z());


	tar = rot.inverse()*tar*rot;

	return Eigen::Vector3d(tar.x(), tar.y(), tar.z());
}
Eigen::Vector3d
Camera::
GetTrackballPoint(int mouseX, int mouseY,int w,int h)
{
	double rad = sqrt((double)(w*w+h*h)) / 2.0;
	double dx = (double)(mouseX)-(double)w / 2.0;
	double dy = (double)(mouseY)-(double)h / 2.0;
	double dx2pdy2 = dx*dx + dy*dy;

	if (rad*rad - dx2pdy2 <= 0)
		return Eigen::Vector3d(dx/rad, dy/rad, 0);
	else
		return Eigen::Vector3d(dx/rad, dy/rad, sqrt(rad*rad - dx*dx - dy*dy)/rad);
}
Eigen::Vector3d
Camera::
UnProject(const Eigen::Vector3d& vec)
{
	Eigen::Vector3d n = lookAt - eye;
	n.normalize();
	
	Eigen::Vector3d v = up.cross(n);
	v.normalize();

	Eigen::Vector3d u = n.cross(v);
	u.normalize();

	return vec.z()*n + vec.x()*v + vec.y()*u;
}