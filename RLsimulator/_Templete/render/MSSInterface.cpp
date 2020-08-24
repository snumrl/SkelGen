#include "MSSInterface.h"
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace MSS;

void
GUI::
DrawSkeleton(
	const dart::dynamics::SkeletonPtr& skel,
	const Eigen::Vector3d& color,
	bool drawBox)
{
	for(int i=0;i<skel->getNumBodyNodes();i++)
	{
		auto bn = skel->getBodyNode(i);
		auto shapeNodes = bn->getShapeNodesWith<VisualAspect>();

		auto T = shapeNodes.back()->getTransform();
		// if(T.translation()[1]>1.0)
		
		if(bn->getName().find("Support")!=std::string::npos){
			// std::cout<<bn->getName()<<std::endl;
			if(drawBox)
				DrawShape(T,shapeNodes[0]->getShape().get(),Eigen::Vector3d(0.6,0.6,0.8));
			else
				DrawShape(T,shapeNodes.back()->getShape().get(),Eigen::Vector3d(0.6,0.6,0.8));
		}
		else
		{
			if(drawBox)
				DrawShape(T,shapeNodes[0]->getShape().get(),color);
			else
				DrawShape(T,shapeNodes.back()->getShape().get(),color);
		}

	}
	// for(int i =0;i<skel->getNumJoints();i++)
	// {
	// 	auto parent = skel->getJoint(i)->getParentBodyNode();
	// 	if(skel->getJoint(i)->getType()=="FreeJoint")
	// 		continue;
	// 	else if(skel->getJoint(i)->getType()=="BallJoint")
	// 		glColor3f(0.8,0.2,0.2);
	// 	else if(skel->getJoint(i)->getType()=="RevoluteJoint")
	// 		glColor3f(0.2,0.8,0.2);
	// 	Eigen::Isometry3d T;
	// 	T.setIdentity();
	// 	if(parent!=nullptr)
	// 		T = parent->getTransform();
	// 	T = T*skel->getJoint(i)->getTransformFromParentBodyNode();
	// 	glPushMatrix();
	// 	glMultMatrixd(T.data());
		
	// 	GUI::DrawSphere(0.004);
	// 	glPopMatrix();
    	
	// }
}
void
GUI::
DrawGround(double y)
{
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	
	// glBegin(GL_QUADS);
	// glVertex3f(-1000,y,-1000);
	// glVertex3f(1000,y,-1000);
	// glVertex3f(1000,y,1000);
	// glVertex3f(-1000,y,1000);
	// glEnd();

	// glColor3f(0.0,0.0,0.0);

/*	double width = 0.005;
	int count = 0;
	glBegin(GL_QUADS);
	for(double x = -100.0;x<100.01;x+=3.0)
	{
		for(double z = -100.0;z<100.01;z+=3.0)
		{
			if(count%2==0)
				glColor3f(0xDF/255.0,0xD3/255.0,0xCF/255.0);			
			else
				glColor3f(0xC7/255.0,0xB7/255.0,0xAB/255.0);			
			count++;
			glVertex3f(x,y,z);
			glVertex3f(x+0.3,y,z);
			glVertex3f(x+0.3,y,z+1.0);
			glVertex3f(x,y,z+1.0);
		}
	}
	glEnd();*/
	glColor3f(0xC7/255.0,0xB7/255.0,0xAB/255.0);
	double width = 0.01;
	glBegin(GL_QUADS);
	for(double x = -100.0;x<100.01;x+=0.5)
	{
		glVertex3f(x-width,y+0.01,-100.0);
		glVertex3f(x-width,y+0.01,100.0);
		glVertex3f(x+width,y+0.01,100.0);
		glVertex3f(x+width,y+0.01,-100.0);
	}
	glEnd();

	glBegin(GL_QUADS);
	for(double x = -100.0;x<100.01;x+=0.5)
	{
		glVertex3f(-100.0,y+0.01,x-width);
		glVertex3f(100.0,y+0.01,x-width);
		glVertex3f(100.0,y+0.01,x+width);
		glVertex3f(-100.0,y+0.01,x+width);
	}
	glEnd();
	glColor3f(0xDF/255.0,0xD3/255.0,0xCF/255.0);		
	glBegin(GL_QUADS);
	glVertex3f(-1000,-1000,1000);
	glVertex3f(-1000,1000,1000);
	glVertex3f(1000,1000,1000);
	glVertex3f(1000,-1000,1000);
	glEnd();
}
void
GUI::
DrawMuscles(const std::vector<Muscle*>& muscles)
{
	// glDisable(GL_LIGHTING);
	int count =0;
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
	for(auto muscle : muscles)
	{
		auto aps = muscle->GetAnchors();
		bool lower_body = true;
		if(muscle->name.find("Wrist_Extensor")!=std::string::npos)
			continue;

		for(auto ap : aps)
		{
			for(auto bn : ap->bodynodes)
			{
				if(bn->getName()=="Torso"||bn->getName()=="Spine"||bn->getName()=="ShoulderR"||bn->getName()=="ShoulderL"||bn->getName()=="Head"||bn->getName()=="Neck"||bn->getName()=="ArmL"||bn->getName()=="ArmR"||bn->getName()=="ForeArmL"||bn->getName()=="ForeArmR"||bn->getName()=="HandR"||bn->getName()=="HandL")
					lower_body=false;
			}
				

		}
		double a = muscle->activation * 5;
		// Eigen::Vector3d color(0.7*(3.0*a),0.2,0.7*(1.0-3.0*a));
		// Eigen::Vector3d idle(0x80, 0x80, 0x80);
		Eigen::Vector3d idle(80, 130, 245);
		// Eigen::Vector3d idle(0, 80, 195);
		Eigen::Vector3d act(0xFF, 0x00, 0x00);
		Eigen::Vector3d color;//0.7*(1.0-3.0*a));
		color = idle * (1-a) + act * a;
		// glColor3f(1.0,0.0,0.362);
		// glColor3f(0.0,0.0,0.0);
		color /= 255.0;
		glColor3f(color[0],color[1],color[2]);
		for(int i=0;i<aps.size();i++)
		{
			Eigen::Vector3d p = aps[i]->GetPoint();
			if(lower_body)
				GUI::DrawSphere(p,0.0025);
			else
				GUI::DrawSphere(p,0.0025);
		}
			
		for(int i=0;i<aps.size()-1;i++)
		{
			Eigen::Vector3d p = aps[i]->GetPoint();
			Eigen::Vector3d p1 = aps[i+1]->GetPoint();
			if(lower_body)
				GUI::drawCylinder(0.002,p,p1);
			else
				GUI::drawCylinder(0.002,p,p1);
		}
		
	}
	glEnable(GL_LIGHTING);
	// glDisable(GL_DEPTH_TEST);
}
void
GUI::
DrawShape(const Eigen::Isometry3d& T,
	const dart::dynamics::Shape* shape,
	const Eigen::Vector3d& color)
{
	glEnable(GL_LIGHTING);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(color[0],color[1],color[2]);
	glPushMatrix();
	glMultMatrixd(T.data());
	if(shape->is<SphereShape>())
	{
		const auto* sphere = dynamic_cast<const SphereShape*>(shape);
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		GUI::DrawSphere(sphere->getRadius());
	}
	else if (shape->is<BoxShape>())
	{
		const auto* box = dynamic_cast<const BoxShape*>(shape);
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    	GUI::DrawCube(box->getSize());
	}
	else if (shape->is<CylinderShape>())
	{
		const auto* cylinder = dynamic_cast<const CylinderShape*>(shape);
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		GUI::drawCylinder(cylinder->getRadius(),cylinder->getHeight());
	}
	else if(shape->is<MeshShape>())
	{
		auto* mesh = dynamic_cast<const MeshShape*>(shape);
    	GUI::DrawMesh(mesh->getScale(),mesh->getMesh());

	}
	glPopMatrix();
}
