#include "DART_interface.h"
using namespace dart::dynamics;
using namespace dart::simulation;

void
GUI::
DrawSkeleton(
		const dart::dynamics::SkeletonPtr& skel,
		std::map<std::string, int> name2ID,
		const Eigen::Vector3d& color,
		bool box)
{
	for(int i=0;i<skel->getNumBodyNodes();i++)
	{
		auto bn = skel->getBodyNode(i);
		auto shapeNodes = bn->getShapeNodesWith<VisualAspect>();

		// for(int j=0;j<shapeNodes.size();j++){
		int j = (box? 1:0);
		if(shapeNodes.size() ==1)
			j=0;
		auto T = shapeNodes[j]->getTransform();
		glPushName(name2ID[bn->getName()]);
		DrawShape(T,shapeNodes[j]->getShape().get(),color);
		glPopName();
		// }
	}
	// for(int i =0;i<skel->getNumJoints();i++)
	// {
	// 	auto parent = skel->getJoint(i)->getParentBodyNode();
	// 	Eigen::Isometry3d T;
	// 	T.setIdentity();
	// 	if(parent!=nullptr)
	// 		T = parent->getTransform();
	// 	T = T*skel->getJoint(i)->getTransformFromParentBodyNode();
	// 	glPushMatrix();
	// 	glMultMatrixd(T.data());
	// 	glBegin(GL_LINES);
	// 	glColor3f(1,0,0);
	// 	glVertex3f(0,0,0);
	// 	glVertex3f(0.1,0,0);

	// 	glColor3f(0,1,0);
	// 	glVertex3f(0,0,0);
	// 	glVertex3f(0,0.1,0);

	// 	glColor3f(0,0,1);
	// 	glVertex3f(0,0,0);
	// 	glVertex3f(0,0,0.1);

	// 	glEnd();
	// 	glPopMatrix();
	// }
}

void
GUI::
DrawSkeleton(
		const dart::dynamics::SkeletonPtr& skel,
		const Eigen::Vector3d& color,
		bool box)
{
	for(int i=0;i<skel->getNumBodyNodes();i++)
	{
		auto bn = skel->getBodyNode(i);
		auto shapeNodes = bn->getShapeNodesWith<VisualAspect>();

		// for(int j=0;j<shapeNodes.size();j++){
		int j = (box? 1:0);
		if(shapeNodes.size() ==1)
			j=0;
		auto T = shapeNodes[j]->getTransform();
		DrawShape(T,shapeNodes[j]->getShape().get(),color);
		// }
	}
	// for(int i =0;i<skel->getNumJoints();i++)
	// {
	// 	auto parent = skel->getJoint(i)->getParentBodyNode();
	// 	Eigen::Isometry3d T;
	// 	T.setIdentity();
	// 	if(parent!=nullptr)
	// 		T = parent->getTransform();
	// 	T = T*skel->getJoint(i)->getTransformFromParentBodyNode();
	// 	glPushMatrix();
	// 	glMultMatrixd(T.data());
	// 	glBegin(GL_LINES);
	// 	glColor3f(1,0,0);
	// 	glVertex3f(0,0,0);
	// 	glVertex3f(0.1,0,0);

	// 	glColor3f(0,1,0);
	// 	glVertex3f(0,0,0);
	// 	glVertex3f(0,0.1,0);

	// 	glColor3f(0,0,1);
	// 	glVertex3f(0,0,0);
	// 	glVertex3f(0,0,0.1);

	// 	glEnd();
	// 	glPopMatrix();
	// }
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
		// glColor3f(0,0,0);
		// glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
		// GUI::DrawSphere(sphere->getRadiusLength());
	}
	else if (shape->is<BoxShape>())
	{
		const auto* box = dynamic_cast<const BoxShape*>(shape);
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    	GUI::DrawCube(box->getSize());
    	// GUI::DrawCube(Eigen::Vector3d(0.01,0.01,0.01));
	}
	else if(shape->is<MeshShape>())
	{
		auto* mesh = dynamic_cast<const MeshShape*>(shape);

		// for(int i =0;i<16;i++)
			// std::cout<<(*mesh->getMesh()->mRootNode->mTransformation)[i]<<" ";
    	GUI::DrawMesh(mesh->getScale(),mesh->getMesh());

	}

	glPopMatrix();

	// glDisable(GL_COLOR_MATERIAL);
}
void
GUI::
DrawMuscleWayPoints(std::shared_ptr<Muscle> muscle, GLUquadric* qobj, bool initialFlag, bool highlightFlag)
{
	std::vector<BlendedWaypoint> bwp;
	bwp = muscle->blendedWaypoints;
	for(int i=0;i<bwp.size()-1;i++) {
//		BlendedWaypoint
//		GUI::DrawLine(getBlendedWaypointGlobalCoord(bwp[i]), getBlendedWaypointGlobalCoord(bwp[i + 1]),Eigen::Vector3d(0,0,0));
		Eigen::Vector3d p0 = getBlendedWaypointGlobalCoord(bwp[i]);
		Eigen::Vector3d p1 = getBlendedWaypointGlobalCoord(bwp[i + 1]);
		Eigen::Vector3d axis = (p1-p0);
		Eigen::Vector3d center = (p0+p1)/2;

//      AnchorPoint
//		GUI::DrawLine(getAnchorPointGlobalCoord(bwp[i].ap), getAnchorPointGlobalCoord(bwp[i + 1].ap),Eigen::Vector3d(0,0,0));
//		Eigen::Vector3d p0 = getAnchorPointGlobalCoord(bwp[i].ap);
//		Eigen::Vector3d p1 = getAnchorPointGlobalCoord(bwp[i + 1].ap);
//		Eigen::Vector3d axis = (p1-p0);
//		Eigen::Vector3d center = (p0+p1)/2;
		glPushMatrix();


		// Cylinder
		if (!initialFlag){
			if (!highlightFlag) glColor3f(COLOR::muscleLine[0], COLOR::muscleLine[1], COLOR::muscleLine[2]);
			else glColor3f(0,1,0);
		}else{
			glColor3f(0.7,0.3,0.2);
		}
//		glColor3f(!initialFlag ? 0.0 : 0.7,!initialFlag ? 0.0 : 0.3,!initialFlag ? 1.0 : 0.2);
//		glColor3f(0.7,0.3,0.2);


		Eigen::Vector3d cross = Eigen::Vector3d(0,0,1).cross(axis);
		cross = cross.normalized();
		double angle = atan2(axis.normalized().cross(Eigen::Vector3d(0,0,1)).norm(), axis.normalized().dot(Eigen::Vector3d(0,0,1)));
		glTranslated(p0[0], p0[1], p0[2]);
		glRotated(angle*180/M_PI, cross[0], cross[1], cross[2]);
		gluCylinder(qobj, !initialFlag ? 0.003 : 0.002, !initialFlag ? 0.003 : 0.002, axis.norm(), 5, 16);


		// Waypoints
		glColor3d(COLOR::muscleWaypoint[0], COLOR::muscleWaypoint[1], COLOR::muscleWaypoint[2]);
		gluSphere(qobj, !initialFlag ? 0.004 : 0.0039, 16, 16);

		glPopMatrix();

		if (i==bwp.size()-2){
			glPushMatrix();
			glTranslated(p1[0], p1[1], p1[2]);
			glColor3f(0, 1, 0);
			gluSphere(qobj, !initialFlag ? 0.004 : 0.0039,  16, 16);
			glPopMatrix();
		}
	}
}