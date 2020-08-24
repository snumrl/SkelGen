#include <tinyxml.h>
#include "SkeletonBuilder.h"

using namespace dart::dynamics;


namespace MSS
{
Eigen::Vector3d Proj(const Eigen::Vector3d& u,const Eigen::Vector3d& v)
{
	Eigen::Vector3d proj;
	proj = u.dot(v)/u.dot(u)*u;
	return proj;	
}
Eigen::Isometry3d Orthonormalize(const Eigen::Isometry3d& T_old)
{
	Eigen::Isometry3d T;
	T.translation() = T_old.translation();
	Eigen::Vector3d v0,v1,v2;
	Eigen::Vector3d u0,u1,u2;
	v0 = T_old.linear().col(0);
	v1 = T_old.linear().col(1);
	v2 = T_old.linear().col(2);

	u0 = v0;
	u1 = v1 - Proj(u0,v1);
	u2 = v2 - Proj(u0,v2) - Proj(u1,v2);

	u0.normalize();
	u1.normalize();
	u2.normalize();

	T.linear().col(0) = u0;
	T.linear().col(1) = u1;
	T.linear().col(2) = u2;
	return T;
}

SkeletonPtr 
SkeletonBuilder::
BuildFromFile(const std::string& filename){
	TiXmlDocument doc;
	if(!doc.LoadFile(filename)){
		std::cout << "Can't open file : " << filename << std::endl;
		return nullptr;
	}

	TiXmlElement *skeldoc = doc.FirstChildElement("Skeleton");
	
	std::string skelname = skeldoc->Attribute("name");
	SkeletonPtr skel = Skeleton::create(skelname);
	std::cout << skelname;

	for(TiXmlElement *body = skeldoc->FirstChildElement("Joint"); body != nullptr; body = body->NextSiblingElement("Joint")){
		// type
		std::string jointType = body->Attribute("type");
		// name
		std::string name = body->Attribute("name");
		// parent name
		std::string parentName = body->Attribute("parent_name");
		std::string objName = "None";
		if(body->Attribute("obj")!=nullptr)
			objName = body->Attribute("obj");
		BodyNode *parent;
		if(!parentName.compare("None"))
			parent = nullptr;
		else
			parent = skel->getBodyNode(parentName);
		// size
		Eigen::Vector3d size = MSS::string_to_vector3d(std::string(body->Attribute("size")));
		// body position
		TiXmlElement *bodyPosElem = body->FirstChildElement("BodyPosition");
		Eigen::Isometry3d bodyPosition;
		bodyPosition.setIdentity();
		bodyPosition.linear() = MSS::string_to_matrix3d(bodyPosElem->Attribute("linear"));
		bodyPosition.translation() = MSS::string_to_vector3d(bodyPosElem->Attribute("translation"));
		bodyPosition = Orthonormalize(bodyPosition);
		// joint position
		TiXmlElement *jointPosElem = body->FirstChildElement("JointPosition");
		Eigen::Isometry3d jointPosition;
		jointPosition.setIdentity();
		if(jointPosElem->Attribute("linear")!=nullptr)
			jointPosition.linear() = MSS::string_to_matrix3d(jointPosElem->Attribute("linear"));
		jointPosition.translation() = MSS::string_to_vector3d(jointPosElem->Attribute("translation"));
		jointPosition = Orthonormalize(jointPosition);
		// mass
		double mass = atof(body->Attribute("mass"));
		
		bool contact = false;
		
		// if(body->Attribute("contact")!=nullptr)
		// {
			// if(std::string(body->Attribute("contact"))=="On")
		contact = true;
		// }
		if(!jointType.compare("FreeJoint") ){
			SkeletonBuilder::MakeFreeJointBody(
				name,
				objName,
				skel,
				parent,
				size,
				jointPosition,
				bodyPosition,
				mass,
				contact
				);
		}
		else if(!jointType.compare("PlanarJoint") ){
			SkeletonBuilder::MakePlanarJointBody(
				name,
				objName,
				skel,
				parent,
				size,
				jointPosition,
				bodyPosition,
				mass,
				contact
				);
		}
		else if(!jointType.compare("BallJoint")){
			// joint limit
			bool limit_enforced = false;
			Eigen::Vector3d upperLimit(1E6,1E6,1E6), lowerLimit(-1E6,-1E6,-1E6);
			if(jointPosElem->Attribute("upper")!=nullptr)
			{
				limit_enforced = true;
				upperLimit = MSS::string_to_vector3d(jointPosElem->Attribute("upper"));
				lowerLimit = MSS::string_to_vector3d(jointPosElem->Attribute("lower"));
			}
			
			SkeletonBuilder::MakeBallJointBody(
				name,
				objName,
				skel,
				parent,
				size,
				jointPosition,
				bodyPosition,
				limit_enforced,
				upperLimit,
				lowerLimit,
				mass,
				contact
				);
		}
		else if(!jointType.compare("RevoluteJoint")){
			// joint limit
			bool limit_enforced = false;
			double upperLimit(1E6), lowerLimit(-1E6);
			if(jointPosElem->Attribute("upper")!=nullptr)
			{
				limit_enforced = true;
				upperLimit = atof(jointPosElem->Attribute("upper"));
				lowerLimit = atof(jointPosElem->Attribute("lower"));
			}

			// axis
			Eigen::Vector3d axis = MSS::string_to_vector3d(body->Attribute("axis"));

			SkeletonBuilder::MakeRevoluteJointBody(
				name,
				objName,
				skel,
				parent,
				size,
				jointPosition,
				bodyPosition,
				limit_enforced,
				upperLimit,
				lowerLimit,
				mass,
				axis,
				contact
				);			
		}
		else if(!jointType.compare("PrismaticJoint")){
			// joint limit
			TiXmlElement *jointLimitElem = body->FirstChildElement("Limit");
			bool isLimitEnforced = false;
			double upperLimit, lowerLimit;
			if( jointLimitElem != nullptr ){
				isLimitEnforced = true;
				upperLimit = atof(jointLimitElem->Attribute("upper"));
				lowerLimit = atof(jointLimitElem->Attribute("lower"));
			}
			// axis
			Eigen::Vector3d axis = MSS::string_to_vector3d(body->Attribute("axis"));

			SkeletonBuilder::MakePrismaticJointBody(
				name,
				objName,
				skel,
				parent,
				size,
				jointPosition,
				bodyPosition,
				isLimitEnforced,
				upperLimit,
				lowerLimit,
				mass,
				axis,
				contact
				);	
		}
		else if(!jointType.compare("WeldJoint")){
			SkeletonBuilder::MakeWeldJointBody(
				name,
				objName,
				skel,
				parent,
				size,
				jointPosition,
				bodyPosition,
				mass,
				contact
				);			
		}

	}
	std::cout<<"(DOFs : "<<skel->getNumDofs()<<")"<< std::endl;
	return skel;
}


/*
void 
SkeletonBuilder::
WriteSkeleton(std::string filename, dart::dynamics::SkeletonPtr& skel){

	std::cout << std::endl << std::endl << "Write skeleton" << std::endl;

	// get body node && name
	BodyNode* bn = skel->getBodyNode(0);
	std::string name = bn->getName();

	// get parent body node && name
	BodyNode* parent = bn->getParentBodyNode();
	std::string parent_name;
	if(parent == nullptr)
		parent_name = "None";
	else
		parent_name = parent->getName();

	// get mass
	double mass = bn->getMass();

	// get size
	ShapeNode* sn = bn->getShapeNodesWith<DynamicsAspect>()[0];
	ShapeNode::BasicProperties props = sn->getShapeNodeProperties();
	ShapePtr sp = props.mShape;
	std::string shape_type = sp->getType();
	Eigen::Vector3d size;
	if(!shape_type.compare("BoxShape")){
		std::shared_ptr<BoxShape> bs = std::dynamic_pointer_cast<BoxShape>(sp);
		size = bs->getSize();
	}
	else{
		std::cout << "undefined shpae type" << std::endl;
	}

	// get type of joint
	std::string type = bn->getParentJoint()->getType();
	if(!type.compare("PrismaticJoint")){
		PrismaticJoint *pj = dynamic_cast<PrismaticJoint*>(bn->getParentJoint());
		PrismaticJoint::Properties props = pj->getPrismaticJointProperties();
	
		Eigen::Vector3d axis = props.mAxis;

		std::cout << mass << std::endl;
		std::cout << name << std::endl;
		std::cout << parent_name << std::endl;
		std::cout << type << std::endl;
		std::cout << axis.transpose() << std::endl;
		std::cout << size.transpose() << std::endl;
	}


}
*/

BodyNode* SkeletonBuilder::MakeFreeJointBody(
	const std::string& body_name,
	const std::string& obj_file,
	const dart::dynamics::SkeletonPtr& target_skel,
	dart::dynamics::BodyNode* const parent,
	const Eigen::Vector3d& size,
	const Eigen::Isometry3d& joint_position,
	const Eigen::Isometry3d& body_position,
	double mass,
	bool contact)
{
	ShapePtr shape = std::shared_ptr<BoxShape>(new BoxShape(size));

	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	BodyNode* bn;
	FreeJoint::Properties props;
	props.mName = body_name;
	// props.mT_ChildBodyToJoint = joint_position;
	props.mT_ParentBodyToJoint = body_position;


	bn = target_skel->createJointAndBodyNodePair<FreeJoint>(
		parent,props,BodyNode::AspectProperties(body_name)).second;

	if(contact)
		bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	else
		bn->createShapeNodeWith<VisualAspect, DynamicsAspect>(shape);
	if(obj_file!="None")
	{
		std::string obj_path = std::string(MSS_ROOT_DIR)+"/model/OBJ/"+obj_file;
		const aiScene* mesh = MeshShape::loadMesh(std::string(obj_path));
		
		ShapePtr visual_shape = std::shared_ptr<MeshShape>(new MeshShape(Eigen::Vector3d(1.00,1.00,1.00),mesh));
		//bn = target_skel->getRootBodyNode();
		auto vsn = bn->createShapeNodeWith<VisualAspect>(visual_shape);
		Eigen::Isometry3d T_visual;
		T_visual.setIdentity();
		// T_visual = body_position.inverse();
		vsn->setRelativeTransform(T_visual);
	}
	bn->setInertia(inertia);
	// bn->setRestitutionCoeff(0.0);
	return bn;
}
BodyNode* SkeletonBuilder::MakePlanarJointBody(
		const std::string& body_name,
		const std::string& obj_file,
		const dart::dynamics::SkeletonPtr& target_skel,
		dart::dynamics::BodyNode* const parent,
		const Eigen::Vector3d& size,
		const Eigen::Isometry3d& joint_position,
		const Eigen::Isometry3d& body_position,
		double mass,
		bool contact)
{
	ShapePtr shape = std::shared_ptr<BoxShape>(new BoxShape(size));

	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	BodyNode* bn;
	PlanarJoint::Properties props;
	props.mName = body_name;
	props.mPlaneType = detail::PlaneType::ZX;
	// props.mT_ChildBodyToJoint = joint_position;
	props.mT_ParentBodyToJoint = body_position;


	bn = target_skel->createJointAndBodyNodePair<PlanarJoint>(
		parent,props,BodyNode::AspectProperties(body_name)).second;

	if(contact)
		bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	else
		bn->createShapeNodeWith<VisualAspect, DynamicsAspect>(shape);
	if(obj_file!="None")
	{
		std::string obj_path = std::string(MSS_ROOT_DIR)+"/model/OBJ/"+obj_file;
		const aiScene* mesh = MeshShape::loadMesh(std::string(obj_path));
		
		ShapePtr visual_shape = std::shared_ptr<MeshShape>(new MeshShape(Eigen::Vector3d(1.00,1.00,1.00),mesh));
		//bn = target_skel->getRootBodyNode();
		auto vsn = bn->createShapeNodeWith<VisualAspect>(visual_shape);
		Eigen::Isometry3d T_visual;
		T_visual.setIdentity();
		// T_visual = body_position.inverse();
		vsn->setRelativeTransform(T_visual);
	}
	bn->setInertia(inertia);
	// bn->setRestitutionCoeff(0.0);
	return bn;
}
BodyNode* SkeletonBuilder::MakeBallJointBody(
	const std::string& body_name,
	const std::string& obj_file,
	const dart::dynamics::SkeletonPtr& target_skel,
	dart::dynamics::BodyNode* const parent,
	const Eigen::Vector3d& size,
	const Eigen::Isometry3d& joint_position,
	const Eigen::Isometry3d& body_position,
	bool isLimitEnforced,
	const Eigen::Vector3d& upper_limit,
	const Eigen::Vector3d& lower_limit,
	double mass,
	bool contact)
{
	ShapePtr shape;
	if (body_name.find("wheel")!=std::string::npos)
		shape = std::shared_ptr<CylinderShape>(new CylinderShape(size[1],size[0]));
	else
		shape = std::shared_ptr<BoxShape>(new BoxShape(size));

	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	BodyNode* bn;
	BallJoint::Properties props;
	props.mName = body_name;
	if(parent!=nullptr)
		props.mT_ParentBodyToJoint = parent->getTransform().inverse()*joint_position;
	props.mT_ChildBodyToJoint = body_position.inverse()*joint_position;
	// props.mDampingCoefficients = Eigen::Vector3d(0.1,0.1,0.1);
	// std::cout<<props.mT_ChildBodyToJoint.translation().transpose()<<std::endl;
	// std::cout<<props.mT_ChildBodyToJoint.linear()<<std::endl;
	// std::cout<<props.mT_ChildBodyToJoint.linear().determinant()<<std::endl;
	// std::cout<<dart::math::verifyTransform(props.mT_ChildBodyToJoint)<<std::endl;

	bn = target_skel->createJointAndBodyNodePair<BallJoint>(
		parent,props,BodyNode::AspectProperties(body_name)).second;
	
	if(contact)
		bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	else
		bn->createShapeNodeWith<VisualAspect, DynamicsAspect>(shape);
	if(obj_file!="None")
	{
		std::string obj_path = std::string(MSS_ROOT_DIR)+"/model/OBJ/"+obj_file;
		const aiScene* mesh = MeshShape::loadMesh(std::string(obj_path));

		ShapePtr visual_shape = std::shared_ptr<MeshShape>(new MeshShape(Eigen::Vector3d(1.00,1.00,1.00),mesh));
		
		//bn = target_skel->getRootBodyNode();
		auto vsn = bn->createShapeNodeWith<VisualAspect>(visual_shape);
		Eigen::Isometry3d T_visual;
		T_visual.setIdentity();
		// T_visual = body_position.inverse();
		vsn->setRelativeTransform(T_visual);
	}
	bn->setInertia(inertia);
	// bn->setRestitutionCoeff(0.0);
	if(isLimitEnforced){
		JointPtr joint = bn->getParentJoint();
		joint->setPositionLimitEnforced(isLimitEnforced);
		for(int i = 0; i < 3; i++)
		{
			joint->setPositionUpperLimit(i, upper_limit[i]);
			joint->setPositionLowerLimit(i, lower_limit[i]);
			if(body_name.find("wheel")==std::string::npos)
				joint->setDampingCoefficient(i,0.4);
		}
	}
	else
	{
		JointPtr joint = bn->getParentJoint();
		for(int i = 0; i < 3; i++)
		{
			joint->setPositionUpperLimit(i,1E3);
			joint->setPositionLowerLimit(i,-1E3);
			if(body_name.find("wheel")==std::string::npos)
				joint->setDampingCoefficient(i,0.4);
		}

	}
	return bn;
}

BodyNode* SkeletonBuilder::MakeRevoluteJointBody(
	const std::string& body_name,
	const std::string& obj_file,
	const dart::dynamics::SkeletonPtr& target_skel,
	dart::dynamics::BodyNode* const parent,
	const Eigen::Vector3d& size,
	const Eigen::Isometry3d& joint_position,
	const Eigen::Isometry3d& body_position,
	bool isLimitEnforced,
	double upper_limit,
	double lower_limit,
	double mass,
	const Eigen::Vector3d& axis,
	bool contact)
{
	ShapePtr shape = std::shared_ptr<BoxShape>(new BoxShape(size));

	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	BodyNode* bn;
	RevoluteJoint::Properties props;
	props.mName = body_name;
	props.mAxis = axis;
	// props.mDampingCoefficients = Eigen::Matrix<double,1,1>(0.1);

	if(parent!=nullptr)
		props.mT_ParentBodyToJoint = parent->getTransform().inverse()*joint_position;
	props.mT_ChildBodyToJoint = body_position.inverse()*joint_position;

	bn = target_skel->createJointAndBodyNodePair<RevoluteJoint>(
		parent,props,BodyNode::AspectProperties(body_name)).second;
	
	if(contact)
		bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	else
		bn->createShapeNodeWith<VisualAspect, DynamicsAspect>(shape);
	if(obj_file!="None")
	{
		std::string obj_path = std::string(MSS_ROOT_DIR)+"/model/OBJ/"+obj_file;
		const aiScene* mesh = MeshShape::loadMesh(std::string(obj_path));
		ShapePtr visual_shape = std::shared_ptr<MeshShape>(new MeshShape(Eigen::Vector3d(1.00,1.00,1.00),mesh));
		//bn = target_skel->getRootBodyNode();
		auto vsn = bn->createShapeNodeWith<VisualAspect>(visual_shape);
		Eigen::Isometry3d T_visual;
		T_visual.setIdentity();
		// T_visual = body_position.inverse();
		vsn->setRelativeTransform(T_visual);
	}
	bn->setInertia(inertia);
	// bn->setRestitutionCoeff(0.0);
	if(isLimitEnforced){
		JointPtr joint = bn->getParentJoint();
		joint->setPositionLimitEnforced(isLimitEnforced);
		joint->setPositionUpperLimit(0, upper_limit);
		joint->setPositionLowerLimit(0, lower_limit);
		joint->setDampingCoefficient(0,0.4);
	}
	else
	{
		JointPtr joint = bn->getParentJoint();
		joint->setPositionUpperLimit(0, 1E3);
		joint->setPositionLowerLimit(0, -1E3);
		joint->setDampingCoefficient(0,0.4);
	}

	return bn;
}
BodyNode* SkeletonBuilder::MakePrismaticJointBody(
	const std::string& body_name,
	const std::string& obj_file,
	const dart::dynamics::SkeletonPtr& target_skel,
	dart::dynamics::BodyNode* const parent,
	const Eigen::Vector3d& size,
	const Eigen::Isometry3d& joint_position,
	const Eigen::Isometry3d& body_position,
	bool isLimitEnforced,
	double upper_limit,
	double lower_limit,
	double mass,
	const Eigen::Vector3d& axis,
	bool contact)
{
	ShapePtr shape = std::shared_ptr<BoxShape>(new BoxShape(size));

	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	BodyNode* bn;
	PrismaticJoint::Properties props;
	props.mName = body_name;
	props.mAxis = axis;

	if(parent!=nullptr)
		props.mT_ParentBodyToJoint = parent->getTransform().inverse()*joint_position;
	props.mT_ChildBodyToJoint = body_position.inverse()*joint_position;

	bn = target_skel->createJointAndBodyNodePair<PrismaticJoint>(
		parent,props,BodyNode::AspectProperties(body_name)).second;
	
	if(contact)
		bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	else
		bn->createShapeNodeWith<VisualAspect, DynamicsAspect>(shape);
	if(obj_file!="None")
	{
		std::string obj_path = std::string(MSS_ROOT_DIR)+"/model/OBJ/"+obj_file;
		const aiScene* mesh = MeshShape::loadMesh(std::string(obj_path));
		ShapePtr visual_shape = std::shared_ptr<MeshShape>(new MeshShape(Eigen::Vector3d(1.00,1.00,1.00),mesh));
		//bn = target_skel->getRootBodyNode();
		auto vsn = bn->createShapeNodeWith<VisualAspect>(visual_shape);
		Eigen::Isometry3d T_visual;
		T_visual.setIdentity();
		// T_visual = body_position.inverse();
		vsn->setRelativeTransform(T_visual);
	}
	bn->setInertia(inertia);

	if(isLimitEnforced){
		JointPtr joint = bn->getParentJoint();
		joint->setPositionLimitEnforced(isLimitEnforced);
		joint->setPositionUpperLimit(0, upper_limit);
		joint->setPositionLowerLimit(0, lower_limit);
	}

	return bn;
}
BodyNode* SkeletonBuilder::MakeWeldJointBody(
	const std::string& body_name,
	const std::string& obj_file,
	const dart::dynamics::SkeletonPtr& target_skel,
	dart::dynamics::BodyNode* const parent,
	const Eigen::Vector3d& size,
	const Eigen::Isometry3d& joint_position,
	const Eigen::Isometry3d& body_position,
	double mass,
	bool contact)
{
	ShapePtr shape = std::shared_ptr<BoxShape>(new BoxShape(size));

	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	BodyNode* bn;
	WeldJoint::Properties props;
	props.mName = body_name;
	
	if(parent!=nullptr)
		props.mT_ParentBodyToJoint = parent->getTransform().inverse()*joint_position;
	props.mT_ChildBodyToJoint = body_position.inverse()*joint_position;

	bn = target_skel->createJointAndBodyNodePair<WeldJoint>(
		parent,props,BodyNode::AspectProperties(body_name)).second;
	
	if(contact)
		bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	else
		bn->createShapeNodeWith<VisualAspect, DynamicsAspect>(shape);
	if(obj_file!="None")
	{
		std::string obj_path = std::string(MSS_ROOT_DIR)+"/model/OBJ/"+obj_file;
		const aiScene* mesh = MeshShape::loadMesh(std::string(obj_path));
		ShapePtr visual_shape = std::shared_ptr<MeshShape>(new MeshShape(Eigen::Vector3d(1.00,1.00,1.00),mesh));
		//bn = target_skel->getRootBodyNode();
		auto vsn = bn->createShapeNodeWith<VisualAspect>(visual_shape);
		Eigen::Isometry3d T_visual;
		T_visual.setIdentity();
		// T_visual = body_position.inverse();
		vsn->setRelativeTransform(T_visual);
	}
	bn->setInertia(inertia);
	// bn->setRestitutionCoeff(0.0);
	return bn;
}
std::vector<double> split_to_double(const std::string& input, int num)
{
    std::vector<double> result;
    std::string::size_type sz = 0, nsz = 0;
    for(int i = 0; i < num; i++){
        result.push_back(std::stof(input.substr(sz), &nsz));
        sz += nsz;
    }
    return result;
}

Eigen::Vector3d string_to_vector3d(const std::string& input){
	std::vector<double> v = split_to_double(input, 3);
	Eigen::Vector3d res;
	res << v[0], v[1], v[2];

	return res;
}

Eigen::VectorXd string_to_vectorXd(const std::string& input, int n){
	std::vector<double> v = split_to_double(input, n);
	Eigen::VectorXd res(n);
	for(int i = 0; i < n; i++){
		res[i] = v[i];
	}

	return res;
}
Eigen::Matrix3d string_to_matrix3d(const std::string& input){
	std::vector<double> v = split_to_double(input, 9);
	Eigen::Matrix3d res;
	res << v[0], v[1], v[2],
			v[3], v[4], v[5],
			v[6], v[7], v[8];

	return res;
}
}