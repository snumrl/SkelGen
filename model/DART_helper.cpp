#include "DART_helper.h"
#include <stdio.h>

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace std;

BodyNode* MakeFreeJointBody(
		const std::string& body_name,
		const std::string& obj_file,
		const dart::dynamics::SkeletonPtr& target_skel,
		dart::dynamics::BodyNode* const parent,
		const Eigen::Vector3d& size,
		const Eigen::Isometry3d& joint_position,
		const Eigen::Isometry3d& body_position,
		double mass,
		bool contact,
		bool obj_visual_consensus)
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
		if (obj_visual_consensus){
			std::string obj_path = string(MSS_ROOT_DIR)+"/model/standard/skeleton/"+obj_file;
			const aiScene* mesh = MeshShape::loadMesh(std::string(obj_path));
			ShapePtr visual_shape = std::shared_ptr<MeshShape>(new MeshShape(Eigen::Vector3d(1,1,1),mesh));
			bn->createShapeNodeWith<VisualAspect>(visual_shape);
		}else{
			std::string obj_path = string(MSS_ROOT_DIR)+"/model/standard/OBJ_reduced/"+obj_file;
			const aiScene* mesh = MeshShape::loadMesh(std::string(obj_path));
			ShapePtr visual_shape = std::shared_ptr<MeshShape>(new MeshShape(Eigen::Vector3d(0.01,0.01,0.01),mesh));
			auto vsn = bn->createShapeNodeWith<VisualAspect>(visual_shape);
			Eigen::Isometry3d T_visual;
			T_visual.setIdentity();
			T_visual = body_position.inverse();
			vsn->setRelativeTransform(T_visual);
		}
	}
	bn->setInertia(inertia);
	bn->getTransform();
	// bn->setRestitutionCoeff(0.0);
	return bn;
}

BodyNode* MakeBallJointBody(
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
		bool contact,
		bool obj_visual_consensus)
{
	ShapePtr shape = std::shared_ptr<BoxShape>(new BoxShape(size));

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
		if (obj_visual_consensus){
			std::string obj_path = string(MSS_ROOT_DIR)+"/model/standard/skeleton/"+obj_file;
			const aiScene* mesh = MeshShape::loadMesh(std::string(obj_path));
			ShapePtr visual_shape = std::shared_ptr<MeshShape>(new MeshShape(Eigen::Vector3d(1,1,1),mesh));
			bn->createShapeNodeWith<VisualAspect>(visual_shape);
		}else{
			std::string obj_path = string(MSS_ROOT_DIR)+"/model/standard/OBJ_reduced/"+obj_file;
			const aiScene* mesh = MeshShape::loadMesh(std::string(obj_path));
			ShapePtr visual_shape = std::shared_ptr<MeshShape>(new MeshShape(Eigen::Vector3d(0.01,0.01,0.01),mesh));
			auto vsn = bn->createShapeNodeWith<VisualAspect>(visual_shape);
			Eigen::Isometry3d T_visual;
			T_visual.setIdentity();
			T_visual = body_position.inverse();
			vsn->setRelativeTransform(T_visual);
		}
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
		}
	}
	else
	{
		JointPtr joint = bn->getParentJoint();
		for(int i = 0; i < 3; i++)
		{
			joint->setPositionUpperLimit(i,1E3);
			joint->setPositionLowerLimit(i,-1E3);
		}
	}
	return bn;
}

BodyNode* MakeRevoluteJointBody(
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
		bool contact,
		bool obj_visual_consensus)
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
		if (obj_visual_consensus){
			std::string obj_path = string(MSS_ROOT_DIR)+"/model/standard/skeleton/"+obj_file;
			const aiScene* mesh = MeshShape::loadMesh(std::string(obj_path));
			ShapePtr visual_shape = std::shared_ptr<MeshShape>(new MeshShape(Eigen::Vector3d(1,1,1),mesh));
			bn->createShapeNodeWith<VisualAspect>(visual_shape);
		}else{
			std::string obj_path = string(MSS_ROOT_DIR)+"/model/standard/OBJ_reduced/"+obj_file;
			const aiScene* mesh = MeshShape::loadMesh(std::string(obj_path));
			ShapePtr visual_shape = std::shared_ptr<MeshShape>(new MeshShape(Eigen::Vector3d(0.01,0.01,0.01),mesh));
			auto vsn = bn->createShapeNodeWith<VisualAspect>(visual_shape);
			Eigen::Isometry3d T_visual;
			T_visual.setIdentity();
			T_visual = body_position.inverse();
			vsn->setRelativeTransform(T_visual);
		}
	}
	bn->setInertia(inertia);
	// bn->setRestitutionCoeff(0.0);
	if(isLimitEnforced){
		JointPtr joint = bn->getParentJoint();
		joint->setPositionLimitEnforced(isLimitEnforced);
		joint->setPositionUpperLimit(0, upper_limit);
		joint->setPositionLowerLimit(0, lower_limit);
	}
	else
	{
		JointPtr joint = bn->getParentJoint();
		joint->setPositionUpperLimit(0, 1E3);
		joint->setPositionLowerLimit(0, -1E3);
	}

	return bn;
}
BodyNode* MakePrismaticJointBody(
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
		bool contact,
		bool obj_visual_consensus)
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
		if (obj_visual_consensus){
			std::string obj_path = string(MSS_ROOT_DIR)+"/model/standard/skeleton/"+obj_file;
			const aiScene* mesh = MeshShape::loadMesh(std::string(obj_path));
			ShapePtr visual_shape = std::shared_ptr<MeshShape>(new MeshShape(Eigen::Vector3d(1,1,1),mesh));
			bn->createShapeNodeWith<VisualAspect>(visual_shape);
		}else{
			std::string obj_path = string(MSS_ROOT_DIR)+"/model/standard/OBJ_reduced/"+obj_file;
			const aiScene* mesh = MeshShape::loadMesh(std::string(obj_path));
			ShapePtr visual_shape = std::shared_ptr<MeshShape>(new MeshShape(Eigen::Vector3d(0.01,0.01,0.01),mesh));
			auto vsn = bn->createShapeNodeWith<VisualAspect>(visual_shape);
			Eigen::Isometry3d T_visual;
			T_visual.setIdentity();
			T_visual = body_position.inverse();
			vsn->setRelativeTransform(T_visual);
		}
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
BodyNode* MakeWeldJointBody(
		const std::string& body_name,
		const std::string& obj_file,
		const dart::dynamics::SkeletonPtr& target_skel,
		dart::dynamics::BodyNode* const parent,
		const Eigen::Vector3d& size,
		const Eigen::Isometry3d& joint_position,
		const Eigen::Isometry3d& body_position,
		double mass,
		bool contact,
		bool obj_visual_consensus)
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
	else
		props.mT_ParentBodyToJoint = joint_position;
	props.mT_ChildBodyToJoint = body_position.inverse()*joint_position;

	bn = target_skel->createJointAndBodyNodePair<WeldJoint>(
			parent,props,BodyNode::AspectProperties(body_name)).second;

	if(contact)
		bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	else
		bn->createShapeNodeWith<VisualAspect, DynamicsAspect>(shape);
	if(obj_file!="None")
	{
		if (obj_visual_consensus){
			std::string obj_path = string(MSS_ROOT_DIR)+"/model/standard/skeleton/"+obj_file;
			const aiScene* mesh = MeshShape::loadMesh(std::string(obj_path));
			ShapePtr visual_shape = std::shared_ptr<MeshShape>(new MeshShape(Eigen::Vector3d(1,1,1),mesh));
			bn->createShapeNodeWith<VisualAspect>(visual_shape);
		}else{
			std::string obj_path = string(MSS_ROOT_DIR)+"/model/standard/OBJ_reduced/"+obj_file;
			const aiScene* mesh = MeshShape::loadMesh(std::string(obj_path));
			ShapePtr visual_shape = std::shared_ptr<MeshShape>(new MeshShape(Eigen::Vector3d(0.01,0.01,0.01),mesh));
			auto vsn = bn->createShapeNodeWith<VisualAspect>(visual_shape);
			Eigen::Isometry3d T_visual;
			T_visual.setIdentity();
			T_visual = body_position.inverse();
			vsn->setRelativeTransform(T_visual);
		}
	}
	bn->setInertia(inertia);
	// bn->setRestitutionCoeff(0.0);
	return bn;
}