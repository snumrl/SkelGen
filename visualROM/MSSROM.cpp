#include "dart/dart.hpp"
#include "MSSROM.h"
#include <assert.h>
#include "../generator/motionAdjust/BVHparser.h"
#include <math.h>

using namespace std;
using namespace dart;
using namespace dart::dynamics;

double toStandardAngle(double angle)
{
	while(abs(angle)>M_PI)
    {
        angle -= 2* M_PI *  angle/abs(angle);
    }
    return angle;
}
BodyROM::BodyROM(std::string name, JOINT_TYPE type):name(name),type(type) {
	for (int i = 0; i <= numTheta; i++)
		for (int j = 0; j < numPhi; j++)
			for (int k = 0; k < numRoll; k++)
				data_ball[i][j][k] = 0;

	for (int i = 0; i <= numRev; i++)
		data_revolute[i] = 0;

	// if (name.find("Femur") != string::npos){
	// 	lowerLimit = Eigen::Vector3d(0, -100, -100);
	// 	upperLimit = Eigen::Vector3d(2.05, 100, 100);
	// }else if (name.find("Foot") != string::npos){
	// 	lowerLimit = Eigen::Vector3d(-100, -100, -100);
	// 	upperLimit = Eigen::Vector3d(100, 100, 100);
	// }else if (name.find("Arm") != string::npos){
	// 	lowerLimit = Eigen::Vector3d(0, -100, -100);
	// 	upperLimit = Eigen::Vector3d(2.0, 100, 100);
	// }else if (name.find("Hand") != string::npos){
	// 	lowerLimit = Eigen::Vector3d(0, -100, -100);
	// 	upperLimit = Eigen::Vector3d(2.0, 100, 100);
	// }else if (name.find("R_ForeArm") != string::npos){
	// 	lowerLimit = Eigen::Vector3d(0, -100, -100);
	// 	upperLimit = Eigen::Vector3d(M_PI, 100, 100);
	// }else if (name.find("L_ForeArm") != string::npos){
	// 	lowerLimit = Eigen::Vector3d(-M_PI, -100, -100);
	// 	upperLimit = Eigen::Vector3d(0, 100, 100);
	// }

	// else{
	// 	lowerLimit = Eigen::Vector3d(-100, -100, -100);
	// 	upperLimit = Eigen::Vector3d(100, 100, 100);
	// }
	// lowerLimit = Eigen::Vector3d(-100, -100, -100);
	// upperLimit = Eigen::Vector3d(100, 100, 100);
	conicTranslate = Eigen::AngleAxisd(0, Eigen::Vector3d(1.0,0.0,0.0));
	conicScale = 1.0;

	centerJ2B = Eigen::Vector3d(0,0,0);
}

Eigen::Vector3d BodyROM::conicEdit(Eigen::Vector3d vec) {
	Eigen::Vector3d center = prev_centerJ2B.normalized() * vec.norm();
	Eigen::Vector3d axis = center.cross(vec).normalized();
	double angle = atan2(center.cross(vec).norm(), center.dot(vec)) * conicScale;

	vec = Eigen::AngleAxisd(angle, axis) * center;

	return vec;
}
double BodyROM::revoluteEdit(double angle){
	double r = toStandardAngle(angle - revoluteEditInfo.prev_center);
	r *= revoluteEditInfo.scale;
	r += revoluteEditInfo.prev_center;
	r += revoluteEditInfo.translate;
	return r;
}

MSSROM::
MSSROM(std::shared_ptr<MusculoSkeletalSystem>& ms)
:ms(ms)
{
	edb = new EditInfoDB();
}

MSSROM::
MSSROM(std::shared_ptr<MusculoSkeletalSystem>& ms, EditInfoDB* edb)
:ms(ms), edb(edb)
{
}

// void MSSROM::setPose(std::string body_name, Eigen::Vector3d j2b, double roll)
// {
// 	dart::dynamics::BodyNodePtr body = ms->getSkeleton()->getBodyNode(body_name);
// 	int bodynodeIdx = 0;
// 	for (int i=0;i<ms->getSkeleton()->getNumBodyNodes();i++) if (ms->getSkeleton()->getBodyNode(i) == body) bodynodeIdx = i;

// 	// set joint position as t-pose
// 	BodyROM* body_rom = getROM(body_name);
// 	if (body_rom->type == JOINT_TYPE::BALL_AND_SOCKET) {
//         // Zero pose is identity rotation
        
//         Eigen::Matrix3d zeroToCurOrientation;

//         body->getParentJoint()->setPositions(Eigen::Vector3d::Zero());
//         Eigen::Isometry3d zeroParentJointT =
//             body->getTransform() * body->getParentJoint()->getTransformFromChildBodyNode();
//         // World Coord. desired conic axis(local coordinate to world coordinate)
//         Eigen::Vector3d world_j2b = j2b;

//         // Conic rotation
//         // World Coord. Joint to body vector for zero pose of the joint.
//         Eigen::Vector3d zero_j2b = ms->getJtoBAxis(body_name).normalized();
        
//         Eigen::Vector3d zero_conic_trans_axis = zero_j2b.cross(world_j2b);
//         double zero_conic_trans_angle = atan2(zero_conic_trans_axis.norm(), zero_j2b.dot(world_j2b));
//         zero_conic_trans_axis.normalize();

//         Eigen::Matrix3d zero_conic_rotation = Eigen::AngleAxisd(zero_conic_trans_angle, zero_conic_trans_axis).toRotationMatrix();

//         // Roll rotation
// 	 	Eigen::Matrix3d zero_roll_rotation = Eigen::AngleAxisd(roll, world_j2b).toRotationMatrix();

//         // Zero pose to center pose rotation.
//         zeroToCurOrientation = zero_roll_rotation * zero_conic_rotation;

//         // Since global zero pose Rotation is zeroParentJointT.linear(), we multply it at the last part
//         Eigen::Matrix3d local_zeroToCurOrientation = 
//         	zeroParentJointT.linear().inverse()*zeroToCurOrientation*zeroParentJointT.linear();

//         body->getParentJoint()->setPositions(2.0 * QuaternionToAngleAxis(Eigen::Quaterniond(local_zeroToCurOrientation)));

// 	}else if (body_rom->type == JOINT_TYPE::REVOLUTE){
// 		// restore whole positions except target joint
// 		cout<<"MSSROM::setPose conic."<<body_name<<"is not a ball-and-socket joint"<<endl;

// 	}

// }


void MSSROM::setPose(std::string body_name, double theta, double phi, double roll){
	dart::dynamics::BodyNodePtr body = ms->getSkeleton()->getBodyNode(body_name);
	int bodynodeIdx = 0;
	for (int i=0;i<ms->getSkeleton()->getNumBodyNodes();i++) if (ms->getSkeleton()->getBodyNode(i) == body) bodynodeIdx = i;

	// set joint position as t-pose
	// body->getParentJoint()->setPositions(Eigen::Vector3d::Zero());

	BodyROM* body_rom = getROM(body_name);

	if (body_rom->type == JOINT_TYPE::BALL_AND_SOCKET) {
		Eigen::Isometry3d parentJointT = body->getTransform() * body->getParentJoint()->getTransformFromChildBodyNode();

		Eigen::Vector3d j2b, v;

// 		j2b = parentJointT.linear().inverse() * ms->getJtoBAxis(body_name);
// 		v = parentJointT.linear().inverse() * ms->getVAxis(body_name);

// 		Eigen::Quaterniond conicQuat = makeConicQuaternion(theta, phi, v, j2b);
// 		Eigen::AngleAxisd conicAA(conicQuat);

// 		Eigen::AngleAxisd localConic(conicAA.toRotationMatrix());

// 		body->getParentJoint()->setPositions(localConic.angle() * localConic.axis());
// //		body->getParentJoint()->setPositions(conicAA.angle() * conicAA.axis());
// 		parentJointT = body->getTransform() * body->getParentJoint()->getTransformFromChildBodyNode();

// 		j2b = parentJointT.linear().inverse() * ms->getJtoBAxis(body_name);

// 		Eigen::Quaterniond rollQuat = makeAxialQuaternion(j2b.normalized(), roll);
// 		Eigen::AngleAxisd rollAA(rollQuat * conicQuat);

// 		Eigen::AngleAxisd localRoll(rollAA.toRotationMatrix());

// 		body->getParentJoint()->setPositions(localRoll.angle() * localRoll.axis());


//------------------------------------------

		j2b = parentJointT.linear().inverse() * ms->getJtoBAxis(body_name);
		v = parentJointT.linear().inverse() * ms->getVAxis(body_name);

		Eigen::Matrix3d rot = Eigen::AngleAxisd(phi, j2b).toRotationMatrix() * Eigen::AngleAxisd(theta, v).toRotationMatrix();
		Eigen::Vector3d roted_j2b = rot * j2b;
		rot = Eigen::AngleAxisd(roll, roted_j2b).toRotationMatrix() * rot;
		body->getParentJoint()->setPositions(2 * QuaternionToAngleAxis(Eigen::Quaterniond(rot)));
	}else if (body_rom->type == JOINT_TYPE::REVOLUTE){
		// restore whole positions except target joint
		body->getParentJoint()->setPosition(0,theta);


	}
}

// Eigen::Quaterniond MSSROM::getRotation(std::string body_name, double theta, double phi, double roll){
// 	dart::dynamics::BodyNodePtr body = ms->getSkeleton()->getBodyNode(body_name);
// 	int bodynodeIdx = 0;
// 	for (int i=0;i<ms->getSkeleton()->getNumBodyNodes();i++) if (ms->getSkeleton()->getBodyNode(i) == body) bodynodeIdx = i;

// 	// save current status
// 	double config[DOF];
// 	for (int i=0;i<DOF;i++) config[i] = ms->getSkeleton()->getPosition(i);

// 	// set whole positions as t-pose
// 	for (int i=0;i<DOF;i++) ms->getSkeleton()->setPosition(i,0);

// 	BodyROM* body_rom = getROM(body_name);

// 	assert(body_rom->type == JOINT_TYPE::BALL_AND_SOCKET);
// 	Eigen::Isometry3d parentJointT = body->getTransform() * body->getParentJoint()->getTransformFromChildBodyNode();

// 	Eigen::Vector3d j2b, v;

// 	j2b = ms->getJtoBAxis(body_name);
// 	v = ms->getVAxis(body_name);

// 	Eigen::Quaterniond conicQuat = makeConicQuaternion(theta, phi, v, j2b);
// 	Eigen::AngleAxisd conicAA(conicQuat);

// 	body->getParentJoint()->setPositions(conicAA.angle() * conicAA.axis());

// 	j2b = ms->getJtoBAxis(body_name);

// 	Eigen::Quaterniond rollQuat = makeAxialQuaternion(j2b.normalized(), roll);

// 	// restor whole positions except target joint
// 	for (int i=0;i<DOF;i++) ms->getSkeleton()->setPosition(i,config[i]);

// 	return rollQuat * conicQuat;

// }

void 
MSSROM::
InitROM()
{
	ifstream in("../visualROM/ROMJointList.txt");
	string line;
	stringstream s;
	string body_name;
	string joint_type;
	while(!in.eof())
	{
		getline(in, line);
		if(line.empty())
			break;
		s = stringstream(line);
		s >> body_name >>joint_type;
		if(joint_type == "Ball")
		{
		    BodyROMs.push_back(new BodyROM(body_name, JOINT_TYPE::BALL_AND_SOCKET));
		}
		else
		{
    		BodyROMs.push_back(new BodyROM(body_name, JOINT_TYPE::REVOLUTE));
		}
	}
    InitEDB();

}

void MSSROM::InitEDB()
{
	ComputeROMCenter("R_Femur");
	ComputeROMCenter("R_Tibia");
	ComputeROMCenter("R_Foot");
	ComputeROMCenter("R_Arm");
	ComputeROMCenter("R_ForeArm");
	ComputeROMCenter("R_Hand");
	
	ComputeROMCenter("L_Femur");
	ComputeROMCenter("L_Tibia");
	ComputeROMCenter("L_Foot");
	ComputeROMCenter("L_Arm");
	ComputeROMCenter("L_ForeArm");
	ComputeROMCenter("L_Hand");

}

bool IsBallJoint(std::string body_name)
{
	if(body_name == "R_Femur" || body_name == "L_Femur" ||
	   	body_name == "R_Foot" || body_name == "L_Foot" ||
		body_name == "R_Arm" || body_name == "L_Arm" ||
		body_name == "R_Hand" || body_name == "L_Hand" ||
		body_name == "Spine" || body_name == "Head" ||
		body_name == "Torso")
		return true;
	else if(body_name == "R_Tibia" || body_name == "L_Tibia" ||
		body_name == "Neck" || 
		body_name == "R_Shoulder" || body_name == "L_Shoulder" ||
		body_name == "R_ForeArm" || body_name == "L_ForeArm")
		return false;
	else
	{
		cout<<"No such joint : "<<body_name<<endl;
		return false;
	}
}
bool IsMajorBallJoint(std::string body_name)
{
	if(body_name == "R_Femur" || body_name == "L_Femur" ||
	   	body_name == "R_Foot" || body_name == "L_Foot" ||
		body_name == "R_Arm" || body_name == "L_Arm" ||
		body_name == "R_Hand" || body_name == "L_Hand" )
	{
		return true;

	}
	else
	{
		return false;
	}
}
bool IsMajorRevoluteJoint(std::string body_name)
{
	if(body_name == "R_Tibia" || body_name == "L_Tibia" ||
	   	body_name == "R_ForeArm" || body_name == "L_ForeArm")
		return true;
	else
	{
		return false;
	}
}
BodyROM* 
MSSROM::
getROM(std::string body_name)
{
	for(auto& bodyrom : BodyROMs)
	{
		if(bodyrom->name == body_name){
			if (body_name == "R_Tibia"){
			}
			return bodyrom;
		}
	}
	cout<<"No such rom body name "<<body_name<<endl;
	return BodyROMs[0];
}

// // Ouput is global j2b
// Eigen::Vector3d MSSROM::getJ2BfromThetaPhi(std::string body_name, double theta, double phi)
// {
// 	Eigen::Isometry3d parentJointT = body->getTransform() * body->getParentJoint()->getTransformFromChildBodyNode();
// 	Eigen::AngleAxisd(phi, j2b).toRotationMatrix() * Eigen::AngleAxisd(theta, v).toRotationMatrix();
// }


void MSSROM::LoadROM(std::string body_name)
{
	BodyNodePtr body = ms->getSkeleton()->getBodyNode(body_name);
	BodyROM* body_rom = getROM(body_name);
	int objID;
	if(body_name == "R_Femur")
		objID = ID_R_Femur;
	else if (body_name == "R_Tibia")
		objID = ID_R_Tibia;
	else if(body_name == "R_Foot")
		objID = ID_R_Foot;
	else if(body_name == "L_Femur")
		objID = ID_L_Femur;
	else if (body_name == "L_Tibia")
		objID = ID_L_Tibia;
	else if(body_name == "L_Foot")
		objID = ID_L_Foot;
    else if(body_name == "Spine")
		objID = ID_Spine;
    else if(body_name == "Torso")
		objID = ID_Torso;
    else if(body_name == "Neck")
		objID = ID_Neck;
    else if(body_name == "Head")
		objID = ID_Head;
    else if(body_name == "L_Shoulder")
		objID = ID_L_Shoulder;
    else if(body_name == "L_Arm")
		objID = ID_L_Arm;
    else if(body_name == "L_ForeArm")
		objID = ID_L_ForeArm;
    else if(body_name == "L_Hand")
		objID = ID_L_Hand;
    else if(body_name == "R_Shoulder")
		objID = ID_R_Shoulder;
    else if(body_name == "R_Arm")
		objID = ID_R_Arm;
    else if(body_name == "R_ForeArm")
		objID = ID_R_ForeArm;
    else if(body_name == "R_Hand")
		objID = ID_R_Hand;
    else{
    	cout << body_name << " : NOTHING!!!!!!!!!!!!!!!!" << endl;
    }



	if(body_rom->type == JOINT_TYPE::BALL_AND_SOCKET)
	{
		glPushName(objID);

		// save current joint position
		Eigen::Vector3d originPos = body->getParentJoint()->getPositions();

		// set joint position as t-pose
		body->getParentJoint()->setPositions(Eigen::Vector3d::Zero());


		Eigen::Vector3d color;
		if(body_name.substr(1) == "_Femur")
			color = Eigen::Vector3d(0.0, 0.0, 1.0);
		else
			color = Eigen::Vector3d(0.0, 1.0, 0.0);


		double radius = 0.15;
		Eigen::Isometry3d zeroParentJointT = body->getTransform() * body->getParentJoint()->getTransformFromChildBodyNode();
		Eigen::Vector3d joint_to_body = body->getTransform().translation() - zeroParentJointT.translation();
		joint_to_body = joint_to_body / joint_to_body.norm();
		Eigen::Vector3d jointPos = zeroParentJointT.translation();

		// draw sphere guideline
		glDisable(GL_LIGHT0);
		glDisable(GL_LIGHT1);
		glLineWidth(0.05);
		for (int i=0;i<120;i++){
			glColor3f(0,0,0);
			double theta = 1.0 * i / 120 * 2 * M_PI;
			double theta2 = 1.0 * (i+1) / 120 * 2 * M_PI;
			glPushMatrix();
			Eigen::Vector3d p1 = Eigen::AngleAxisd(theta, Eigen::Vector3d(1,0,0)) * Eigen::Vector3d(0, radius*1.005, 0) + jointPos;
			Eigen::Vector3d p2 = Eigen::AngleAxisd(theta2, Eigen::Vector3d(1,0,0)) * Eigen::Vector3d(0, radius*1.005, 0) + jointPos;
			GUI::DrawLine(p1, p2);
//			GUI::DrawCube(Eigen::Vector3d(0.001963495, 0.001963495, 0.001963495));
			glPopMatrix();

		}

		for (int i=0;i<120;i++){
			glColor3f(0,0,0);
			double theta = 1.0 * i / 120 * 2 * M_PI;
			double theta2 = 1.0 * (i+1) / 120 * 2 * M_PI;
			glPushMatrix();
			Eigen::Vector3d p1 = Eigen::AngleAxisd(theta, Eigen::Vector3d(0,1,0)) * Eigen::Vector3d(radius*1.005, 0, 0) + jointPos;
			Eigen::Vector3d p2 = Eigen::AngleAxisd(theta2, Eigen::Vector3d(0,1,0)) * Eigen::Vector3d(radius*1.005, 0, 0) + jointPos;
			GUI::DrawLine(p1, p2);
			glPopMatrix();

		}

		for (int i=0;i<120;i++){
			glColor3f(0,0,0);
			double theta = 1.0 * i / 120 * 2 * M_PI;
			double theta2 = 1.0 * (i+1) / 120 * 2 * M_PI;
			glPushMatrix();
//			glTranslated(jointPos[0], jointPos[1], jointPos[2]);
//			glRotated(theta * 180.0 / M_PI, 1, 0, 0);
//			glTranslated(0, radius, 0);
			Eigen::Vector3d p1 = Eigen::AngleAxisd(theta, Eigen::Vector3d(0,0,1)) * Eigen::Vector3d(0, radius*1.005, 0) + jointPos;
			Eigen::Vector3d p2 = Eigen::AngleAxisd(theta2, Eigen::Vector3d(0,0,1)) * Eigen::Vector3d(0, radius*1.005, 0) + jointPos;
			GUI::DrawLine(p1, p2);
//			GUI::DrawCube(Eigen::Vector3d(0.001963495, 0.001963495, 0.001963495));
			glPopMatrix();

		}

		double theta = 0.0, phi = 0.0, roll = -M_PI;
		double stepTheta = M_PI / numTheta, stepPhi = 2 * M_PI / numPhi, stepRoll = 2 * M_PI / numRoll;
		for (int i = 0; i < numTheta; i++) {
			for (int j = 0; j < numPhi; j++) {
				int cnt = 0;
				for (int k = 0; k < numRoll; k++) {
					theta = stepTheta * i;
					phi = stepPhi * j;
					roll = stepRoll * k;

					setPose(body_name, theta, phi, roll);

					Eigen::Vector3d joint_to_body = body->getTransform().translation() - zeroParentJointT.translation();
					joint_to_body = joint_to_body / joint_to_body.norm() * radius;
					Eigen::Vector3d vec = joint_to_body;
					//					cout << "HI" << endl;

					if(body_rom->data_ball[i][j][k]) {
						//						Eigen::Vector3d p = jointPos + 1.0 * k / numRoll * vec;
						//						Eigen::Vector3d q = jointPos + 1.0 * (k + 1) / numRoll * vec;
						//						GUI::DrawLine(p, q, color);
						cnt++;
					}
				}
				if (cnt>0) {
					glPushMatrix();
					glTranslated(jointPos[0], jointPos[1], jointPos[2]);
					double ratio = 1.0 - 1.0 * cnt / numRoll;
					ratio = pow(ratio, 2);

					color = (1 - ratio) * Eigen::Vector3d(1.0, 0, 0) + ratio * Eigen::Vector3d(1.0, 1.0, 0.0);

					GLfloat mat_amb_diff[] = { GLfloat(0.8*color[0]), GLfloat(0.8*color[1]), GLfloat(0.8*color[2]), 1.0};
					GLfloat mat_spec[] = { 0, 0, 0, 0.8};
					glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,
								 mat_amb_diff);
					/*glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE,
								 mat_amb_diff);*/
//					glMaterialf( GL_FRONT, GL_SHININESS, 0.0f );
					glColor4f(color[0],color[1],color[2], 1.0);
/*					const int numTheta = 18;
					const int numPhi = 36;
					const int numRoll = 18;*/
					double radius = 0.15;
					double theta = 0.0, phi = 0.0, roll = -M_PI;
					double stepTheta = M_PI / numTheta, stepPhi = 2 * M_PI / numPhi, stepRoll = 2 * M_PI / numRoll;

					int offset = 10;
					double vTheta, vPhi, hRoll;
					hRoll = radius / numRoll;
					vPhi = radius / numRoll * (offset) * stepTheta;
					vTheta = radius / numRoll * (offset) * stepPhi;


					Eigen::Matrix3d userRotation;

					Eigen::Vector3d world_prev_centerJ2B = zeroParentJointT.linear() * body_rom->prev_centerJ2B.normalized();
					Eigen::Vector3d world_cur_centerJ2B = zeroParentJointT.linear() * body_rom->cur_centerJ2B.normalized();

					Eigen::Vector3d user_rot_axis = world_prev_centerJ2B.cross(world_cur_centerJ2B);
					double user_rot_angle
						= atan2(user_rot_axis.norm(), world_prev_centerJ2B.dot(world_cur_centerJ2B));
					user_rot_axis.normalize();
					if(user_rot_angle < 1e-6)
					{
						user_rot_angle=0.0;
						user_rot_axis = Eigen::Vector3d::UnitX();
					}
					user_rot_axis.normalize();

					userRotation = Eigen::AngleAxisd(body_rom->cur_roll, world_cur_centerJ2B)
						* Eigen::AngleAxisd(user_rot_angle, user_rot_axis).toRotationMatrix();
					// userRotation = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX()).toRotationMatrix();

					// if(body_name.find("Foot")!=string::npos)
					// {
					// 	userRotation = userRotation * Eigen::AngleAxisd(-M_PI/4, Eigen::Vector3d::UnitX());
					// }

					theta = stepTheta * i;
					phi = stepPhi * j;

					vector<Eigen::Vector3d> points;

					setPose(body_name, theta - stepTheta / 2, phi - stepPhi / 2, 0);
					joint_to_body = body->getTransform().translation() - zeroParentJointT.translation();
					joint_to_body = joint_to_body / joint_to_body.norm() * radius;
					joint_to_body = body_rom->conicEdit(joint_to_body);
					points.emplace_back(userRotation * joint_to_body);

					setPose(body_name, theta - stepTheta / 2, phi + stepPhi / 2, 0);
					joint_to_body = body->getTransform().translation() - zeroParentJointT.translation();
					joint_to_body = joint_to_body / joint_to_body.norm() * radius;
					joint_to_body = body_rom->conicEdit(joint_to_body);
					points.emplace_back(userRotation * joint_to_body);

					setPose(body_name, theta + stepTheta / 2, phi + stepPhi / 2, 0);
					joint_to_body = body->getTransform().translation() - zeroParentJointT.translation();
					joint_to_body = joint_to_body / joint_to_body.norm() * radius;
					joint_to_body = body_rom->conicEdit(joint_to_body);
					points.emplace_back(userRotation * joint_to_body);

					setPose(body_name, theta + stepTheta / 2, phi - stepPhi / 2, 0);
					joint_to_body = body->getTransform().translation() - zeroParentJointT.translation();
					joint_to_body = joint_to_body / joint_to_body.norm() * radius;
					joint_to_body = body_rom->conicEdit(joint_to_body);
					points.emplace_back(userRotation * joint_to_body);

					for (int i=0;i<4;i++) points.emplace_back(points[i]*0.95);

					Eigen::Vector3d n ;

					glDisable(GL_LIGHT0);
					glDisable(GL_LIGHT1);


					glBegin(GL_QUADS);
					n = (points[0]-points[1]).cross(points[2]-points[1]);
					glNormal3f(n[0],n[1],n[2]);
					for (int i=0;i<4;i++){
						glVertex3f(points[i][0],points[i][1],points[i][2]);
					}
					glEnd();
					glColor3f(0.1, 0.1, 0.1);
					glBegin(GL_QUADS);
					n = -(points[4]-points[5]).cross(points[6]-points[5]);
					glNormal3f(n[0],n[1],n[2]);
					for (int i=4;i<8;i++){
						glVertex3f(points[i][0],points[i][1],points[i][2]);
					}
					glEnd();


					glColor3f(0.2, 0.2, 0.2);
					glBegin(GL_QUADS);
					n = -(points[0]-points[1]).cross(points[5]-points[1]);
					glNormal3f(n[0],n[1],n[2]);
					glVertex3f(points[0][0],points[0][1],points[0][2]);
					glVertex3f(points[1][0],points[1][1],points[1][2]);
					glVertex3f(points[5][0],points[5][1],points[5][2]);
					glVertex3f(points[4][0],points[4][1],points[4][2]);
					glEnd();

					glBegin(GL_QUADS);
					n = -(points[1]-points[2]).cross(points[6]-points[2]);
					glNormal3f(n[0],n[1],n[2]);
					glVertex3f(points[1][0],points[1][1],points[1][2]);
					glVertex3f(points[2][0],points[2][1],points[2][2]);
					glVertex3f(points[6][0],points[6][1],points[6][2]);
					glVertex3f(points[5][0],points[5][1],points[5][2]);
					glEnd();

					glBegin(GL_QUADS);
					n = -(points[2]-points[3]).cross(points[7]-points[3]);
					glNormal3f(n[0],n[1],n[2]);
					glVertex3f(points[2][0],points[2][1],points[2][2]);
					glVertex3f(points[3][0],points[3][1],points[3][2]);
					glVertex3f(points[7][0],points[7][1],points[7][2]);
					glVertex3f(points[6][0],points[6][1],points[6][2]);
					glEnd();

					glBegin(GL_QUADS);
					n = -(points[3]-points[0]).cross(points[0]-points[4]);
					glNormal3f(n[0],n[1],n[2]);
					glVertex3f(points[3][0],points[3][1],points[3][2]);
					glVertex3f(points[0][0],points[0][1],points[0][2]);
					glVertex3f(points[4][0],points[4][1],points[4][2]);
					glVertex3f(points[7][0],points[7][1],points[7][2]);
					glEnd();

					//
					//					setPose(body_name, theta, phi, 0);
					//					joint_to_body = body->getTransform().translation() - parentJointT.translation();
					//					joint_to_body = joint_to_body / joint_to_body.norm() * radius;
					//					glTranslated(joint_to_body[0],joint_to_body[1],joint_to_body[2]);
					//					GUI::DrawSphere(0.01);

					glPopMatrix();
					GLfloat no_mat[] = {0.0, 0.0, 0.0, 1.0};
					glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, no_mat);

				}
			}
		}


		// Draw centerJ2B
		Eigen::Vector3d p = jointPos;
		Eigen::Vector3d q = jointPos + body_rom->cur_centerJ2B.normalized() * radius * 1.2;
		GUI::DrawLine(p, q, Eigen::Vector3d(1,0,0));

		// restore joint position
		body->getParentJoint()->setPositions(originPos);
		glPopName();;
	}
	else if(body_rom->type == JOINT_TYPE::REVOLUTE)
	{
		glPushName(objID);
		glPushMatrix();

		// save current joint position
		double originPos = body->getParentJoint()->getPosition(0);

		// set joint position as t-pose
		body->getParentJoint()->setPosition(0, 0);


		Eigen::Vector3d color;
		if(body_name.substr(1) == "_Femur")
			color = Eigen::Vector3d(0.0, 0.0, 1.0);
		else
			color = Eigen::Vector3d(0.0, 1.0, 0.0);


		double radius = 0.15;
		Eigen::Isometry3d parentJointT = body->getTransform() * body->getParentJoint()->getTransformFromChildBodyNode();
		Eigen::Vector3d joint_to_body = body->getTransform().translation() - parentJointT.translation();
		joint_to_body = joint_to_body / joint_to_body.norm();
		Eigen::Vector3d jointPos = parentJointT.translation();

		double rev = 0.0, phi = 0.0, roll = -M_PI;
		double stepRev = 2 * M_PI / numRev, stepPhi = 2 * M_PI / numPhi, stepRoll = 2 * M_PI / numRoll;

		for (int i = 0; i < numRev; i++) {
			rev = stepRev * i;

	//			setPose(body_name, rev, 0, 0);


			if(body_rom->data_revolute[i]) {
				glPushMatrix();
				glTranslated(jointPos[0], jointPos[1], jointPos[2]);

				color = Eigen::Vector3d(1.0, 0.0, 0.3);

				GLfloat mat_amb_diff[] = { GLfloat(0.6*color[0]), GLfloat(0.6*color[1]), GLfloat(0.6*color[2]), 1.0};
				glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,
							 mat_amb_diff);
				glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE,
							 mat_amb_diff);
				glColor4f(color[0],color[1],color[2], 1.0);
				const int numTheta = 18;
				const int numPhi = 36;
				const int numRoll = 18;
				double radius = 0.15;

				double rev = 0.0, phi = 0.0, roll = -M_PI;
				double stepRev = 2 * M_PI / numRev, stepPhi = 2 * M_PI / numPhi, stepRoll = 2 * M_PI / numRoll;

				rev = stepRev * i;
				rev = body_rom->revoluteEdit(rev);

				while(abs(rev)>M_PI)
	            {
	                rev -= 2* M_PI *  rev/abs(rev);
	            }


				vector<Eigen::Vector3d> _points, points(8);

				setPose(body_name, rev - stepRev / 2, 0, 0);
				joint_to_body = body->getTransform().translation() - parentJointT.translation();
				joint_to_body = joint_to_body / joint_to_body.norm() * radius;
				_points.emplace_back(joint_to_body);

				setPose(body_name, rev + stepRev / 2, 0, 0);
				joint_to_body = body->getTransform().translation() - parentJointT.translation();
				joint_to_body = joint_to_body / joint_to_body.norm() * radius;
				_points.emplace_back(joint_to_body);

				_points.emplace_back(_points[1]*0.95);
				_points.emplace_back(_points[0]*0.95);

				Eigen::Vector3d n ;
				n = (_points[0]-_points[1]).cross(_points[2]-_points[1]);
				n.normalize();

				for (int i=0;i<4;i++){
					points[i] = _points[i] + n * radius * 0.025;
					points[i+4] = _points[i] - n * radius * 0.025;
				}

				glBegin(GL_QUADS);
				n = (points[0]-points[1]).cross(points[2]-points[1]);
				glNormal3f(n[0],n[1],n[2]);
				for (int i=0;i<4;i++){
					glVertex3f(points[i][0],points[i][1],points[i][2]);
				}

				glEnd();

				glBegin(GL_QUADS);
				n = -(points[4]-points[5]).cross(points[6]-points[5]);
				glNormal3f(n[0],n[1],n[2]);
				for (int i=4;i<8;i++){
					glVertex3f(points[i][0],points[i][1],points[i][2]);
				}
				glEnd();

				glBegin(GL_QUADS);
				n = -(points[0]-points[1]).cross(points[5]-points[1]);
				glNormal3f(n[0],n[1],n[2]);
				glVertex3f(points[0][0],points[0][1],points[0][2]);
				glVertex3f(points[1][0],points[1][1],points[1][2]);
				glVertex3f(points[5][0],points[5][1],points[5][2]);
				glVertex3f(points[4][0],points[4][1],points[4][2]);
				glEnd();

				glBegin(GL_QUADS);
				n = -(points[1]-points[2]).cross(points[6]-points[2]);
				glNormal3f(n[0],n[1],n[2]);
				glVertex3f(points[1][0],points[1][1],points[1][2]);
				glVertex3f(points[2][0],points[2][1],points[2][2]);
				glVertex3f(points[6][0],points[6][1],points[6][2]);
				glVertex3f(points[5][0],points[5][1],points[5][2]);
				glEnd();

				glBegin(GL_QUADS);
				n = -(points[2]-points[3]).cross(points[7]-points[3]);
				glNormal3f(n[0],n[1],n[2]);
				glVertex3f(points[2][0],points[2][1],points[2][2]);
				glVertex3f(points[3][0],points[3][1],points[3][2]);
				glVertex3f(points[7][0],points[7][1],points[7][2]);
				glVertex3f(points[6][0],points[6][1],points[6][2]);
				glEnd();

				glBegin(GL_QUADS);
				n = -(points[3]-points[0]).cross(points[0]-points[4]);
				glNormal3f(n[0],n[1],n[2]);
				glVertex3f(points[3][0],points[3][1],points[3][2]);
				glVertex3f(points[0][0],points[0][1],points[0][2]);
				glVertex3f(points[4][0],points[4][1],points[4][2]);
				glVertex3f(points[7][0],points[7][1],points[7][2]);
				glEnd();



	//
	//					setPose(body_name, rev, phi, 0);
	//					joint_to_body = body->getTransform().translation() - parentJointT.translation();
	//					joint_to_body = joint_to_body / joint_to_body.norm() * radius;
	//					glTranslated(joint_to_body[0],joint_to_body[1],joint_to_body[2]);
	//					GUI::DrawSphere(0.01);

				glPopMatrix();
				GLfloat no_mat[] = {0.0, 0.0, 0.0, 1.0};
				glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, no_mat);
			}
		}
		// Draw centerJ2B
		rev = body_rom->revoluteEditInfo.prev_center + body_rom->revoluteEditInfo.translate;
		// rev = body_rom->revoluteEdit(rev);
		setPose(body_name, rev, 0, 0);
		joint_to_body = body->getTransform().translation() - parentJointT.translation();
		joint_to_body = joint_to_body / joint_to_body.norm() * radius;
		Eigen::Vector3d p = jointPos;
		Eigen::Vector3d q = jointPos + joint_to_body * 1.2;
		GUI::DrawLine(p, q, Eigen::Vector3d(1,0,0));

		// restore joint position
		body->getParentJoint()->setPosition(0, originPos);
		glPopMatrix();
		glPopName();
	}

}
void MSSROM::setComfortablePosition()
{
	for(auto& body : ms->getSkeleton()->getBodyNodes())
	{
		if(IsMajorBallJoint(body->getName()) && body->getName().find("Hand") == string::npos
											 && body->getName().find("Foot") == string::npos)
		// if(body->getName()=="R_Femur")
		{
			std::string body_name = body->getName();
			BodyROM* body_rom = getROM(body_name);
			body->getParentJoint()->setPositions(Eigen::Vector3d::Zero());
			Eigen::Isometry3d zeroParentJointT = body->getTransform() * body->getParentJoint()->getTransformFromChildBodyNode();
			Eigen::Matrix3d rot_from_zero;
			Eigen::Vector3d world_cur_centerJ2B = zeroParentJointT.linear() * body_rom->cur_centerJ2B.normalized();
			Eigen::Vector3d axis = ms->getJtoBAxis(body_name).cross(world_cur_centerJ2B);
			double angle = atan2(axis.norm(), ms->getJtoBAxis(body_name).dot(world_cur_centerJ2B));
			axis.normalize();

			rot_from_zero = Eigen::AngleAxisd(angle, axis).toRotationMatrix();

			// Save center v vector
			edb->setCenterV_vector(body_name, zeroParentJointT.linear().inverse() * 
				rot_from_zero * ms->getVAxis(body_name));

			Eigen::Vector3d centerJ2BPosition =
				2.0 * QuaternionToAngleAxis(Eigen::Quaterniond(zeroParentJointT.linear().inverse() * 
				rot_from_zero));


			// cout<<world_cur_centerJ2B.transpose()<<endl;
			// cout<<ms->getJtoBAxis(body_name).transpose()<<endl;
			// cout<<angle<<endl;

			// cout<<centerJ2BPosition.transpose()<<endl;
			body->getParentJoint()->setPositions(centerJ2BPosition);
			cout<<body_name<<" "<<body_rom->cur_centerJ2B.transpose()<<endl;
			// cout<<zeroParentJointT.linear().inverse() * 
			// 	rot_from_zero<<endl;
		}

		if(IsMajorRevoluteJoint(body->getName()))
		// if(body->getName()=="R_Tibia")
		{
			BodyROM *body_rom = getROM(body->getName());
			double rev = body_rom->revoluteEditInfo.prev_center + body_rom->revoluteEditInfo.translate;
			setPose(body->getName(), rev, 0, 0);
			// cout<<body->getName()<<" "<<rev<<endl;
		}
		// cout<<body->getName()<<" Check : "<<ms->checkPosAvailable()<<endl;
	}
}

void MSSROM::setComfortablePosition(std::string body_name)
{
	BodyNodePtr body = ms->getSkeleton()->getBodyNode(body_name);
	if(IsMajorBallJoint(body->getName()))
	{
		std::string body_name = body->getName();
		BodyROM* body_rom = getROM(body_name);
		body->getParentJoint()->setPositions(Eigen::Vector3d::Zero());
		Eigen::Isometry3d zeroParentJointT = body->getTransform() * body->getParentJoint()->getTransformFromChildBodyNode();
		Eigen::Matrix3d rot_from_zero;
		Eigen::Vector3d world_cur_centerJ2B = zeroParentJointT.linear() * body_rom->cur_centerJ2B.normalized();
		Eigen::Vector3d axis = ms->getJtoBAxis(body_name).cross(world_cur_centerJ2B);
		double angle = atan2(axis.norm(), ms->getJtoBAxis(body_name).dot(world_cur_centerJ2B));
		axis.normalize();

		rot_from_zero = Eigen::AngleAxisd(angle, axis).toRotationMatrix();

		// Save center v vector
		edb->setCenterV_vector(body_name, zeroParentJointT.linear().inverse() * 
			rot_from_zero * ms->getVAxis(body_name));

		Eigen::Vector3d centerJ2BPosition =
			2.0 * QuaternionToAngleAxis(Eigen::Quaterniond(zeroParentJointT.linear().inverse() * 
			rot_from_zero));


		// cout<<world_cur_centerJ2B.transpose()<<endl;
		// cout<<ms->getJtoBAxis(body_name).transpose()<<endl;
		// cout<<angle<<endl;

		// cout<<centerJ2BPosition.transpose()<<endl;
		body->getParentJoint()->setPositions(centerJ2BPosition);
	}

}



void MSSROM::ComputeROM(std::string body_name, bool nonMuscleOn) {

	// get body with given name
	BodyNodePtr body = ms->getSkeleton()->getBodyNode(body_name);
	// rom information of body
	BodyROM *body_rom = getROM(body_name);

	cout<<body->getName()<<endl;
	if (body_rom->type == JOINT_TYPE::BALL_AND_SOCKET) {

		// save current joint position
		Eigen::Vector3d originPos = body->getParentJoint()->getPositions();

		// set joint position as t-pose
		body->getParentJoint()->setPositions(Eigen::Vector3d::Zero());

		Eigen::Isometry3d zeroParentJointT = body->getTransform() * body->getParentJoint()->getTransformFromChildBodyNode();

		double theta = 0.0, phi = 0.0, roll = -M_PI;
		double stepTheta = M_PI / numTheta, stepPhi = 2 * M_PI / numPhi, stepRoll = 2 * M_PI / numRoll;
		int cnt = 0;

		body_rom->comfortablePositionBall.setZero();
		for (int i = 0; i <= numTheta; i++) {
			for (int j = 0; j < numPhi; j++) {
				for (int k = 0; k < numRoll; k++) {
					theta = stepTheta * i;
					phi = stepPhi * j;
					roll = stepRoll * k;

					setPose(body_name, theta, phi, roll);

					bool flag = true;
					if(nonMuscleOn)
					{
						if(body_rom->name.find("Femur")!= string::npos)
						{
							Eigen::Vector3d j2b = ms->getJtoBAxis(body_rom->name);
							if(j2b[1]> -0.7 && j2b[2] < 0.0)
							{
								flag = false;
							}
						}

						if(body_rom->name.find("Foot")!= string::npos)
						{
							double stdTheta = toStandardAngle(theta);
							if(abs(stdTheta) >M_PI/2.0)
							{
								flag = false;
							}
						}
					}
					// if (!(body_rom->lowerLimit[0] <= theta && theta <= body_rom->upperLimit[0])) flag = false;
					
					body_rom->data_ball[i][j][k] = (ms->checkPosAvailable() && flag) ? 1 : 0;

					Eigen::Vector3d joint_to_body = body->getTransform().translation() - zeroParentJointT.translation();
					joint_to_body = joint_to_body / joint_to_body.norm();

					body_rom->comfortablePositionBall += body_rom->data_ball[i][j][k] * joint_to_body;

					cnt += body_rom->data_ball[i][j][k];
				}
			}
		}
		body_rom->comfortablePositionBall /= cnt;
		body_rom->prev_centerJ2B = body_rom->cur_centerJ2B;
		body_rom->conicScale = 1.0;
		// restore joint position
		body->getParentJoint()->setPositions(originPos);

	} else if (body_rom->type == JOINT_TYPE::REVOLUTE) {
		// save current joint position
		double originPos = body->getParentJoint()->getPosition(0);
		body_rom->revoluteEditInfo.prev_center = body_rom->revoluteEditInfo.center + body_rom->revoluteEditInfo.translate;
		body_rom->revoluteEditInfo.translate = 0.0;
		body_rom->revoluteEditInfo.scale = 1.0;
		// set joint position as t-pose
		body->getParentJoint()->setPosition(0, 0);
		double rev = 0.0;
		double stepRev = 2 * M_PI / numRev;

		int cnt=0;
		body_rom->comfortablePositionRevolute = 0.0;
		for (int i = 0; i <= numRev; i++) {
			rev = stepRev * i;
			setPose(body_name, rev, 0, 0);
			body_rom->data_revolute[i] = ms->checkPosAvailable() ? 1 : 0;
		}
		body_rom->comfortablePositionRevolute = body_rom->revoluteEditInfo.prev_center;
		body->getParentJoint()->setPosition(0, originPos);

	}
}


void MSSROM::ComputeROMCenter(std::string body_name, bool nonMuscleOn)
{

	// get body with given name
	BodyNodePtr body = ms->getSkeleton()->getBodyNode(body_name);
	// rom information of body
	BodyROM *body_rom = getROM(body_name);

	if (body_rom->type == JOINT_TYPE::BALL_AND_SOCKET) {

		// save current joint position
		Eigen::Vector3d originPos = body->getParentJoint()->getPositions();

		// set joint position as t-pose
		body->getParentJoint()->setPositions(Eigen::Vector3d::Zero());

		Eigen::Isometry3d zeroParentJointT = body->getTransform() * body->getParentJoint()->getTransformFromChildBodyNode();

		double theta = 0.0, phi = 0.0, roll = -M_PI;
		double stepTheta = M_PI / numTheta, stepPhi = 2 * M_PI / numPhi, stepRoll = 2 * M_PI / numRoll;
		int cnt = 0;

		body_rom->centerJ2B = Eigen::Vector3d::Zero();

		for (int i = 0; i <= numTheta; i++) {
			for (int j = 0; j < numPhi; j++) {
				for (int k = 0; k < numRoll; k++) {
					theta = stepTheta * i;
					phi = stepPhi * j;
					roll = stepRoll * k;

					setPose(body_name, theta, phi, roll);

					bool flag = true;

					if(nonMuscleOn)
					{
						if(body_rom->name.find("Femur")!= string::npos)
						{
							Eigen::Vector3d j2b = ms->getJtoBAxis(body_rom->name);
							if(j2b[1]> -0.7 && j2b[2] < 0.0)
							{
								flag = false;
							}
						}

						if(body_rom->name.find("Foot")!= string::npos)
						{
							double stdTheta = toStandardAngle(theta);
							if(abs(stdTheta) >M_PI/2.0)
							{
								flag = false;
							}
						}
					}
					
					// if (!(body_rom->lowerLimit[0] <= theta && theta <= body_rom->upperLimit[0])) flag = false;
					
					body_rom->data_ball[i][j][k] = (ms->checkPosAvailable() && flag) ? 1 : 0;

					cnt += body_rom->data_ball[i][j][k];

					Eigen::Vector3d joint_to_body = body->getTransform().translation() - zeroParentJointT.translation();
					joint_to_body = joint_to_body / joint_to_body.norm();

					body_rom->centerJ2B += body_rom->data_ball[i][j][k] * joint_to_body;
				}
			}
		}
		if (cnt>0) {
			body_rom->centerJ2B /= cnt;
			body_rom->centerJ2B = zeroParentJointT.linear().inverse() * body_rom->centerJ2B.normalized();
			body_rom->conicScale = 1.0;
			body_rom->cur_centerJ2B = body_rom->centerJ2B;
			body_rom->prev_centerJ2B = body_rom->centerJ2B;
			body_rom->cur_roll = 0.0;

			body->getParentJoint()->setPositions(Eigen::Vector3d::Zero());
			edb->setCenterJ2B_vector(body_name, zeroParentJointT.linear().inverse() * body_rom->centerJ2B);

			Eigen::Matrix3d rot_from_zero;
			Eigen::Vector3d world_cur_centerJ2B = zeroParentJointT.linear() * body_rom->cur_centerJ2B.normalized();
			Eigen::Vector3d axis = ms->getJtoBAxis(body_name).cross(world_cur_centerJ2B);
			double angle = atan2(axis.norm(), ms->getJtoBAxis(body_name).dot(world_cur_centerJ2B));
			axis.normalize();

			rot_from_zero = Eigen::AngleAxisd(angle, axis).toRotationMatrix();

			// Save center v vector
			edb->setCenterV_vector(body_name, zeroParentJointT.linear().inverse() * 
				rot_from_zero * ms->getVAxis(body_name));

			// Eigen::Vector3d centerJ2BPosition =
			// 	2.0 * QuaternionToAngleAxis(Eigen::Quaterniond(zeroParentJointT.linear().inverse() * 
			// 	rot_from_zero));


			// cout<<world_cur_centerJ2B.transpose()<<endl;
			// cout<<ms->getJtoBAxis(body_name).transpose()<<endl;
			// cout<<angle<<endl;

			// cout<<centerJ2BPosition.transpose()<<endl;
			// body->getParentJoint()->setPositions(centerJ2BPosition);

			// cout<<angle<<endl;

			// cout<<ms->getJtoBAxis(body->getName()).dot(body_rom->centerJ2B.normalized())<<endl;
			// cout<<endl;
			body->getParentJoint()->setPositions(originPos);
		}
		else
		{
			// restore joint position
			body->getParentJoint()->setPositions(originPos);
		}


	} else if (body_rom->type == JOINT_TYPE::REVOLUTE) {
		// save current joint position
		double originPos = body->getParentJoint()->getPosition(0);

		// set joint position as t-pose
		body->getParentJoint()->setPosition(0, 0);

		double rev = 0.0;
		double stepRev = 2 * M_PI / numRev;

		double tempCenter= 0.0;
		int cnt=0;
		for (int i = 0; i <= numRev; i++) {
			rev = stepRev * i;

			setPose(body_name, rev, 0, 0);

			body_rom->data_revolute[i] = ms->checkPosAvailable() ? 1 : 0;

			tempCenter += body_rom->data_revolute[i] * rev;
			cnt += body_rom->data_revolute[i];

		}

		tempCenter /= cnt;
		tempCenter = toStandardAngle(tempCenter);

        rev = 0.0;
        cnt=0;
        body_rom->revoluteEditInfo.center = 0.0;
		for (int i = 0; i <= numRev; i++) {
			rev = stepRev * i;

			// rev = toStandardAngle(rev);

			rev = toStandardAngle(rev - tempCenter) + tempCenter;
	        
        	setPose(body_name, rev, 0, 0);

			body_rom->data_revolute[i] = ms->checkPosAvailable() ? 1 : 0;

			body_rom->revoluteEditInfo.center += body_rom->data_revolute[i] * rev;
			cnt += body_rom->data_revolute[i];
		}
		body_rom->revoluteEditInfo.center /= cnt;
		body_rom->revoluteEditInfo.center = toStandardAngle(body_rom->revoluteEditInfo.center);
		body_rom->revoluteEditInfo.prev_center = body_rom->revoluteEditInfo.center;
		body_rom->revoluteEditInfo.translate = 0.0;
		body_rom->revoluteEditInfo.scale = 1.0;

		edb->setRevCenter(body_name, body_rom->revoluteEditInfo.center);

		// restore joint position
		body->getParentJoint()->setPosition(0, originPos);
	}
}
