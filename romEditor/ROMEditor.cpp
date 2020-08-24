#include "ROMEditor.h"
#include "../visualROM/MSSROM.h"
#include "../generator/motionAdjust/BVHparser.h"
#include "../generator/motionAdjust/MotionAdjust.h"
#include "dart/dart.hpp"
#include <stdio.h>
using namespace std;
using namespace dart;
using namespace dart::dynamics;

EditInfoDB::EditInfoDB()
{
}

int EditInfoDB::findBodyIndex(std::string body_name)
{
    for(int i=0;i<body_name_vector.size();i++)
    {
        if(body_name_vector[i] == body_name)
            return i;
    }
    return -1;
}

void EditInfoDB::setEditInfo(std::string body_name, 
    Eigen::Vector3d new_centerJ2B, double centerJ2B_scale, 
    double roll_trans, double roll_scale)
{
    if(!IsBallJoint(body_name))
    {
        cout<<"setEditInfo : "<<body_name<<" is not ball joint"<<endl;
        exit(1);
        return;
    }
    int body_index = findBodyIndex(body_name);
    if(body_index == -1)
    {
        cout<<body_name<<" : setEditInfo.. not exists!"<<endl;
    }
    else
    {
        new_centerJ2B_vector[body_index] = new_centerJ2B;
        centerJ2B_scale_vector[body_index] *= centerJ2B_scale;
        roll_trans_vector[body_index] = roll_trans;
        roll_scale_vector[body_index] = roll_scale;
    }
}

// We will call this first.(ball-and-socket)
void EditInfoDB::setCenterJ2B_vector(std::string body_name, Eigen::Vector3d centerJ2B)
{
    if(!IsBallJoint(body_name))
    {
        cout<<"setCenterJ2B_vector : "<<body_name<<" is not ball joint"<<endl;
        exit(1);
        return;
    }
    int body_index = findBodyIndex(body_name);
    if(body_index == -1)
    {
        body_name_vector.push_back(body_name);
        new_centerJ2B_vector.push_back(centerJ2B);
        centerJ2B_scale_vector.push_back(1.0);
        roll_trans_vector.push_back(0.0);
        roll_scale_vector.push_back(1.0);

        rev_center_vector.push_back(0.0);
        rev_trans_vector.push_back(0.0);
        rev_scale_vector.push_back(1.0);

        centerJ2B_vector.push_back(centerJ2B);
        centerV_vector.push_back(Eigen::Vector3d::UnitX());

    }
    else
    {
        // body_name_vector[body_index]=(body_name);
        // new_centerJ2B_vector[body_index]=(centerJ2B);
        // centerJ2B_scale_vector[body_index]=(1.0);
        // roll_trans_vector[body_index]=(0.0);
        // roll_scale_vector[body_index]=(1.0);

        // rev_center_vector[body_index]=(0.0);
        // rev_trans_vector[body_index]=(0.0);
        // rev_scale_vector[body_index]=(1.0);

        centerJ2B_vector[body_index]=(centerJ2B);
        centerV_vector[body_index]=(Eigen::Vector3d::UnitX());
    }
}

// We will call this first.(revolute)
void EditInfoDB::setRevCenter(std::string body_name, double rev_center)
{
    if(IsBallJoint(body_name))
    {
        cout<<"setCenterJ2B_vector : "<<body_name<<" is not revolute joint"<<endl;
        exit(1);
        return;
    }
    int body_index = findBodyIndex(body_name);
    if(body_index == -1)
    {
        body_name_vector.push_back(body_name);
        new_centerJ2B_vector.push_back(Eigen::Vector3d::UnitX());
        centerJ2B_scale_vector.push_back(1.0);
        roll_trans_vector.push_back(0.0);
        roll_scale_vector.push_back(1.0);

        rev_center_vector.push_back(rev_center);
        rev_trans_vector.push_back(0.0);
        rev_scale_vector.push_back(1.0);

        centerJ2B_vector.push_back(Eigen::Vector3d::UnitX());
        centerV_vector.push_back(Eigen::Vector3d::UnitX());

    }
    else
    {
        rev_center_vector[body_index] = rev_center;
    }
}


void EditInfoDB::setCenterV_vector(std::string body_name, Eigen::Vector3d centerV)
{
    if(!IsBallJoint(body_name))
    {
        cout<<"setCenterRoll : "<<body_name<<" is not ball joint"<<endl;
        exit(1);
        return;
    }
    int body_index = findBodyIndex(body_name);
    if(body_index == -1)
    {
        cout<<body_name<<" : setCenterV_vector.. not exists!"<<endl;
    }
    else
    {
        centerV_vector[body_index] = centerV;
    }
}


void EditInfoDB::setEditInfo(std::string body_name, double rev_trans, double rev_scale)
{
    if(IsBallJoint(body_name))
    {
        cout<<"setEditInfo : "<<body_name<<" is ball joint"<<endl;
        exit(1);
        return;
    }
    int body_index = findBodyIndex(body_name);
    if(body_index == -1)
    {
        cout<<body_name<<" : setEditInfo.. not exists!"<<endl;
    }
    else
    {
        rev_trans_vector[body_index] = rev_trans;
        rev_scale_vector[body_index] *= rev_scale;
    }
}



// Muscle Parameter editting. Other joint is fixed to current pos
void ROMEditor::editMPwithROM(std::shared_ptr<MusculoSkeletalSystem>& ms, EditInfoDB* edb)
{
    MotionAdjust* ma_edit = new MotionAdjust("../generator/motionAdjust/PosePriorDataset/bvh", ms);
    
    ma_edit->bvh_index = 0;
    ma_edit->bvh_frame = 0;

    // Start loading bvh file.
    ma_edit->LoadNewBVHFile();

    // Initialize muscle_max_l_mt_list, muscle_min_l_mt_list
    for(int i =0;i<ma_edit->ms->getNumMuscles();i++)
    {
        ma_edit->muscle_max_l_mt_list.push_back(0.0);
        ma_edit->muscle_min_l_mt_list.push_back(100000.0);
    }

    Eigen::VectorXd originPos = ma_edit->ms->getSkeleton()->getPositions();
    
    // Currently we just use 25% of bvhs for fast test.
    while(ma_edit->bvh_index < ma_edit->bvhLists.size()/1)
    {
        if(ma_edit->bvh_frame >= ma_edit->bvhParser->frames)
            ma_edit->LoadNewBVHFile();
        ROMEditor::setPositionFromFilteredBVH(ma_edit->ms, ma_edit->bvhParser, ma_edit->bvh_frame, edb);
        // BVHmanager::SetPositionFromBVH(ma_edit->ms->getSkeleton(), ma_edit->bvhParser, ma_edit->bvh_frame);

        double R_Tibia_angle = ma_edit->ms->getSkeleton()->getBodyNode("R_Tibia")->getParentJoint()->getPositions()[0];
        double R_ForeArm_angle = ma_edit->ms->getSkeleton()->getBodyNode("R_ForeArm")->getParentJoint()->getPositions()[0];
        double L_Tibia_angle = ma_edit->ms->getSkeleton()->getBodyNode("L_Tibia")->getParentJoint()->getPositions()[0];
        double L_ForeArm_angle = ma_edit->ms->getSkeleton()->getBodyNode("L_ForeArm")->getParentJoint()->getPositions()[0];

        while(abs(R_Tibia_angle)>M_PI)
        {
            R_Tibia_angle -= 2* M_PI *  R_Tibia_angle/abs(R_Tibia_angle);
        }
        while(abs(R_ForeArm_angle)>M_PI)
        {
            R_ForeArm_angle -= 2 * M_PI *  R_ForeArm_angle/abs(R_ForeArm_angle);
        }
        while(abs(L_Tibia_angle)>M_PI)
        {
            L_Tibia_angle -= 2 * M_PI *  L_Tibia_angle/abs(L_Tibia_angle);
        }
        while(abs(L_ForeArm_angle)>M_PI)
        {
            L_ForeArm_angle -= 2 * M_PI *  L_ForeArm_angle/abs(L_ForeArm_angle);
        }

        if( R_Tibia_angle >= 0 || R_ForeArm_angle >= 0 || L_Tibia_angle >= 0 || L_ForeArm_angle <= 0)
        {
            ma_edit->UpdateMuscleMINMAX_l_mt_list();
        }
        ma_edit->bvh_frame += 10;
    }
    cout<<"MIN MAX lmt edited"<<endl;
    ma_edit->ms->getSkeleton()->setPositions(originPos);

    // Sync the left and right muscles.
    // ma_edit->LeftRightSyncAdjustmentValue();

    // Change muscle's max l_mt value
    for(int i =0;i< ms->getNumMuscles();i++)
    {
         ms->getMuscles()[i]->max_l_mt = ma_edit->muscle_max_l_mt_list[i];
         ms->getMuscles()[i]->min_l_mt = ma_edit->muscle_min_l_mt_list[i];
    }
    FILE* out1=fopen("../generator/motionAdjust/adjustValue/max_min_l_mt_tildaEdit.txt","w");
    for(int i=0;i<ma_edit->ms->getNumMuscles();i++)
    {
        fprintf(out1, "%s %f %f", ma_edit->ms->getMuscles()[i]->name.c_str(), ma_edit->muscle_max_l_mt_list[i], ma_edit->muscle_max_l_mt_list[i]);
        fprintf(out1, "\n");
    }
    fclose(out1);
}


//For given bvh pose and edb data, the function filter the bvh pose to edited bvh pose
void ROMEditor::setPositionFromFilteredBVH(std::shared_ptr<MusculoSkeletalSystem>& ms, BVHparser *bvhParser, int bvh_frame, EditInfoDB* edb)
{
    SkeletonPtr skel =ms->getSkeleton();
    Eigen::Vector3d theta_axis = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d phi_axis = Eigen::Vector3d::UnitY();

    Eigen::VectorXd bvh_filtered_positions;
    // Approximate. This differs to our skeletal model.
    double cur_femur_size = 0.400;

    // We will change here to fit to any bvh. This is related to foot sliding
    bvhParser->scale = cur_femur_size / 16.34683;
    MotionNode* curNode = bvhParser->getRootNode();

    // We have to modify information for revolute joint 
    bvh_filtered_positions = skel->getPositions();
    bvh_filtered_positions.setZero();

    // For root position and orientation
    bvh_filtered_positions.segment(0,3) = 2 * Eigen::Vector3d(
        curNode->data[bvh_frame][3], 
        curNode->data[bvh_frame][4], 
        curNode->data[bvh_frame][5]);

    bvh_filtered_positions.segment(3,3) = Eigen::Vector3d(
        curNode->data[bvh_frame][0] * bvhParser->scale, 
        curNode->data[bvh_frame][1] * bvhParser->scale + (1.0 - 31.64356 * bvhParser->scale) - 1.0, 
        curNode->data[bvh_frame][2] * bvhParser->scale);
    skel->setPositions(bvh_filtered_positions);

    curNode= curNode->getNextNode();
    int dof = 0;
    BodyNodePtr body;
    std::string body_name;

    while(curNode!=nullptr)
    {
        if(curNode->ContainedInModel())
        {
            body_name = curNode->getName_std();
            body = skel->getBodyNode(body_name);
            dof = body->getParentJoint()->getNumDofs();
            int body_index_edb = edb->findBodyIndex(body_name);
            if(dof == 3)    // ball joint
            {
                Eigen::Vector3d bvh_position = Eigen::Vector3d(curNode->data[bvh_frame][0], 
                                                        curNode->data[bvh_frame][1], 
                                                        curNode->data[bvh_frame][2]);
                // if(body_name == "R_Shoulder" || body_name == "L_Shoulder")
                // {
                //     bvh_position.setZero();
                // }

                Eigen::Matrix3d bvh_rotation = AngleAxisToQuaternion(bvh_position).toRotationMatrix();

                body->getParentJoint()->setPositions(Eigen::Vector3d::Zero());
                Eigen::Isometry3d zeroParentJointT =
                    body->getTransform() * body->getParentJoint()->getTransformFromChildBodyNode();
                body->getParentJoint()->setPositions(2.0 * bvh_position);
                Eigen::Isometry3d curParentJointT =
                    body->getTransform() * body->getParentJoint()->getTransformFromChildBodyNode();

                int body_index_skel = skel->getIndexOf(body);
                if(body_index_edb!= -1)
                {
                    // Zero pose is identity rotation.

                    // At first we compute the rotation change from zero pose(identity rotation) to center pose.
                    Eigen::Matrix3d zeroToCenterOritentation;

                    // World Coord. Center joint to body vector from edb data
                    Eigen::Vector3d cj2b = zeroParentJointT.linear() * edb->centerJ2B_vector[body_index_edb].normalized();

                    // World Coord. Center v vector from edb data 
                    Eigen::Vector3d cv = zeroParentJointT.linear() * edb->centerV_vector[body_index_edb].normalized();

                    // Conic rotation
                    // World Coord. Joint to body vector for zero pose of the joint.
                    Eigen::Vector3d zero_j2b = 
                        zeroParentJointT.linear()* curParentJointT.linear().inverse() * ms->getJtoBAxis(body_name).normalized();
                    
                    Eigen::Vector3d zero_conic_trans_axis = zero_j2b.cross(cj2b);
                    double zero_conic_trans_angle = atan2(zero_conic_trans_axis.norm(), zero_j2b.dot(cj2b));
                    zero_conic_trans_axis.normalize();

                    Eigen::Matrix3d zero_conic_rotation = Eigen::AngleAxisd(zero_conic_trans_angle, zero_conic_trans_axis).toRotationMatrix();

                    // Roll rotation
                    // World Coord. v vector for zero pose.
                    Eigen::Vector3d zero_v = zero_conic_rotation * 
                        zeroParentJointT.linear()* curParentJointT.linear().inverse() * ms->getVAxis(body_name).normalized();
                    
                    Eigen::Vector3d zero_roll_axis = zero_v.cross(cv);
                    double zero_roll_angle = atan2(zero_roll_axis.norm(), zero_v.dot(cv));
                    zero_roll_axis.normalize();

                    Eigen::Matrix3d zero_roll_rotation = Eigen::AngleAxisd(zero_roll_angle, zero_roll_axis).toRotationMatrix();

                    // Zero pose to center pose rotation.
                    zeroToCenterOritentation = zero_roll_rotation * zero_conic_rotation;



                    // Now we compute the rotation from center pose rotation to bvh pose rotation.

                    // World Coord. Joint to body vector for current bvh pose.
                    Eigen::Vector3d cur_j2b = ms->getJtoBAxis(body_name);


                    Eigen::Vector3d conic_scale_axis = cj2b.cross(cur_j2b);
                    double conic_scale_angle = atan2(conic_scale_axis.norm(), cj2b.dot(cur_j2b));
                    conic_scale_axis.normalize();

                    // World Coord. Rotation of conic scaling.
                    Eigen::Matrix3d rotation_conic_scale = 
                        Eigen::AngleAxisd(conic_scale_angle*
                            edb->centerJ2B_scale_vector[body_index_edb],
                                            conic_scale_axis).toRotationMatrix();

                    // World Coord. Center joint to body vector of conic translated.
                    Eigen::Vector3d ncj2b = zeroParentJointT.linear() * edb->new_centerJ2B_vector[body_index_edb].normalized();

                    Eigen::Vector3d conic_trans_axis = cj2b.cross(ncj2b);
                    double conic_trans_angle = atan2(conic_trans_axis.norm(), cj2b.dot(ncj2b));
                    if(conic_trans_angle!=0)
                        conic_trans_axis.normalize();

                    // World Coord. Rotation of conic translating.
                    Eigen::Matrix3d rotation_conic_trans = Eigen::AngleAxisd(conic_trans_angle, conic_trans_axis).toRotationMatrix();

                    // World Coord. V vector of current bvh pose.
                    Eigen::Vector3d cur_v = ms->getVAxis(body_name);
                    
                    //World Coord. Conic Rotated center v vector.
                    Eigen::Vector3d roted_cv = Eigen::AngleAxisd(conic_scale_angle,
                                            conic_scale_axis).toRotationMatrix() * cv;

                    roted_cv.normalize();

                    Eigen::Vector3d roll_scale_axis = roted_cv.cross(cur_v);
                    double roll_scale_angle = atan2(roll_scale_axis.norm(), roted_cv.dot(cur_v));
                    if(roll_scale_angle!=0)
                        roll_scale_axis.normalize();


                    if(roll_scale_axis.dot(cur_j2b) < 0)
                        roll_scale_angle *= -1;

                    Eigen::Matrix3d rotation_roll_scale = 
                        Eigen::AngleAxisd(roll_scale_angle*(edb->roll_scale_vector[body_index_edb]), rotation_conic_scale*cj2b).toRotationMatrix();
                    Eigen::Matrix3d rotation_roll_trans = 
                        Eigen::AngleAxisd(edb->roll_trans_vector[body_index_edb],  rotation_conic_scale*cj2b).toRotationMatrix();

                    // Final Rotation.
                    Eigen::Matrix3d final_rotation = zeroParentJointT.linear().inverse() * rotation_roll_scale * rotation_roll_trans* 
                        rotation_conic_scale* rotation_conic_trans * zeroToCenterOritentation *zeroParentJointT.linear();

                    // cout<<body_name<<endl;
                    // cout<<"conic scale : "<<edb->centerJ2B_scale_vector[body_index_edb]<<endl;
                    // cout<<edb->centerJ2B_vector[body_index_edb].transpose()<<endl;
                    // cout<<edb->new_centerJ2B_vector[body_index_edb].transpose()<<endl;
                    // cout<<endl;
                    // cout<<"roll scale : "<<edb->roll_scale_vector[body_index_edb]<<endl;
                    // cout<<"roll trans : "<<edb->roll_trans_vector[body_index_edb]<<endl;
                    // cout<<final_rotation.inverse() * bvh_rotation<<endl;
                    // cout<<endl;

                    body->getParentJoint()->setPositions(2.0 * 
                        QuaternionToAngleAxis(Eigen::Quaterniond(final_rotation)));

                }
                
            }
            else if (dof == 1)
            {
                //angle axis have value : θ/2.0 * quternion.vec()
                //only think about x rotation. but arm have y rotation.
                // if(body_index_edb!= -1)
                if(body_index_edb!= -1)
                {

                    if(curNode->getName_std() == "R_ForeArm" || curNode->getName_std() == "L_ForeArm" )
                    {
                        skel->getBodyNode(curNode->getName_std())->getParentJoint()->setPosition(0, 
                            edb->rev_center_vector[body_index_edb] + (curNode->data[bvh_frame][1] * 2.0 - edb->rev_center_vector[body_index_edb]) * edb->rev_scale_vector[body_index_edb] + 
                        edb->rev_trans_vector[body_index_edb]);

                    }
                    
                    else
                    {
                        skel->getBodyNode(curNode->getName_std())->getParentJoint()->setPosition(0, 
                            edb->rev_center_vector[body_index_edb] + (curNode->data[bvh_frame][0] * 2.0 - edb->rev_center_vector[body_index_edb]) * edb->rev_scale_vector[body_index_edb] + 
                        edb->rev_trans_vector[body_index_edb]);

                    }
                }
                else
                {
                    if(curNode->getName_std() == "R_ForeArm" || curNode->getName_std() == "L_ForeArm" )
                        skel->getBodyNode(curNode->getName_std())->getParentJoint()->setPosition(0, curNode->data[bvh_frame][1] * 2.0);
                    else
                        skel->getBodyNode(curNode->getName_std())->getParentJoint()->setPosition(0, curNode->data[bvh_frame][0] * 2.0);
                }
                
            }
            else
            {
                // cout<<"not expected"<<endl;
                // exit(1);
            }
        }
        else
        {
            // cout<<curNode->getName()<<endl;
        }
        curNode = curNode->getNextNode();
    }
}
