//
// Created by minseok on 11/09/18.
//
#include <boost/filesystem.hpp>
#include <stdlib.h>
#include <memory>
#include "../../world/IntegratedWorld.h"
#include "../../model/MusculoSkeletalSystem.h"
#include "../../render/SimWindow.h"
#include "BVHparser.h"
#include "MotionAdjust.h"
#include <stdio.h>
#include <vector>
#include <string>
#include <iostream>
#include <experimental/filesystem>
using namespace std;

MotionAdjust::MotionAdjust(const char* dir, std::shared_ptr<MusculoSkeletalSystem>& ms)
{
    this->ms = ms;
    InitBVHLists(dir);
    // nnChecker = new NNChecker("../nnChecker");
    // nnChecker->train_nn(NN_TARGET::LEFTLEG);
    // nnChecker->load_nn(NN_TARGET::RIGHTLEG);
    this->bvh_index = 0;
    this->bvh_frame = 0;
}

void
MotionAdjust::
InitBVHLists(const char* dir)
{
    std::string path(dir);
    for(auto & p : std::experimental::filesystem::directory_iterator(path))
    {
        for(auto & p_sub : std::experimental::filesystem::directory_iterator(p))
        {
            std::string p_str(p_sub.path());
            if(p_str.substr(p_str.size()-4, 4)==".bvh")
            {
                this->bvhLists.push_back(p_str);
            }
        }
    }
}


void BVHmanager::SetPositionFromBVH(dart::dynamics::SkeletonPtr skel, BVHparser *bvhParser, int bvh_frame)
{
    // approximate
    double cur_femur_size = 0.400;

    // we will change here to fit to any bvh
    bvhParser->scale = cur_femur_size / 16.34683;
    MotionNode* curNode = bvhParser->getRootNode();

    //we have to modify information for revolute joint 
    Eigen::VectorXd bvh_position = skel->getPositions();
    bvh_position.setZero();

    // for root position and orientation
    bvh_position.segment(0,3) = 2 * Eigen::Vector3d(
        curNode->data[bvh_frame][3], 
        curNode->data[bvh_frame][4], 
        curNode->data[bvh_frame][5]);

    bvh_position.segment(3,3) = Eigen::Vector3d(
        curNode->data[bvh_frame][0] * bvhParser->scale, 
        curNode->data[bvh_frame][1] * bvhParser->scale + (1.0 - 31.64356 * bvhParser->scale), 
        curNode->data[bvh_frame][2] * bvhParser->scale);
    skel->setPositions(bvh_position);

    curNode= curNode->getNextNode();
    int index = 0;
    int dof = 0;

    while(curNode!=nullptr)
    {
        if(curNode->ContainedInModel())
        {
            dof = skel->getBodyNode(curNode->getName_std())->getParentJoint()->getNumDofs();
            if(dof == 3)    // ball joint
            {
                // if(curNode->getName_std()!="R_Shoulder" && curNode->getName_std()!="L_Shoulder")
                if(true)
                {
                    // cout<<curNode->getName_std()<<" "<<2.0 * Eigen::Vector3d(
                    //     curNode->data[bvh_frame][0], 
                    //     curNode->data[bvh_frame][1], 
                    //     curNode->data[bvh_frame][2]).transpose()<<endl;
                     skel->getBodyNode(curNode->getName_std())->getParentJoint()->setPositions(2.0 * Eigen::Vector3d(
                        curNode->data[bvh_frame][0], 
                        curNode->data[bvh_frame][1], 
                        curNode->data[bvh_frame][2]));
                 }
                 else
                 {
                    skel->getBodyNode(curNode->getName_std())->getParentJoint()->setPositions(Eigen::Vector3d::Zero());
                 }

            }
            else if (dof == 1)
            {
                //angle axis have value : θ/2.0 * quternion.vec()
                //only think about x rotation
                if(curNode->getName_std() == "R_ForeArm" || curNode->getName_std() == "L_ForeArm" )
                    skel->getBodyNode(curNode->getName_std())->getParentJoint()->setPosition(0, curNode->data[bvh_frame][1] * 2.0);
                else
                    skel->getBodyNode(curNode->getName_std())->getParentJoint()->setPosition(0, curNode->data[bvh_frame][0] * 2.0);
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


    // shoulder blocking
    // skel->setPosition(30, 0);
    // skel->setPosition(31, 0);
    // skel->setPosition(32, 0);

    // skel->setPosition(40, 0);
    // skel->setPosition(41, 0);
    // skel->setPosition(42, 0);

}

void WriteAdjustmentValueMIN_MAX_LMT(string name, std::shared_ptr<Muscle> muscle,
                                                double min_l_m_tilda, double max_l_m_tilda,
                                                double l_mt_min, double l_mt_max,
                                                bool hard_min, bool hard_max)
{
    //Remove the existing file. We will write adjustment value one by one.
    string filePath = "../generator/motionAdjust/adjustValue/adjustment_";
    filePath = filePath + name + ".txt";
    FILE* out=fopen(filePath.c_str(),"a+");
    // std::cout<<"# writing file "<<filePath<<std::endl;
    double a_val, b_val;
    MuscleCal::compute_a_b_val(muscle, 
            min_l_m_tilda, max_l_m_tilda, 
            l_mt_min, l_mt_max, 
            &a_val, &b_val, 
            hard_min, hard_max);
        fprintf(out, "%s ", muscle->name.c_str());
        fprintf(out, "%.3f %.3f", a_val, b_val);
        fprintf(out, "\n");
    fclose(out);
}

void MotionAdjust::LoadNewBVHFile()
{
    if(bvh_index!=0)
        delete bvhParser;
    // cout<<"Loading file \r"<<this->bvhLists[bvh_index]<<endl;
    bvhParser = new BVHparser(this->bvhLists[bvh_index].c_str());
    bvhParser->InitMatchNameListForMotionNode("../generator/motionAdjust/body_name_match.txt");
    this->bvh_frame = 0;
    bvh_index++;
    cout<<"\rAdjusting bvh file " << std::setfill('0') << std::setw(3)<<bvh_index<<"/"<<this->bvhLists.size()<<std::flush;

    // if want to render :
    // if(bvh_index >= motionAdjust->bvhLists.size()/5.0)
    // {
    //     cout<<"end!"<<endl;
    //     executePrintingFiberLengthAngle();
    //     exit(1);
    // }
}

void MotionAdjust::UpdateMuscleMINMAX_l_mt_list()
{
    for(int i =0;i<ms->getNumMuscles();i++)
    {
        double cur_muscle_l_mt = ms->getMuscles()[i]->getLength();
        if(cur_muscle_l_mt > muscle_max_l_mt_list[i])
            muscle_max_l_mt_list[i] = cur_muscle_l_mt;
        if(cur_muscle_l_mt < muscle_min_l_mt_list[i])
            muscle_min_l_mt_list[i] = cur_muscle_l_mt;
    }
}

void MotionAdjust::LeftRightSyncAdjustmentValue()
{
    for(int i =0;i<this->ms->getNumMuscles();i++)
    {
        for(int j=i+1;j<this->ms->getNumMuscles();j++)
        {
            if(this->ms->getMuscles()[i]->name.substr(1) ==
             this->ms->getMuscles()[j]->name.substr(1))
            {
                if(muscle_max_l_mt_list[i] > muscle_max_l_mt_list[j])
                    muscle_max_l_mt_list[j] = muscle_max_l_mt_list[i];
                else
                    muscle_max_l_mt_list[i] = muscle_max_l_mt_list[j];

                if(muscle_min_l_mt_list[i] < muscle_min_l_mt_list[j])
                    muscle_min_l_mt_list[j] = muscle_min_l_mt_list[i];
                else
                    muscle_min_l_mt_list[i] = muscle_min_l_mt_list[j];
            }
        }
    }
}

void MotionAdjust::SetAdjustmentValueForMotionFile(bool write, bool apply, std::string label_name)
{

    this->ms->SetMin_l_m_tilda(0.0);
    this->ms->SetMax_l_m_tilda(1.6);
        
    if(write)
    {
        this->bvh_index = 0;
        this->bvh_frame = 0;
        this->LoadNewBVHFile();

        for(int i =0;i<this->ms->getNumMuscles();i++)
        {
            this->muscle_max_l_mt_list.push_back(0.0);
            this->muscle_min_l_mt_list.push_back(100000.0);
        }
        Eigen::VectorXd originPos = this->ms->getSkeleton()->getPositions();
        //currently use just 20%
        while(this->bvh_index < this->bvhLists.size()/1)
        {
            if(this->bvh_frame >= this->bvhParser->frames)
                this->LoadNewBVHFile();
            BVHmanager::SetPositionFromBVH(this->ms->getSkeleton(), this->bvhParser, this->bvh_frame);

            double R_Tibia_angle = this->ms->getSkeleton()->getBodyNode("R_Tibia")->getParentJoint()->getPositions()[0];
            double R_ForeArm_angle = this->ms->getSkeleton()->getBodyNode("R_ForeArm")->getParentJoint()->getPositions()[0];
            double L_Tibia_angle = this->ms->getSkeleton()->getBodyNode("L_Tibia")->getParentJoint()->getPositions()[0];
            double L_ForeArm_angle = this->ms->getSkeleton()->getBodyNode("L_ForeArm")->getParentJoint()->getPositions()[0];

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
                this->UpdateMuscleMINMAX_l_mt_list();
            }
            else
            {
                // if(R_Tibia_angle <0)
                //     cout<<"R_Tibia : "<<R_Tibia_angle<<endl;
                // if(R_ForeArm_angle <0)
                //     cout<<"R_Tibia : "<<R_ForeArm_angle<<endl;
                // if(L_Tibia_angle <0)
                //     cout<<"L_Tibia : "<<L_Tibia_angle<<endl;
                // if(L_ForeArm_angle <0)
                //     cout<<"L_ForeArm : "<<L_ForeArm_angle<<endl;
                // this->UpdateMuscleMINMAX_l_mt_list();
            }

            this->UpdateMuscleMINMAX_l_mt_list();
            this->bvh_frame += 10;
            // cout<<nnChecker->checkAvailable(this->ms, NN_TARGET::RIGHTLEG);
        }
        this->ms->getSkeleton()->setPositions(originPos);

        // LeftRightSyncAdjustmentValue();

        cout<<endl;

        cout<<"# writing file ../generator/motionAdjust/adjustValue/max_min_l_mt_tilda"+label_name+".txt"<<endl;

        FILE* out1=fopen(("../generator/motionAdjust/adjustValue/max_min_l_mt_tilda"+label_name+".txt").c_str(),"w");
        for(int i=0;i<this->ms->getNumMuscles();i++)
        {
            fprintf(out1, "%s %f %f", ms->getMuscles()[i]->name.c_str(), muscle_min_l_mt_list[i], muscle_max_l_mt_list[i]);
            fprintf(out1, "\n");
        }
        fclose(out1);


        // fclose(out1);
        //remove out the present filefalse

        // FILE* out=fopen(("../generator/motionAdjust/adjustValue/adjustment_bvh"+label_name+".txt").c_str(),"w");
        // fclose(out);
        // for(int i =0;i<this->ms->getNumMuscles();i++)
        // { 
        //     auto& muscle = this->ms->getMuscles()[i];
        //     WriteAdjustmentValueMIN_MAX_LMT(("bvh"+label_name).c_str(), muscle, 
        //     muscle->min_l_m_tilda, muscle->max_l_m_tilda, 
        //     muscle_min_l_mt_list[i], muscle_max_l_mt_list[i], 
        //     false, true);
        // }
    }
    
    if(apply)
    {
        std::string filePath = "../generator/motionAdjust/adjustValue/max_min_l_mt_tilda";
        filePath = filePath + label_name + ".txt";
        string line;
        stringstream s;
        ifstream in(filePath);
        std::cout<<"# reading file "<<filePath<<std::endl;
        string muscleName;
        float a_value, b_value;
        while(!in.eof())
        {
            getline(in, line);
            if(line.empty())
                break;
            s = stringstream(line);
            s>>muscleName >> a_value >> b_value;
            for(int i =0;i< ms->getNumMuscles();i++)
            {
                if(ms->getMuscles()[i]->name == muscleName)
                {
                    ms->getMuscles()[i]->max_l_mt = b_value;
                    ms->getMuscles()[i]->min_l_mt = a_value;
                    // cout<<muscleName<<" "<<ms->getMuscles()[i]->max_l_mt<<endl;
                }
            }
        }
        in.close();
        // cout<<"in motion adjust"<<endl;
        // for(auto& muscle : this->ms->getMuscles())
        // {
        //     // cout<<muscle->max_l_mt<<endl;
        // }

    }
}
