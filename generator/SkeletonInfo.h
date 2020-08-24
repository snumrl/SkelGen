//
// Created by hoseok on 10/29/18.
//

#ifndef SKELGEN_SKELETONINFO_H
#define SKELGEN_SKELETONINFO_H

#include <string>

/**
 * Information about retargeting. Every parameter is real world information.
 */
class MetaBoneInfo{
public:
    MetaBoneInfo():name(""), alpha_lengthening(1.0), alpha_scaling(1.0), theta_proximal(0.0), theta_distal(0.0){}
    MetaBoneInfo(std::string name, double l, double s, double t):name(name),alpha_lengthening(l), alpha_scaling(s), theta_distal(t) {}
    std::string name;
    double alpha_lengthening, alpha_scaling; // ratio; 1.0 is default
    double theta_proximal, theta_distal; // degree unit
    int side; // 0 as right, 1 as left
};
class SkeletonInfo {
public:
    class BoneInfo{
    public:
        BoneInfo(){
            // Method 1: measure zygote model
            // zygote model height: 175.0 cm
            FemurLength = 44.53391; // (cm), 0.4453391 cm in zygote
            TibiaLength = 42.49466; // (cm), 0.4249466 cm in zygote
            HumerusLength = 33.24388; // (cm), 0.3324388 cm in zygote
            RadiusLength = 26.98825; // (cm), 0.2698825 cm in zygote
            SpineLength = 0;
            Torso = 1;

            /*
            // Method 2
            // reference paper [2]
            FemurLength = 45.85; // (cm), 26.2% of height
            TibiaLength = 39.025; // (cm), 22.3% of height
            HumerusLength = 32.9; // (cm), 18.8% of height
            RadiusLength = 26.425; // (cm), 15.1% of height
            */
        }

        double FemurLength, TibiaLength, HumerusLength, RadiusLength, SpineLength, Torso;

        // scale from "cm" to dart world
        double getFemurLength() const {
            // 44.53391 cm : 0.390
            return 0.4043;
        }

        // scale from "cm"(cm) to dart world
        double getTibiaLength() const {
            // 42.49466 cm : 0.460
            return 0.4156;
        }

        // scale from "cm" to dart world
        double getHumerusLength() const {
            // 33.24388 cm : ?
            return HumerusLength;
        }

        // scale from "cm" to dart world
        double getRadiusLength() const {
            // 26.98825 cm : ?
            return RadiusLength;
        }

        void setFemurLength(double FemurLength) {           // unit of input parameter must be "cm"
            BoneInfo::FemurLength = FemurLength;
        }

        void setTibiaLength(double TibiaLength) {           // unit of input parameter must be "cm"
            BoneInfo::TibiaLength = TibiaLength;
        }

        void setHumerusLength(double HumerusLength) {       // unit of input parameter must be "cm"
            BoneInfo::HumerusLength = HumerusLength;
        }

        void setRadiusLength(double RadiusLength) {         // unit of input parameter must be "cm"
            BoneInfo::RadiusLength = RadiusLength;
        }
    };

    SkeletonInfo(){
        height = 175.0; // (cm)
        weight = 82.7; // (kg)
	    boneInfo = BoneInfo();
	    R_Femur = MetaBoneInfo(), L_Femur = MetaBoneInfo(), R_Tibia = MetaBoneInfo(), L_Tibia = MetaBoneInfo();
        symmetry = true;
    }

    // parameters for skeleton
    double height, weight;

    void setHeight(double height);

    void setWeight(double weight);

	double getR_FemurLength(){ return 0.4043 * R_Femur.alpha_lengthening; }
	double getL_FemurLength(){ return 0.4043 * L_Femur.alpha_lengthening; }

	double getR_TibiaLength(){ return 0.4156 * R_Tibia.alpha_lengthening; }
	double getL_TibiaLength(){ return 0.4156 * L_Tibia.alpha_lengthening; }

	double getSpineLength() { return 0.0976 * Spine.alpha_lengthening; }

    void setMetaBoneInfo(MetaBoneInfo _R_Femur, MetaBoneInfo _L_Femur, MetaBoneInfo _R_Tibia, MetaBoneInfo _L_Tibia){
        R_Femur=_R_Femur, L_Femur=_L_Femur, R_Tibia=_R_Tibia, L_Tibia=_L_Tibia;
    }

    void clear();

    bool symmetry;
    BoneInfo boneInfo;

    MetaBoneInfo R_Femur, L_Femur, R_Tibia, L_Tibia, Spine;

};


#endif //SKELGEN_SKELETONINFO_H
