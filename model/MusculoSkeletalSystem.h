#ifndef __MUSCULO_SKELETAL_SYSTEM_H__
#define __MUSCULO_SKELETAL_SYSTEM_H__
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"
#include "DART_helper.h"


typedef std::pair<dart::dynamics::BodyNode*,Eigen::Vector3d> AnchorPoint;
Eigen::Vector3d getAnchorPointGlobalCoord(const AnchorPoint &ap);

const double e_t_o = 0.033;
const double e_m_o = 0.6;
const double k_pe = 4.0;
const double gam = 0.5;
const double l_dot_max_tilda = 10.0;
const double f_m_len_tilda = 1.8;
const double A_f = 0.3;

struct BlendedWaypoint{
public:
	BlendedWaypoint(){}
	BlendedWaypoint(AnchorPoint ap):ap(ap){}

	void calculateWeight(Eigen::Vector3d globalCoord){

	}

	void pushBodyNode(dart::dynamics::BodyNode* bn){
		for (int i=0;i<relatedBodyNodes.size();i++) if (relatedBodyNodes[i] == bn) return;
		relatedBodyNodes.emplace_back(bn);
	}

	void setWeights(std::vector<double>& _weights){
		weights = _weights;
	}

	AnchorPoint ap;
	std::vector<dart::dynamics::BodyNode*> relatedBodyNodes;
	std::vector<Eigen::Vector3d> relativeCoords;
	std::vector<double> weights;

};
Eigen::Vector3d getBlendedWaypointGlobalCoord(const BlendedWaypoint &bwp);

struct Muscle
{
	Muscle(){};
	int getNumForces();
	Eigen::MatrixXd getJacobianTranspose();
	void transferForce(const double f_mt, Eigen::VectorXd &f);
	void setActivationLevel(double a);
	void initialize();
	double getPassiveForce(){ return mPassiveForce; }
	double getLength();
	double getLmtilda();
	double getLm();
	double f_t(double l_t), g_t(double l_t);
	double f_pl(double l_m), g_pl(double l_m);
	double f_al(double l_m), g_al(double l_m);
	double compute_f_mt();
	void applyForce();
	void releaseMuscleParameterByPassiveTorque();

	std::pair<Eigen::VectorXd,Eigen::VectorXd> getForceJacobianAndPassive();
	std::string 										name;
	std::vector<int>							        waypoint_indices;
	std::vector<BlendedWaypoint>						blendedWaypoints;
	double												activation_level;
	double												mPassiveForce;
	double                                              f_m_o, l_m_o, l_t_sl, pen_angle; // input parameter
	double												l_m, l_t;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	// saving
	double												l_m_tilda;

	double												max_l_m_tilda, min_l_m_tilda;

	//maximum extended length
	double												max_l_mt;
	double												min_l_mt;

	//Skeleton Position when maximum extended
	Eigen::VectorXd										max_l_mt_position;

	std::string											max_l_mt_bvh;
	int													max_l_mt_bvh_frame;
};

class MusculoSkeletalSystem
{
public:
	MusculoSkeletalSystem();
	void 									addMuscle(const std::string &name,
													  const std::vector<int> &waypoint_indices,
	                                                  const double f_m_o,
	                                                  const double l_m_o,
	                                                  const double l_t_sl,
	                                                  const double pen_angle);

	void 									initialize(const dart::simulation::WorldPtr &rigid_world);

	void 									setActivationLevels(const Eigen::VectorXd &a);
	void 									transformAttachmentPoints();
	void 									applyForcesToSkeletons(const dart::simulation::WorldPtr rigid_world);

	void 									computeMuscleForce(std::shared_ptr<Muscle> muscle, double *f_tilda);
	void 									computeMuscleForceActive(std::shared_ptr<Muscle> muscle, double *f_tilda);
	void 									computeMuscleForcePassive(std::shared_ptr<Muscle> muscle, double *f_tilda);
	void 									computeForceDerivative(const dart::simulation::WorldPtr rigid_world,
	                                                               Eigen::SparseMatrix<double> &J);
	Eigen::VectorXd 						computeForce(const dart::simulation::WorldPtr rigid_world = nullptr);
	Eigen::VectorXd 						computeForceActive(const dart::simulation::WorldPtr rigid_world = nullptr);
	Eigen::VectorXd 						computeForcePassive(const dart::simulation::WorldPtr rigid_world = nullptr);
	
	Eigen::VectorXd							updateForceActive(Eigen::VectorXd active, int muscleIndex);
	Eigen::VectorXd							updateForcePassive(Eigen::VectorXd passive, int muscleIndex);

	void 									computeOriginPoint(std::shared_ptr<Muscle> muscle, Eigen::Vector3d &p);
	void 									computeInsertionPoint(std::shared_ptr<Muscle> muscle, Eigen::Vector3d &p);

	Eigen::MatrixXd 						getJacobianTranspose();

	int 									getNumMuscles() 		{return mMuscles.size();}
	std::vector<AnchorPoint>&    			getWaypoints()  		{return mWaypoints;}
	std::vector<BlendedWaypoint>&           getBlendedWaypoints()   {return mBlendedWaypoints;}
	std::vector<std::shared_ptr<Muscle>>&	getMuscles()			{return mMuscles;}
	dart::dynamics::SkeletonPtr&			getSkeleton()			{return mSkeleton;}
	Eigen::VectorXd 						getActivationLevels()	{return mActivationLevels;}
	int	 									getNumMuscleForces();

	void									SetMax_l_m_tilda(double max_val);
	void									SetMin_l_m_tilda(double min_val);
	void									ApplyAdjustmentValue(const char *name);
	void									ApplyAdjustmentValue_FiberTendon(const char *name);

	/// update bwp.ap as newly version + calculate blending weight (must execute after setting related bodynodes)
	void                                    calculateSingleBlendingWeights(int bwpIdx);
	void                                    calculateTotalBlendingWeights();

	/// AnchorPoints(MSS->mWaypoints)를 옮겼으니까 BlendedWaypoints를 업데이트 하고, muscle들도 업데이트 하는 함수
	void                                    updateSingleWaypoints(int waypointIdx);
	void                                    updateTotalWaypoints();

	/// muscle에 있는 blended waypoint를 최신 버전으로 업데이트하는 함수
	void                                    updateBlendedWaypoints();

	/// Save muscle XML after optimizing
	void                                    saveMuscleXML(std::string fileName,double scale = -1.0, Eigen::Vector3d _p = Eigen::Vector3d(0,0,0));

	/// Skeleton text extract
	void                                    saveSkeletonText(double scale = -1.0, Eigen::Vector3d _p = Eigen::Vector3d(0,0,0));

	/// Skeleton Xml Extract
	void                                    saveSkeletonXml(std::string filename,double scale = -1.0, Eigen::Vector3d _p = Eigen::Vector3d(0,0,0));

	/// set skeleton pose as given configuration
	void                                    setPose(double *config);

	int                                     getBodyNodeIndex(std::string body_name);
	Eigen::Vector3d                         getUAxis(std::string body_name);
	Eigen::Vector3d                         getVAxis(std::string body_name);
	Eigen::Vector3d                         getJtoBAxis(std::string body_name);

	bool									checkPosAvailable();

	void									SetZeroPose();


	std::vector<bool>                       originFlag;
	std::vector<bool>                       insertionFlag;
	std::vector<bool>                       usedFlag;
	std::vector< std::vector<int> >         con; // con[i][j]=1 : i-th waypoint and j-th waypoint are connected by some muscle.

	std::vector<AnchorPoint>                mWaypoints;
	std::vector<BlendedWaypoint>            mBlendedWaypoints;
//private:
	//Muscles and Skeleton
	std::vector<std::shared_ptr<Muscle>>	mMuscles;
	dart::dynamics::SkeletonPtr 			mSkeleton;
	std::vector<Eigen::AngleAxisd>          mCoordinateSystemAngleAxis;

	//Material Properties
	double	mTendonStiffness;
	double	mMuscleStiffness;
	double	mYoungsModulus;
	double	mPoissonRatio;

	Eigen::VectorXd							mActivationLevels;

	void									updateMuscleParametersByMaxLmt();
	void									updateMuscleParametersByMaxLmtFast();

	void									releaseMuscleParametersByPassiveTorque();


	void									applyForces(Eigen::VectorXd& activationLevels);
	void									applyForces();

	int 									getForcesIndex(int muscleIndex);
	// int 									getNumForcesIndex(std::shared_ptr<Muscle> muscle);

};

#endif