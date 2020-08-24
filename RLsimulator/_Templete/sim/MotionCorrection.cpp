#include "MotionCorrection.h"
#include <IpIpoptData.hpp>

using namespace MSS;
using namespace Ipopt;
using namespace dart::dynamics;
using namespace dart::simulation;

MotionCorrection::
MotionCorrection(Character* c,const dart::simulation::WorldPtr& w)
	:mCharacter(c),mWorld(w)
{

}
void
MotionCorrection::
Setq0(const Eigen::VectorXd& q0)
{
	mq0 = q0;

	mCharacter->GetSkeleton()->setPositions(mq0);
	mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);
	for(int i =0;i<mCharacter->GetMuscles().size();i++)
		mCharacter->GetMuscles()[i]->Update();
	mx0.clear();
	for(int i=0;i<mCharacter->GetEndEffectors().size();i++){
		mx0.push_back(mCharacter->GetEndEffectors()[i]->getCOM());
	}
	
	mCOM0 = 0.5*(mx0[0] + mx0[1]);
	mCOM0[2] += 0.1;
	// std::cout<<mCOM0.transpose()<<std::endl;
}
bool
MotionCorrection::
get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style) 
{
	n = mCharacter->GetSkeleton()->getNumDofs();
	m = mCharacter->GetMuscleLimitConstraints().size();
	nnz_jac_g = m*n;
	nnz_h_lag = n;
	index_style = TNLP::C_STYLE;
	return true;
}
bool
MotionCorrection::
get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u) 	
{
	for(int i=0;i<n;i++)
	{
		x_l[i] = mCharacter->GetSkeleton()->getDof(i)->getPositionLowerLimit();
		x_u[i] = mCharacter->GetSkeleton()->getDof(i)->getPositionUpperLimit();
	}
	// for(int i=0;i<3;i++)
	// {
	// 	x_l[i] = -1.5;
	// 	x_u[i] = 1.5;	
	// }

	// for(int i=3;i<6;i++)
	// {
	// 	x_l[i] = -0.3;
	// 	x_u[i] = 0.3;	
	// }
	
	for(int i=0;i<m;i++)
	{
		g_l[i] = 0.0;
		g_u[i] = 1E6;
	}
	return true;
}
bool
MotionCorrection::
get_starting_point(	Ipopt::Index n, bool init_x, Ipopt::Number* x,bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,Ipopt::Index m, bool init_lambda,Ipopt::Number* lambda) 
{
	
	// std::cout<<mq0.transpose()<<std::endl;
	
	// mq0.setZero();
	if(init_x)
	{
		// mq0 = mCharacter->GetSkeleton()->getPositions();
		for(int i=0;i<n;i++)
			x[i] = mq0[i];


	}
	return true;
}
bool
MotionCorrection::
eval_f(	Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value) 
{
	Eigen::VectorXd q(n);
	for(int i=0;i<n;i++)
		q[i] = x[i];

	mCharacter->GetSkeleton()->setPositions(q);
	mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);
	for(int i =0;i<mCharacter->GetMuscles().size();i++)
		mCharacter->GetMuscles()[i]->Update();
	obj_value = 0;
	for(int i=0;i<mx0.size();i++)
	{
		obj_value += 1.0/10.0*0.5*(mx0[i]-mCharacter->GetEndEffectors()[i]->getCOM()).squaredNorm();
		// std::cout<<(mx0[i]-mCharacter->GetEndEffectors()[i]->getCOM()).transpose()<<std::endl;
	}
	// std::cout<<std::endl;
	auto root = mCharacter->GetSkeleton()->getRootBodyNode();

	Eigen::Vector3d root_diff = root->getCOM()-mCOM0;
	root_diff[1] = 0.0;
	obj_value += 0.5*1.0/10.0*root_diff.squaredNorm();
	obj_value += 0.5*0.03*(q-mq0).squaredNorm();
	
	
	// std::cout<<obj_value<<std::endl;
	// std::cout<<q.transpose()<<std::endl;
	return true;
}
bool
MotionCorrection::
eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f) 
{
	Eigen::VectorXd q = Eigen::VectorXd::Zero(n);
	Eigen::VectorXd g = Eigen::VectorXd::Zero(n);
	for(int i =0;i<n;i++)
		q[i] = x[i];
	
	mCharacter->GetSkeleton()->setPositions(q);
	mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);
	for(int i =0;i<mCharacter->GetMuscles().size();i++)
		mCharacter->GetMuscles()[i]->Update();
	for(int i=0;i<mx0.size();i++)
	{
		auto ee = mCharacter->GetEndEffectors()[i];
		dart::math::LinearJacobian J = mCharacter->GetSkeleton()->getLinearJacobian(ee,Eigen::Vector3d(0,0,0));
		// std::cout<<J.rows()<<" "<<J.cols()<<std::endl;
		Eigen::MatrixXd J_inv = J.transpose()*(J*J.transpose()).inverse();
		g += 1.0/10.0*J_inv*(mCharacter->GetEndEffectors()[i]->getCOM()-mx0[i]);
		// Eigen::RowVectorXd g_row = (mCharacter->GetEndEffectors()[i]->getCOM()-mx0[i]).transpose()*J;
		// std::cout<<g_row<<std::endl;
		// Eigen::MatrixXd dddd = J.transpose()*(mCharacter->GetEndEffectors()[i]->getCOM()-mx0[i]);
		// std::cout<<dddd.rows()<<" "<<dddd.cols()<<std::endl;
		// g += J.transpose()*(mCharacter->GetEndEffectors()[i]->getCOM()-mx0[i]);
	}

	auto root = mCharacter->GetSkeleton()->getRootBodyNode();
	dart::math::LinearJacobian J = mCharacter->GetSkeleton()->getLinearJacobian(root,Eigen::Vector3d(0,0,0));

	Eigen::MatrixXd J_inv = J.transpose()*(J*J.transpose()).inverse();
	Eigen::Vector3d root_diff = root->getCOM()-mCOM0;
	root_diff[1] = 0.0;
	g += 1.0/10.0*J_inv*(root_diff);
	// std::cout<<"root : "<<(1.0/50.0*J_inv*(root_diff)).norm()<<std::endl;
	// std::cout<<g.norm()<<std::endl;
	g += 0.03*(q-mq0);
	// std::cout<<"reg : "<<(q-mq0).norm()<<std::endl;
	for(int i =0;i<n;i++)
		grad_f[i] = g[i];
	return true;
}
bool
MotionCorrection::
eval_g(	Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g) 
{
	Eigen::VectorXd q = Eigen::VectorXd::Zero(n);
	for(int i =0;i<n;i++)
		q[i] = x[i];
	
	mCharacter->GetSkeleton()->setPositions(q);
	mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);
	for(int i =0;i<mCharacter->GetMuscles().size();i++)
		mCharacter->GetMuscles()[i]->Update();
	// std::cout<<"Violation : ";
	for(int i =0;i<m;i++){
		g[i] = mCharacter->GetMuscleLimitConstraints()[i]->EvalObjective();	
		// std::cout<<g[i]<<" ";
	}
	// std::cout<<std::endl;
	
	return true;
}
bool
MotionCorrection::
eval_jac_g( Ipopt::Index n, const Ipopt::Number* x, bool new_x,Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index *jCol,Ipopt::Number* values) 
{
	int nnz = 0;
	if(values == NULL)
	{
		for(int i =0;i<m;i++)
		{
			for(int j =0;j<n;j++)
			{
				iRow[nnz] = i;
				jCol[nnz++] = j;
			}
		}
	}
	else
	{
		Eigen::VectorXd q = Eigen::VectorXd::Zero(n);
		
		mCharacter->GetSkeleton()->setPositions(q);
		mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);
		for(int i =0;i<mCharacter->GetMuscles().size();i++)
			mCharacter->GetMuscles()[i]->Update();
		for(int i =0;i<m;i++)
		{
			Eigen::VectorXd dci_dq = mCharacter->GetMuscleLimitConstraints()[i]->EvalGradient();
			for(int j =0;j<n;j++)
			{
				values[nnz++] = dci_dq[j];
			}
			// std::cout<<dci_dq.norm()<<std::endl;
		}
	}
	// std::cout<<std::endl;
	return true;
}
bool
MotionCorrection::
eval_h( Ipopt::Index n, const Ipopt::Number* x, bool new_x,Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,Ipopt::Index* jCol, Ipopt::Number* values) 
{
	int nnz = 0;

	if(values == NULL)
	{
		for(int i =0;i<n;i++)
		{
			iRow[nnz] = i;
			jCol[nnz++] = i;
		}

	}
	else
	{
		for(int i =0;i<n;i++)
			values[nnz++] = 1.0;
	}
	return true;
}
void 
MotionCorrection::
finalize_solution(	Ipopt::SolverReturn status,Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U,Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda,Ipopt::Number obj_value,const Ipopt::IpoptData* ip_data,Ipopt::IpoptCalculatedQuantities* ip_cq)
{
	// std::cout<<ip_data->iter_count()<<std::endl;
	mSolution = Eigen::VectorXd::Zero(n);
	for(int i =0;i<n;i++)
		mSolution[i] = x[i];
}