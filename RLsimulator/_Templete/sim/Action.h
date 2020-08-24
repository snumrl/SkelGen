#ifndef __MSS_MODE_H__
#define __MSS_MODE_H__
#include "dart/dart.hpp"
namespace MSS
{
class MotionAction
{
public:
	std::string name;
	double lb;
	Eigen::VectorXd a;

	MotionAction(const std::string& _name,const Eigen::VectorXd& upper,const Eigen::VectorXd& lower);
	void SetLB(double _lb){lb = _lb;};
	void Set(Eigen::VectorXd& p){p+=lb*a;};
};

};


#endif
