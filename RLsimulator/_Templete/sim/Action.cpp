#include "Action.h"
namespace MSS
{
MotionAction::
MotionAction(const std::string& _name,const Eigen::VectorXd& upper,const Eigen::VectorXd& lower)
	:name(_name),lb(0.0),a(0.5*(upper-lower))
{
	for(int i =0;i<a.rows();i++)
		a[i] = dart::math::clip(a[i],-1.0,1.0);
}
};