//
// Created by minseok on 11/14/18.
//

#ifndef MUSCLE_CAL_H
#define MUSCLE_CAL_H
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"
#include "../../model/MusculoSkeletalSystem.h"

class MusculoSkeletalSystem;

namespace MuscleCal{
	double g_t_tilda(double eps_t);
	double g_t(std::shared_ptr<Muscle> muscle, double l_t, double a_val);
	double f_t(std::shared_ptr<Muscle> muscle, double l_t, double a_val);
	double g_pl_tilda_o(double l_m_tilda);
	double g_pl_tilda(double l_m_tilda);
	double g_pl(std::shared_ptr<Muscle> muscle, double l_m, double a_val, double b_val = 1.0);
	double f_pl(std::shared_ptr<Muscle> muscle, double l_m, double a_val, double b_val = 1.0);

	double compute_a_val(std::shared_ptr<Muscle> muscle, bool hard_max = true);
	double compute_a_val(std::shared_ptr<Muscle> muscle, double l_mt, bool hard_max);
	void   compute_a_b_val(std::shared_ptr<Muscle> muscle, 
	    double target_l_m_tilda_min,    double target_l_m_tilda_max,
	    double l_mt_min,                double l_mt_max,
	    double* a,                      double* b,
	    bool hard_min=true,             bool hard_max=true);
};
#endif