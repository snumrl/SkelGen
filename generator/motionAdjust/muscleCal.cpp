//
// Created by minseok on 11/14/18.
//
#include "muscleCal.h"
#include <math.h>
#include <stdio.h>

class MusculoSkeletalSystem;

double MuscleCal::g_t_tilda(double eps_t){
    double f_toe_tilda = 0.33;
    double k_toe = 3.0;
    double eps_t_0 = 0.033;
    double eps_t_toe = 0.609 * eps_t_0;
    double k_lin = 1.712;

    // if (eps_t <= eps_t_toe){
        return f_toe_tilda / (exp(k_toe) - 1) * (exp(k_toe * eps_t / eps_t_toe) - 1);
    // }else{
    //     return k_lin * (eps_t - eps_t_toe) + f_toe_tilda;
    // }
}

double MuscleCal::g_t(std::shared_ptr<Muscle> muscle, double l_t, double a_val){
    return muscle->f_m_o * MuscleCal::g_t_tilda((l_t - (muscle->l_t_sl*a_val)) / (muscle->l_t_sl*a_val));
}

double MuscleCal::f_t(std::shared_ptr<Muscle> muscle, double l_t, double a_val){
    return MuscleCal::g_t(muscle, l_t, a_val);
}

double MuscleCal::g_pl_tilda_o(double l_m_tilda){
    double k_pe = 4.0;
    double eps_m_o = 0.6;
    return (exp(k_pe * (l_m_tilda - 1) / eps_m_o) - 1) / (exp(k_pe) - 1);
}

double MuscleCal::g_pl_tilda(double l_m_tilda){
    if (l_m_tilda <= 1.0) return 0;
    else return MuscleCal::g_pl_tilda_o(l_m_tilda);
}

double MuscleCal::g_pl(std::shared_ptr<Muscle> muscle, double l_m, double a_val, double b_val) {
    return muscle->f_m_o * MuscleCal::g_pl_tilda(l_m / (muscle->l_m_o* a_val*b_val));
}

double MuscleCal::f_pl(std::shared_ptr<Muscle> muscle, double l_m, double a_val, double b_val){
    return MuscleCal::g_pl(muscle, l_m, a_val, b_val)* cos(muscle->pen_angle);
}


double MuscleCal::compute_a_val(std::shared_ptr<Muscle> muscle, bool hard_max){
    double target_l_m_tilda = muscle->max_l_m_tilda;
    double l_mt = muscle->getLength();
    double l=0, r=l_mt, mid=100000;
    double a_val = 1.0;
    if(hard_max)
    {
        while(fabs(mid/(muscle->l_m_o*a_val) - target_l_m_tilda)>0.01)
        {
            a_val += 0.001 * (mid/(muscle->l_m_o*a_val) - target_l_m_tilda)/fabs(mid/(muscle->l_m_o*a_val) - target_l_m_tilda);
            l=0, r=l_mt, mid=100000;
            while (r-l > 1e-6){
                mid = (l + r) / 2.0;
                double res = MuscleCal::f_pl(muscle, mid, a_val) - MuscleCal::f_t(muscle, l_mt - mid, a_val);
                if (res>=0) r=mid;
                else l=mid;
            }
            if(fabs(mid/(muscle->l_m_o*a_val) - target_l_m_tilda)>0.01)
            {
                a_val += 0.001 * (mid/(muscle->l_m_o*a_val) - target_l_m_tilda)/
                    fabs(mid/(muscle->l_m_o*a_val) - target_l_m_tilda);
            }

        }
        return a_val;
    }
    else
    {
        while(mid/(muscle->l_m_o*a_val) > target_l_m_tilda)
        {
            l=0, r=l_mt, mid=100000;
            while (r-l > 1e-6){
                mid = (l + r) / 2.0;
                double res = MuscleCal::f_pl(muscle, mid, a_val) - MuscleCal::f_t(muscle, l_mt - mid, a_val);
                if (res>=0) r=mid;
                else l=mid;
            }
            if(mid/(muscle->l_m_o*a_val) > target_l_m_tilda)
                a_val += 0.001;
        }
        return a_val;
    }
    
}
double MuscleCal::compute_a_val(std::shared_ptr<Muscle> muscle, double l_mt, bool hard_max){
    double target_l_m_tilda = muscle->max_l_m_tilda;
    double l=0, r=l_mt, mid=100000;
    double a_val = 1.0;
    if(hard_max)
    {
        while(fabs(mid/(muscle->l_m_o*a_val) - target_l_m_tilda)>0.01)
        {
            a_val += 0.0005 * (mid/(muscle->l_m_o*a_val) - target_l_m_tilda)/fabs(mid/(muscle->l_m_o*a_val) - target_l_m_tilda);
            l=0, r=l_mt, mid=100000;
            while (r-l > 1e-6){
                mid = (l + r) / 2.0;
                double res = MuscleCal::f_pl(muscle, mid, a_val) - MuscleCal::f_t(muscle, l_mt - mid, a_val);
                if (res>=0) r=mid;
                else l=mid;
            }
            // if(fabs(mid/(muscle->l_m_o*a_val) - target_l_m_tilda)>0.02)
            // {
            //     a_val += 0.001 * (mid/(muscle->l_m_o*a_val) - target_l_m_tilda)/
            //         fabs(mid/(muscle->l_m_o*a_val) - target_l_m_tilda);
            // }

        }
        // std::cout<<"##"<<std::endl;
        // std::cout<<(l_mt -mid)/(muscle->l_t_sl*a_val)<<" "<<(mid)/(muscle->l_m_o*a_val)<<std::endl;
        return a_val;
    }
    else
    {
        while(mid/(muscle->l_m_o*a_val) > target_l_m_tilda)
        {
            l=0, r=l_mt, mid=100000;
            while (r-l > 1e-6){
                mid = (l + r) / 2.0;
                double res = MuscleCal::f_pl(muscle, mid, a_val) - MuscleCal::f_t(muscle, l_mt - mid, a_val);
                if (res>=0) r=mid;
                else l=mid;
            }
            if(mid/(muscle->l_m_o*a_val) > target_l_m_tilda)
                a_val += 0.001;
        }
        return a_val;
    }
    
}

void MuscleCal::compute_a_b_val(std::shared_ptr<Muscle> muscle, 
    double target_l_m_tilda_min,    double target_l_m_tilda_max,
    double l_mt_min,                double l_mt_max,
    double* a,                      double* b,
    bool hard_min,             bool hard_max){
    if(!hard_min && hard_max)
    {
        double a_val = 1.0, b_val = 1.0;
        while(1)
        {
            double l=0, r=l_mt_max, mid=100000;
            while(fabs(mid/(muscle->l_m_o*a_val*b_val) - target_l_m_tilda_max)>0.01)
            {
                l=0, r=l_mt_max, mid=100000;
                while (r-l > 1e-6){
                    mid = (l + r) / 2.0;
                    double res = MuscleCal::f_pl(muscle, mid, a_val, b_val) - MuscleCal::f_t(muscle, l_mt_max - mid, a_val);
                    if (res>=0) r=mid;
                    else l=mid;
                }
                if(fabs(mid/(muscle->l_m_o*a_val*b_val) - target_l_m_tilda_max)>0.01)
                {
                    a_val += 0.001 * (mid/(muscle->l_m_o*a_val*b_val) - target_l_m_tilda_max)/
                        fabs(mid/(muscle->l_m_o*a_val*b_val) - target_l_m_tilda_max);
                }
            }

            l=0, r=l_mt_min, mid=100000;
            while (r-l > 1e-6){
                mid = (l + r) / 2.0;
                double res = MuscleCal::f_pl(muscle, mid, a_val, b_val) - MuscleCal::f_t(muscle, l_mt_min - mid, a_val);
                if (res>=0) r=mid;
                else l=mid;
            }
            if(mid/(muscle->l_m_o*a_val*b_val) - target_l_m_tilda_min<-0.01)
            {
                b_val -= 0.001 * (mid/(muscle->l_m_o*a_val*b_val) - target_l_m_tilda_min)/fabs(mid/(muscle->l_m_o*a_val*b_val) - target_l_m_tilda_min);
            }
            else 
            {
                *a = a_val;
                *b = b_val; 
                break;
            }
        }
    }
    else if(!hard_max && !hard_min)
    {
        double a_val = 1.0, b_val = 1.0;
        while(1)
        {
            double l=0, r=l_mt_max, mid=100000;
            while(mid/(muscle->l_m_o*a_val*b_val) - target_l_m_tilda_max > 0)
            {
                l=0, r=l_mt_max, mid=100000;
                while (r-l > 1e-6){
                    mid = (l + r) / 2.0;
                    double res = MuscleCal::f_pl(muscle, mid, a_val, b_val) - MuscleCal::f_t(muscle, l_mt_max - mid, a_val);
                    if (res>=0) r=mid;
                    else l=mid;
                }
                if(mid/(muscle->l_m_o*a_val*b_val)>target_l_m_tilda_max)
                {
                    a_val += 0.001 * (mid/(muscle->l_m_o*a_val*b_val) - target_l_m_tilda_max)/
                    fabs(mid/(muscle->l_m_o*a_val*b_val) - target_l_m_tilda_max);
                }
            }

            l=0, r=l_mt_min, mid=100000;
            while (r-l > 1e-6){
                mid = (l + r) / 2.0;
                double res = MuscleCal::f_pl(muscle, mid, a_val, b_val) - MuscleCal::f_t(muscle, l_mt_min - mid, a_val);
                if (res>=0) r=mid;
                else l=mid;
            }
            if(mid/(muscle->l_m_o*a_val*b_val) - target_l_m_tilda_min<-0.01)
            {
                b_val -= 0.001 * (mid/(muscle->l_m_o*a_val*b_val) - target_l_m_tilda_min)/fabs(mid/(muscle->l_m_o*a_val*b_val) - target_l_m_tilda_min);
            }
            else 
            {
                *a = a_val;
                *b = b_val; 
                break;
            }
        }
    }
    else {
        std::cout<<"Not implemented yet"<<std::endl;
        exit(1);
    }

}
