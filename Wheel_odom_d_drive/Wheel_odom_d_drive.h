#ifndef WHEEL_ODOM_D_DRIVE_H
#define WHEEL_ODOM_D_DRIVE_H

#include <math.h>
#include "Math_functions.h"

class Wheel_odom_d_drive{
    public:
        Wheel_odom_d_drive();
        void set_param(long N_, double N_g_, double r_, double L_);
        void set_dt(double dt_);
        void update(long nr, long nl);
        void get_wheel_speed(double* wr, double* wl);
        void get_twist(double* vc, double* wc);
        void get_pose(double* xc, double* yc, double* thc);
        void set_PPR(long N_);
        void set_gear_ratio(double N_g_);
        void set_r(double r_);
        void set_L(double L_);
        unsigned long get_PPR();
        double get_gear_ratio();
        double get_r();
        double get_L();
        double get_dt();

    private:
        unsigned long N = 133600;
        double N_g = 1.0;
        double r = 0.0625;
        double L = 0.265;

        double dt = 0.004;
        long N_L, N_R;
        long N_L_pre, N_R_pre;
        double w_L, w_R;
        double v;
        double w;
        double x;
        double y;
        double th;

        Math_functions math;
};

#endif
