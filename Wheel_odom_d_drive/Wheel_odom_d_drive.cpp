#include "Wheel_odom_d_drive.h"

Wheel_odom_d_drive::Wheel_odom_d_drive(){
}

void Wheel_odom_d_drive::set_param(long N_, double N_g_, double r_, double L_){
    N = N_;
    N_g = N_g_;
    r = r_;
    L = L_;
}

void Wheel_odom_d_drive::set_dt(double dt_){
    dt = dt_;
}

void Wheel_odom_d_drive::update(long nr, long nl){
    N_L = nl;
    N_R = nr;
    w_L = const_math_2pi*(double)(N_L - N_L_pre)/((double)N*N_g*4.0*dt);
    w_R = const_math_2pi*(double)(N_R - N_R_pre)/((double)N*N_g*4.0*dt);
    v = (r/2.0)*(w_L+w_R);
    w = (r/L)*(w_R-w_L);

    double dl = (const_math_pi*r*(double)((N_R-N_R_pre) + (N_L-N_L_pre)))/((double)N*N_g*4.0);
    double dth = (const_math_2pi*r*(double)((N_R-N_R_pre) - (N_L-N_L_pre)))/((double)N*N_g*4.0*L);

    th += dth;
    th = math.wrap(th,-const_math_pi,const_math_pi);
    x += dl*cos(th);
    y += dl*sin(th);

    N_L_pre = N_L;
    N_R_pre = N_R;
}

void Wheel_odom_d_drive::get_wheel_speed(double* wr, double* wl){
    *wr = w_R;
    *wl = w_L;
}

void Wheel_odom_d_drive::get_twist(double* vc, double* wc){
    *vc = v;
    *wc = w;
}

void Wheel_odom_d_drive::get_pose(double* xc, double* yc, double* thc){
    *xc = x;
    *yc = y;
    *thc = th;
}

void Wheel_odom_d_drive::set_PPR(long N_){
    N = N_;
}

void Wheel_odom_d_drive::set_gear_ratio(double N_g_){
    N_g = N_g_;
}

void Wheel_odom_d_drive::set_r(double r_){
    r = r_;
}

void Wheel_odom_d_drive::set_L(double L_){
    L = L_;
}

long Wheel_odom_d_drive::get_PPR(){
    return N;
}

double Wheel_odom_d_drive::get_gear_ratio(){
    return N_g;
}

double Wheel_odom_d_drive::get_r(){
    return r;
}

double Wheel_odom_d_drive::get_L(){
    return L;
}

double Wheel_odom_d_drive::get_dt(){
    return dt;
}