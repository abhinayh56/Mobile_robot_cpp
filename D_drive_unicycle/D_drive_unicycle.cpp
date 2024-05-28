#include "D_drive_unicycle.h"

D_drive_unicycle::D_drive_unicycle(){
}

void D_drive_unicycle::set_param(double r_, double L_){
    r = r_;
    L = L_;
}

void D_drive_unicycle::set_specs(double vc_max_, double wc_max_){
    vc_max = vc_max_;
    wc_max = wc_max_;
    w_lr_max = vc_max/r;
}

void D_drive_unicycle::set_r(double r_){
    r = r_;
}

void D_drive_unicycle::set_L(double L_){
    L = L_;
}

void D_drive_unicycle::set_vc_max(double vc_max_){
    vc_max = vc_max_;
    w_lr_max = vc_max/r;
    wc_max = 2.0*vc_max/L;
}

void D_drive_unicycle::set_wc_max(double wc_max_){
    wc_max = wc_max_;
    vc_max = wc_max*L/2.0;
    w_lr_max = vc_max/r;
}

double D_drive_unicycle::get_r(){
    return r;
}

double D_drive_unicycle::get_L(){
    return L;
}

double D_drive_unicycle::get_wlr_max(){
    return w_lr_max;
}

double D_drive_unicycle::get_vc_max(){
    return vc_max;
}

double D_drive_unicycle::get_wc_max(){
    return wc_max;
}

void D_drive_unicycle::uni2ddr(double vc, double wc, double* wr, double* wl){
    double vc_ = vc;
    double wc_ = wc;
    update_domain_vw(vc, wc, &vc_, &wc_);
    
    *wr = (vc_ + wc_*L*0.5) / r;
    *wl = (vc_ - wc_*L*0.5) / r;
}

double D_drive_unicycle::get_wr(double vc, double wc){
    return (vc + wc*L*0.5) / r;
}

double D_drive_unicycle::get_wl(double vc, double wc){
    return (vc - wc*L*0.5) / r;
}

void D_drive_unicycle::ddr2uni(double wr, double wl, double* vc, double* wc){
    *vc = (wr + wl)*r*0.5;
    *wc = (wr - wl)*r/L;
}

double D_drive_unicycle::get_vc(double wr, double wl){
    return (wr + wl)*r*0.5;
}

double D_drive_unicycle::get_wc(double wr, double wl){
    return (wr - wl)*r/L;
}

void D_drive_unicycle::update_domain_vw(double vc_in, double wc_in, double* vc_out, double* wc_out){
    double vc_n = vc_in;
    double wc_n = wc_in;

    int region = 0;

    if(wc_in>=wc_max){
        region = 5;
    }
    else if(wc_in <=(-wc_max)){
        region = 6;
    }
    else if(wc_in>=0){
        if(vc_in>=0){
            if(math_fun.points_A0_line_same_side(vc_in, wc_in, vc_max, 0.0, 0.0, wc_max)==false){
                region = 1;
            }
        }
        else{
            if(math_fun.points_A0_line_same_side(vc_in, wc_in, -vc_max, 0.0, 0.0, wc_max)==false){
                region = 2;
            }
        }
    }
    else{
        if(vc_in>=0){
            if(math_fun.points_A0_line_same_side(vc_in, wc_in, vc_max, 0.0, 0.0, -wc_max)==false){
                region = 4;
            }
        }
        else{
            if(math_fun.points_A0_line_same_side(vc_in, wc_in, -vc_max, 0.0, 0.0, -wc_max)==false){
                region = 3;
            }
        }
    }

    switch(region){
        case 0:
            vc_n = vc_in;
            wc_n = wc_in;
            break;
        case 1:
            vc_n = math_fun.linear_map(wc_in, 0.0, wc_max, vc_max, 0.0);
            wc_n = wc_in;
            break;
        case 2:
            vc_n = math_fun.linear_map(wc_in, 0.0, wc_max, -vc_max, 0.0);
            wc_n = wc_in;
            break;
        case 3:
            vc_n = math_fun.linear_map(wc_in, 0.0, -wc_max, -vc_max, 0.0);
            wc_n = wc_in;
            break;
        case 4:
            vc_n = math_fun.linear_map(wc_in, 0.0, -wc_max, vc_max, 0.0);
            wc_n = wc_in;
            break;
        case 5:
            vc_n = 0.0;
            wc_n = wc_in;
            break;
        case 6:
            vc_n = 0.0;
            wc_n = wc_in;
            break;
        default:
            vc_n = vc_in;
            wc_n = wc_in;
            break;
    }

    *vc_out = vc_n;
    *wc_out = wc_n;
}