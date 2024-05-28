#ifndef DIFF_DRIVE_UNICYCLE
#define DIFF_DRIVE_UNICYCLE

#include "Math_functions.h"

class Diff_drive_unicycle{
    public:
        Diff_drive_unicycle();
        void set_param(double r_, double L_);
        void set_specs(double vc_max_, double wc_max_);
        void set_r(double r_);
        void set_L(double L_);
        void set_vc_max(double vc_max_);
        void set_wc_max(double wc_max_);
        double get_r();
        double get_L();
        double get_wlr_max();
        double get_vc_max();
        double get_wc_max();
        void uni2ddr(double vc, double wc, double* wr, double* wl);
        double get_wr(double vc, double wc);
        double get_wl(double vc, double wc);
        void ddr2uni(double wr, double wl, double* vc, double* wc);
        double get_vc(double wr, double wl);
        double get_wc(double wr, double wl);
        void update_domain_vw(double vc_in, double wc_in, double* vc_out, double* wc_out);

    private:
        Math_functions math_fun;
        
        double r = 1.0;
        double L = 1.0;
        double w_lr_max = 0.0;
        double vc_max = 0.0;
        double wc_max = 0.0;
};

#endif