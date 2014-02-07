#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <Boards_ctrl_basic.h>
#include <Boards_exception.h>
#include <boost/tokenizer.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/format.hpp>

using namespace boost::numeric::ublas;
using namespace arma;

// **************************EXTRA INCLUDES************************************
//#include "calc_jacob.h"
#include "mySkew.h"
#include "Traj_gen.h"        //trajectory generation
#include "orientation_ctrl.h"       //Basic computation for Orientaiton control

// **************************EXTRA INCLUDES************************************

// *************************ALL GLOBAL VARIABLES*******************************
static const std::vector<float> homeVel(15,15);

static const std::vector<float> homePos = {
    // lower body #15
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
//  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15
    // upper body #10
    0, 60,  0, -45, 0, -60, 0,  -45, 0,  0,  0,  0, 0,  0 , 0,  0 };
// 16, 17, 18, 19, 20,  21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31

std::vector<int> two_arms = {16, 17, 18 ,19, 26, 27, 28, 32,
                             20, 21, 22, 23, 29, 30, 31, 33};

std::vector<int> two_arms_nohands = {16, 17, 18 ,19, 26, 27, 28,
                                     20, 21, 22, 23, 29, 30, 31};

////////////////////////
// LPF TEST
///////////////////////
double lamda = 80.0;
vec qf_l(8), qf_l_old(8);


//For kinematics
ManipulationVars::ManipulationVars() :
        jacob_right(6, 8), jacob_left(6, 8), jacob_R(6, 16), jacob_R_1(3,16), jacob_R_2(3,16), zeros6b8(6,8), jacob_R_resized(6, 14),
        fkin_or_left(3,3), fkin_or_right(3,3), fkin_po_left(3,1), fkin_po_right(3,1),
        I_16(16,16),I_14(14,14), I_8(8,8), I_6(6,6), obj_R_G(3,3), o_R_A(3,3), o_R_B(3,3),
        o_base(3,1), o_r_A(3,1), o_r_B(3,1), obj_r_G(3,1),
        A_R_hat_BA(3,3), A_R_hat_T(3,3), A_R_hat_G(3,3), J_11(3,8), J_12(3,8), J_21(3,8), J_22(3,8),
        J_p_L(3,8), J_p_R(3,8), J_o_L(3,8), J_o_R(3,8), joint_error_ar(16),
        Theta_0_R(8), Theta_0_L(8), q_l(16), q_l_resized(14), q_dot(16), tau(14), C_vel(6), q_ref(16), q_h(16), delta_q(16), delta_q_sum(16), null_vel(16), obj_r_T(3), obj_w_T(3),
        lambda_dot_jntlmt_r(8),lambda_dot_jntlmt_l(8), lambda_dot_jntlmt(16), q_max_r(8), q_min_r(8), q_bar_r(8), q_max_l(8), q_min_l(8), q_bar_l(8),
        Qr(4), Ql(4), Qi_r(4), Qi_l(4), Qd_r(4), Qd_l(4), Eo_r(3), Eo_l(3),
        Xi_R(6), Xi_L(6),orient_e_R(3),
        dXd_D(6), Xd_D(6),
        Xd_R(6), Xd_L(6),
        dXd_R(6), dXd_L(6), z6(6),
        Xd_D_init(6) {

        K_inv   = 0.001;     //Tuned (0.02* 0.05)
        K_clik  = 1.5;       //Tuned
        K_null  = 0.01;     //not tuned
        L0=0.15154; L1=0.1375; L2=0.19468; a3=0.005; Le=0.010;
        S1=0;S2=0;S3=0;S4=0;S5=0;S6=0;S7=0;S8=0;S9=0;S10=0;S11=0;S12=0;S13=0;S14=0;S15=0;S16=0;
        C1=0;C2=0;C3=0;C4=0;C5=0;C6=0;C7=0;C8=0;C9=0;C10=0;C11=0;C12=0;C13=0;C14=0;C15=0;C16=0;

        //flags
        flag_run_once = false;
        safety_flag=0, jntlmt_flag=0;
        count_loop_1=0, hand_flag_control_r=0, hand_flag_control_l=0;
        flag_init_hands = false;
        flag_init_pushing = false;
        flag_init_movingfar=false;
        init_rot_po = false;
    }

void ManipulationVars::manip_kine()
{
    int bId;

    I_16.eye();
    I_14.eye();
    I_8.eye();
    I_6.eye();
    obj_R_G.eye();
    o_R_A.eye();
    o_R_B.eye();
    o_base.zeros();
    o_r_A.zeros();
    o_r_B.zeros();
    obj_r_G.zeros();
    zeros6b8.zeros();
    orient_e_R.zeros();
    z6.zeros();

    q_max_r <<195*M_PI/180.0 <<endr
            <<170*M_PI/180.0 <<endr
            <<90*M_PI/180.0 <<endr
            <<0*M_PI/180.0  <<endr
            <<90*M_PI/180.0 <<endr
            <<30*M_PI/180.0 <<endr
            <<45*M_PI/180.0 <<endr
            <<90*M_PI/180.0 <<endr;

    q_min_r <<-95*M_PI/180.0 <<endr
            <<-18*M_PI/180.0 <<endr
            <<-90*M_PI/180.0 <<endr
            <<-135*M_PI/180.0<<endr
            <<-90*M_PI/180.0 <<endr
            <<-30*M_PI/180.0 <<endr
            <<-80*M_PI/180.0 <<endr
            <<-90*M_PI/180.0 <<endr;

    q_bar_r = 0.5*(q_max_r + q_min_r);
    q_bar_r(6) = 0.0*M_PI/180.0;    //wrist
    //q_bar_r(5) = -10.0*M_PI/180.0;    //wrist

    q_max_l <<95*M_PI/180.0 <<endr
            <<18*M_PI/180.0 <<endr
            <<90*M_PI/180.0 <<endr
            <<0*M_PI/180.0  <<endr
            <<90*M_PI/180.0 <<endr
            <<30*M_PI/180.0 <<endr
            <<80*M_PI/180.0 <<endr
            <<90*M_PI/180.0 <<endr;

    q_min_l <<-195*M_PI/180.0 <<endr
            <<-170*M_PI/180.0 <<endr
            <<-90*M_PI/180.0 <<endr
            <<-135*M_PI/180.0<<endr
            <<-90*M_PI/180.0 <<endr
            <<-30*M_PI/180.0 <<endr
            <<-45*M_PI/180.0 <<endr
            <<-90*M_PI/180.0 <<endr;

    q_bar_l = 0.5*(q_max_l + q_min_l);
    q_bar_l(6) = 0.0*M_PI/180.0;
    //q_bar_l(5) = -10.0*M_PI/180.0;

    S1=sin(q_l(0) - M_PI/2.0);
    S2=sin(q_l(1));
    S3=sin(q_l(2) + M_PI/2.0);
    S4=sin(q_l(3));
    S5=sin(q_l(4));
    S6=sin(q_l(5) - M_PI/2.0);
    S7=sin(q_l(6));
    S8=0.0;
    S9=sin(q_l(8) - M_PI/2.0);
    S10=sin(q_l(9));
    S11=sin(q_l(10)- M_PI/2.0);
    S12=sin(q_l(11));
    S13=sin(q_l(12));
    S14=sin(q_l(13)- M_PI/2.0);
    S15=sin(q_l(14));
    S16=0.0;

    C1=cos(q_l(0) - M_PI/2.0);
    C2=cos(q_l(1));
    C3=cos(q_l(2) + M_PI/2.0);
    C4=cos(q_l(3));
    C5=cos(q_l(4));
    C6=cos(q_l(5) - M_PI/2.0);
    C7=cos(q_l(6));
    C8=1.0;
    C9=cos(q_l(8) - M_PI/2.0);
    C10=cos(q_l(9));
    C11=cos(q_l(10)- M_PI/2.0);
    C12=cos(q_l(11));
    C13=cos(q_l(12));
    C14=cos(q_l(13)- M_PI/2.0);
    C15=cos(q_l(14));
    C16=1.0;


    lambda_dot_jntlmt_r  <<(q_l(0)-q_bar_r(0))/(q_max_r(0)-q_min_r(0))<<endr
                         <<(q_l(1)-q_bar_r(1))/(q_max_r(1)-q_min_r(1))<<endr
                         <<(q_l(2)-q_bar_r(2))/(q_max_r(2)-q_min_r(2))<<endr
                         <<(q_l(3)-q_bar_r(3))/(q_max_r(3)-q_min_r(3))<<endr
                         <<(q_l(4)-q_bar_r(4))/(q_max_r(4)-q_min_r(4))<<endr
                         <<(q_l(5)-q_bar_r(5))/(q_max_r(5)-q_min_r(5))<<endr
                         <<(q_l(6)-q_bar_r(6))/(q_max_r(6)-q_min_r(6))<<endr
                         <<(q_l(7)-q_bar_r(7))/(q_max_r(7)-q_min_r(7))<<endr;

   lambda_dot_jntlmt_l   <<(q_l(8)-q_bar_l(0))/(q_max_l(0)-q_min_l(0))<<endr
                         <<(q_l(9)-q_bar_l(1))/(q_max_l(1)-q_min_l(1))<<endr
                         <<(q_l(10)-q_bar_l(2))/(q_max_l(2)-q_min_l(2))<<endr
                         <<(q_l(11)-q_bar_l(3))/(q_max_l(3)-q_min_l(3))<<endr
                         <<(q_l(12)-q_bar_l(4))/(q_max_l(4)-q_min_l(4))<<endr
                         <<(q_l(13)-q_bar_l(5))/(q_max_l(5)-q_min_l(5))<<endr
                         <<(q_l(14)-q_bar_l(6))/(q_max_l(6)-q_min_l(6))<<endr
                         <<(q_l(15)-q_bar_l(7))/(q_max_l(7)-q_min_l(7))<<endr;

   lambda_dot_jntlmt_r(7)=0;
   lambda_dot_jntlmt_l(7)=0;

   lambda_dot_jntlmt = join_cols(lambda_dot_jntlmt_r,lambda_dot_jntlmt_l);



    //SCin<<S1<<S2<<S3<<S4<<S5<<S6<<S7<<S8<<S9<<S10<<S11<<S12<<S13<<S14<<S15<<S16<<C1<<C2<<C3<<C4<<C5<<C6<<C7<<C8<<C9<<C10<<C11<<C12<<C13<<C14<<C15<<C16;


   //-------RELATIVE KINEMATICS

    fkin_or_left<<S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11)) - C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11)))<< S15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) + C15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11))<< - C14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - S14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))<<endr
     <<S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12))<< C15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) - S15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12))<< C14*(C10*C12 + C11*S10*S12) - S14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13)<<endr
     <<S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11)) - C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11)))<< S15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) + C15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11))<< - C14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - S14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))<<endr;

    fkin_or_right<<C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3))<< - S7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - C7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3))<< C6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) + S6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))<<endr
     <<- S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) - C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4))<< S7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4)) - C7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3)<< S6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) - C6*(C2*C4 - C3*S2*S4)<<endr
     <<C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3))<< - S7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - C7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3))<< C6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) + S6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))<<endr;

    fkin_po_left<<a3*(S9*S11 + C9*C10*C11) - L2*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) + Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11))) + L1*C9*S10<<endr
    << L0 + L2*(C10*C12 + C11*S10*S12) + L1*C10 - Le*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12))) - a3*C11*S10<<endr
    << a3*(C9*S11 - C10*C11*S9) - L2*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) + Le*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11))) - L1*S9*S10<<endr;

    fkin_po_right<<L2*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - a3*(S1*S3 - C1*C2*C3) - Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3))) - L1*C1*S2<<endr
    << Le*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4))) - L2*(C2*C4 - C3*S2*S4) - L1*C2 - L0 - a3*C3*S2<<endr
    << L2*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - a3*(C1*S3 + C2*C3*S1) - Le*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3))) + L1*S1*S2<<endr;

    jacob_right<<L2*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - a3*(C1*S3 + C2*C3*S1) - Le*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3))) + L1*S1*S2<< -C1*(L2*(C2*C4 - C3*S2*S4) + L1*C2 - Le*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4))) + a3*C3*S2)<< - C2*(a3*(C1*S3 + C2*C3*S1) - L2*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) + Le*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3)))) - S1*S2*(L2*(C2*C4 - C3*S2*S4) - Le*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4))) + a3*C3*S2)<< S2*S3*(L2*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - Le*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3)))) - (L2*(C2*C4 - C3*S2*S4) - Le*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4))))*(C1*C3 - C2*S1*S3)<< Le*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2)*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4))) - Le*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3)))*(C2*C4 - C3*S2*S4)<< - Le*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4)))*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3)) - Le*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3)*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3)))<< - Le*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3)))*(S6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) - C6*(C2*C4 - C3*S2*S4)) - Le*(C6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) + S6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3)))*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4)))<< 0<<endr
    <<0<< C1*(a3*(S1*S3 - C1*C2*C3) - L2*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) + Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3))) + L1*C1*S2) - S1*(a3*(C1*S3 + C2*C3*S1) - L2*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) + Le*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3))) - L1*S1*S2)<< S1*S2*(a3*(S1*S3 - C1*C2*C3) - L2*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) + Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3)))) + C1*S2*(a3*(C1*S3 + C2*C3*S1) - L2*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) + Le*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3))))<< (C3*S1 + C1*C2*S3)*(L2*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - Le*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3)))) - (C1*C3 - C2*S1*S3)*(L2*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3))))<< Le*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2)*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3))) - Le*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3)))*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2)<< Le*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3))*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3))) - Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3)))*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3))<< Le*(C6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) + S6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3)))*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3))) - Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3)))*(C6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) + S6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3)))<< 0<<endr
    <<a3*(S1*S3 - C1*C2*C3) - L2*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) + Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3))) + L1*C1*S2<< S1*(L2*(C2*C4 - C3*S2*S4) + L1*C2 - Le*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4))) + a3*C3*S2)<< C2*(a3*(S1*S3 - C1*C2*C3) - L2*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) + Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3)))) - C1*S2*(L2*(C2*C4 - C3*S2*S4) - Le*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4))) + a3*C3*S2)<< (L2*(C2*C4 - C3*S2*S4) - Le*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4))))*(C3*S1 + C1*C2*S3) - S2*S3*(L2*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3))))<< Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3)))*(C2*C4 - C3*S2*S4) - Le*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4)))*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2)<< Le*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4)))*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3)) + Le*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3)*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3)))<< Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3)))*(S6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) - C6*(C2*C4 - C3*S2*S4)) + Le*(C6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) + S6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3)))*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4)))<< 0<<endr
    <<0<< -S1<< C1*S2<< - C3*S1 - C1*C2*S3<< C1*C4*S2 - S4*(S1*S3 - C1*C2*C3)<< S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3)<< C6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) + S6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))<< C6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) + S6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))<<endr
    <<1<< 0<< C2<< S2*S3<< C2*C4 - C3*S2*S4<< S5*(C2*S4 + C3*C4*S2) + C5*S2*S3<< S6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) - C6*(C2*C4 - C3*S2*S4)<< S6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) - C6*(C2*C4 - C3*S2*S4)<<endr
    <<0<< -C1<< -S1*S2<< C2*S1*S3 - C1*C3<< - S4*(C1*S3 + C2*C3*S1) - C4*S1*S2<< S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3)<< C6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) + S6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))<< C6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) + S6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))<<endr;

    jacob_left<<a3*(C9*S11 - C10*C11*S9) - L2*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) + Le*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11))) - L1*S9*S10<< C9*(L2*(C10*C12 + C11*S10*S12) + L1*C10 - Le*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12))) - a3*C11*S10)<< S9*S10*(Le*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12))) - L2*(C10*C12 + C11*S10*S12) + a3*C11*S10) - C10*(a3*(C9*S11 - C10*C11*S9) - L2*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) + Le*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11))))<< - (L2*(C10*C12 + C11*S10*S12) - Le*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12))))*(C9*C11 + C10*S9*S11) - S10*S11*(L2*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - Le*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11))))<< Le*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10)*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12))) - Le*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11)))*(C10*C12 + C11*S10*S12)<< - Le*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12)))*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11)) - Le*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11)*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11)))<< - Le*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11)))*(S14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) - C14*(C10*C12 + C11*S10*S12)) - Le*(C14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) + S14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11)))*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12)))<< 0<<endr
    <<0<< S9*(a3*(C9*S11 - C10*C11*S9) - L2*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) + Le*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11))) - L1*S9*S10) - C9*(a3*(S9*S11 + C9*C10*C11) - L2*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) + Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11))) + L1*C9*S10)<< S9*S10*(a3*(S9*S11 + C9*C10*C11) - L2*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) + Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11)))) + C9*S10*(a3*(C9*S11 - C10*C11*S9) - L2*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) + Le*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11))))<< (C11*S9 - C9*C10*S11)*(L2*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - Le*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11)))) - (C9*C11 + C10*S9*S11)*(L2*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11))))<< Le*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10)*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11))) - Le*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11)))*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10)<< Le*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11))*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11))) - Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11)))*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11))<< Le*(C14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) + S14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11)))*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11))) - Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11)))*(C14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) + S14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11)))<< 0<<endr
    <<L2*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - a3*(S9*S11 + C9*C10*C11) - Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11))) - L1*C9*S10<< -S9*(L2*(C10*C12 + C11*S10*S12) + L1*C10 - Le*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12))) - a3*C11*S10)<< C10*(a3*(S9*S11 + C9*C10*C11) - L2*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) + Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11)))) + C9*S10*(Le*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12))) - L2*(C10*C12 + C11*S10*S12) + a3*C11*S10)<< (L2*(C10*C12 + C11*S10*S12) - Le*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12))))*(C11*S9 - C9*C10*S11) + S10*S11*(L2*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11))))<< Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11)))*(C10*C12 + C11*S10*S12) - Le*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12)))*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10)<< Le*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12)))*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11)) + Le*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11)*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11)))<< Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11)))*(S14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) - C14*(C10*C12 + C11*S10*S12)) + Le*(C14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) + S14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11)))*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12)))<< 0<<endr
    <<0<< -S9<< -C9*S10<< C11*S9 - C9*C10*S11<< S12*(S9*S11 + C9*C10*C11) - C9*C12*S10<< C13*(C11*S9 - C9*C10*S11) - S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12)<< - C14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - S14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))<< - C14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - S14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))<<endr
    <<1<< 0<< -C10<< S10*S11<< - C10*C12 - C11*S10*S12<< C13*S10*S11 - S13*(C10*S12 - C11*C12*S10)<< C14*(C10*C12 + C11*S10*S12) - S14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13)<< C14*(C10*C12 + C11*S10*S12) - S14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13)<<endr
    <<0<< -C9<< S9*S10<< C9*C11 + C10*S9*S11<< S12*(C9*S11 - C10*C11*S9) + C12*S9*S10<< C13*(C9*C11 + C10*S9*S11) - S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12)<< - C14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - S14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))<< - C14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - S14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))<<endr;

    J_p_L=jacob_left.rows(0,2);
    J_o_L=jacob_left.rows(3,5);

    J_p_R=jacob_right.rows(0,2);
    J_o_R=jacob_right.rows(3,5);


    //-------RELATIVE KINEMATICS

    //calcSkew(fkin_or_right.t() * o_R_A.t() * (o_r_B - o_r_A), A_R_hat_BA);
    //A_R_hat_BA=-1*A_R_hat_BA;

    A_R_hat_BA.zeros();
    calcSkew(fkin_or_right.t() * o_R_A.t() * o_R_B * fkin_po_left, A_R_hat_T);
    A_R_hat_T=-1*A_R_hat_T;
    calcSkew(fkin_or_right.t() * fkin_po_right, A_R_hat_G);
    A_R_hat_G=-1*A_R_hat_G;

    J_11=-obj_R_G * (fkin_or_right.t() * J_p_R + A_R_hat_BA * fkin_or_right.t() * J_o_R + A_R_hat_T * fkin_or_right.t() * J_o_R - A_R_hat_G * fkin_or_right.t() * J_o_R);
    J_12= obj_R_G * fkin_or_right.t() *  o_R_A.t() * o_R_B * J_p_L;
    J_21=-obj_R_G * fkin_or_right.t() * J_o_R;
    J_22= obj_R_G * fkin_or_right.t() * o_R_A.t() * o_R_B * J_o_L;

    jacob_R_1=join_rows(J_11,J_12);
    jacob_R_2=join_rows(J_21,J_22);

    jacob_R=join_cols(jacob_R_1,jacob_R_2);

    jacob_R_resized.cols(0,6)=jacob_R.cols(0,6);
    jacob_R_resized.cols(7,13)=jacob_R.cols(8,14);

    obj_r_T=obj_R_G * fkin_or_right.t() * o_R_A.t() * (o_r_B - o_r_A  + o_R_B * fkin_po_left) - obj_R_G * fkin_or_right.t() * fkin_po_right + obj_r_G;
    //obj_w_T=obj_R_G * fkin_or_right.t() *(o_R_A.t() * o_R_B * J_o_L *delta_q.rows(8,15)  -  J_o_R * delta_q.rows(0,7));
    obj_w_T=obj_R_G * fkin_or_right.t() *(o_R_A.t() * o_R_B * J_o_L *q_dot.rows(8,15)  -  J_o_R * q_dot.rows(0,7));

    pinv_jacob_right=pinv(jacob_right);
    pinv_jacob_left=pinv(jacob_left);
    pinv_jacob_R=pinv(jacob_R);

    orient_e_R=orient_e_R+obj_w_T;

    /*A_R_hat_G
    if (det(jacob_right*jacob_right.t())>0.001)
        pinv_jacob_right=pinv(jacob_right);
    else
        pinv_jacob_right=jacob_right.t() * inv(jacob_right*jacob_right.t() + 0.001*I_6);

    if (det(jacob_left*jacob_left.t())>0.001)
        pinv_jacob_left=pinv(jacob_left);
    else
        pinv_jacob_left=jacob_left.t() * inv(jacob_left*jacob_left.t() + 0.001*I_6);

    if (det(jacob_R*jacob_R.t())>0.001)
        pinv_jacob_R=pinv(jacob_R);
    else
        pinv_jacob_R=jacob_R.t() * inv(jacob_R*jacob_R.t() + 0.001*I_6);
    */

    //   START: PRIRORITY KINEMATICS
    Js_1 = join_rows(jacob_right,zeros6b8);
    Js_2 = jacob_R;

    Js_1.col(7)=z6;//added
    Js_2.col(7)=z6;
    Js_2.col(15)=z6;


     if (det(Js_1*Js_1.t())>0.0000001)
         pinv_Js_1=pinv(Js_1);
     else
        pinv_Js_1=Js_1.t() * inv(Js_1*Js_1.t() + 0.001*I_6);


     Jhat_2=Js_2*(I_16- pinv_Js_1*Js_1);

     if (det(Jhat_2*Jhat_2.t())>0.0000001)
        pinv_Jhat_2=pinv(Jhat_2);
     else
        pinv_Jhat_2=Jhat_2.t() * inv(Jhat_2*Jhat_2.t() + 0.001*I_6);
     //   END: PRIRORITY KINEMATICS


     //postion two hands
     X_R<<fkin_po_right(0)<<fkin_po_right(1)<<fkin_po_right(2)<<0.0<<0.0<<0.0;
     X_L<<fkin_po_left(0)<<fkin_po_left(1)<<fkin_po_left(2)<<0.0<<0.0<<0.0;

     //orientation for two hands (Quaternion)
     //vec Qr(4), Ql(4): declared as global
     RotQuaternion(fkin_or_right, Qr);
     RotQuaternion(fkin_or_left, Ql);

     //relative pos/ori
     X_D<<obj_r_T(0)<<obj_r_T(1)<<obj_r_T(2)<<orient_e_R(0)<<orient_e_R(1)<<orient_e_R(2);
}

void ManipulationVars::reaching(u_int64_t dt_ns){


    vec Xf_R(6), Xf_L(6);
    Xf_R.zeros(); Xf_L.zeros();

    if(flag_init_hands == false){
        //t_traj_start = get_time_ns();
        Xi_R<<fkin_po_right(0)<<fkin_po_right(1)<<fkin_po_right(2)<<0.0<<0.0<<0.0;
        Xi_L<<fkin_po_left(0)<<fkin_po_left(1)<<fkin_po_left(2)<<0.0<<0.0<<0.0;                
        flag_init_hands = true;
        Xd_R = Xi_R;
        Xd_L = Xi_L;

        RotQuaternion(fkin_or_right, Qi_r);
        RotQuaternion(fkin_or_left, Qi_l);
        Qd_r = Qi_r;
        Qd_l = Qi_l;
    }

        //Get valve position/orientation
        //vec Xv(6);
        //vec Xv = get_valve_data();
        vec Xv(6);
        Xv<<0.25<<0.0<<0.0<<M_PI<<0.0<<0.0;
        mat ROTv(3,3), deltaR_R(3,3), deltaR_L(3,3), L_rot_R, L_rot_L;
        vec Qv(4);
        //Rotation matrix from ZYX Euler angle (alpha-beta-gamma)
        ROTv<< cos(Xv(3))*cos(Xv(4)) << cos(Xv(3))*sin(Xv(4))*sin(Xv(5)) - sin(Xv(3))*cos(Xv(5)) << cos(Xv(3))*sin(Xv(4))*cos(Xv(5)) + sin(Xv(3))*sin(Xv(5)) <<endr
            << sin(Xv(3))*cos(Xv(4)) << sin(Xv(3))*sin(Xv(4))*sin(Xv(5)) + cos(Xv(3))*cos(Xv(5)) << sin(Xv(3))*sin(Xv(4))*cos(Xv(5)) - cos(Xv(3))*sin(Xv(5)) <<endr
            << -sin(Xv(4)) << cos(Xv(4))*sin(Xv(5)) << cos(Xv(4))*cos(Xv(5)) <<endr;
        //Quaternion for valve orientation
        RotQuaternion(ROTv, Qv);

        deltaR_R=ROTv* fkin_or_right.t();
        deltaR_L=ROTv* fkin_or_left.t();

        //Get valve radius
        //double Rv = get_radius();
        double Rv=0.1;

        //Calculate target position for two hands
        vec Xt_R(6), Xt_L(6);
        double Roff = 0.05;     //offset, 5cm

        Xt_R=Xv;    Xt_R(1)=Xv(1) - Rv - Roff;
        Xt_L=Xv;    Xt_L(1)=Xv(1) + Rv + Roff;


        //displacement: target - init
        //TEST:
        //Xf_R<<0.10<<-0.10<<0.10<<0.0<<0.0<<0.0;
        //Xf_L<<0.10<< 0.10<<0.10<<0.0<<0.0<<0.0;
        Xf_R = Xt_R - Xi_R;
        Xf_L = Xt_L - Xi_L;

       //Arms Trajectory
        line_traj( Xi_R, Xf_R , 10.0, (dt_ns/1e9), Xd_R, dXd_R);
        line_traj( Xi_L, Xf_L , 10.0, (dt_ns/1e9), Xd_L, dXd_L);


        //Quaternion Trajectory
        vec Xfq_R(4), Xfq_L(4);
        vec dQd_r(4), dQd_l(4); //dummy

        Xfq_R = Qv - Qi_r;
        Xfq_L = Qv - Qi_l;

        line_traj(Qi_r, Xfq_R, 2.0, (dt_ns/1e9), Qd_r, dQd_r);
        line_traj(Qi_l, Xfq_L, 2.0, (dt_ns/1e9), Qd_l, dQd_l);

        //Orientation Error
        Eo_r.zeros();
        Eo_l.zeros();
        //OrientationError(Qd_r, Qr, Eo_r);
        //OrientationError(Qd_l, Ql, Eo_l);

       // RotQuaternion(deltaR_R, Eo_r);
       // RotQuaternion(deltaR_L, Eo_l);

        mat Sn_e, Ss_e, Sa_e, Sn_d, Ss_d, Sa_d;
        vec n_e, s_e, a_e, n_d, s_d, a_d;

        n_d=ROTv.col(0);
        s_d=ROTv.col(1);
        a_d=ROTv.col(2);

        calcSkew(n_d, Sn_d);
        calcSkew(s_d, Ss_d);
        calcSkew(a_d, Sa_d);


        n_e=fkin_or_right.col(0);
        s_e=fkin_or_right.col(1);
        a_e=fkin_or_right.col(2);

        calcSkew(n_e, Sn_e);
        calcSkew(s_e, Ss_e);
        calcSkew(a_e, Sa_e);
        L_rot_R=-0.5*(Sn_d*Sn_e + Ss_d*Ss_e + Sa_d*Sa_e);
        Eo_r=0.5*(cross(n_e,n_d) + cross(s_e,s_d) + cross(a_e,a_d));


        n_e=fkin_or_left.col(0);
        s_e=fkin_or_left.col(1);
        a_e=fkin_or_left.col(2);

        calcSkew(n_e, Sn_e);
        calcSkew(s_e, Ss_e);
        calcSkew(a_e, Sa_e);
        L_rot_L=-0.5*(Sn_d*Sn_e + Ss_d*Ss_e + Sa_d*Sa_e);
        Eo_l=0.5*(cross(n_e,n_d) + cross(s_e,s_d) + cross(a_e,a_d));

        //IK solution: CLICK
        vec V_R(6), V_L(6);

        V_R.subvec(0,2) = dXd_R.subvec(0,2) + K_clik*(Xd_R.subvec(0,2) - X_R.subvec(0,2));
        V_R.subvec(3,5) =  1 * Eo_r;

        V_L.subvec(0,2) = dXd_L.subvec(0,2) + K_clik*(Xd_L.subvec(0,2) - X_L.subvec(0,2));
        //V_L.subvec(3,5) = 0.5*K_clik*Eo_l;
        V_L.subvec(3,5) =  1 * Eo_l;


        delta_q.rows(0,7)=  K_inv* pinv_jacob_right*V_R + K_null * (I_8 - pinv_jacob_right*jacob_right) * (-lambda_dot_jntlmt_r);
        delta_q.rows(8,15)= K_inv* pinv_jacob_left *V_L + K_null * (I_8 - pinv_jacob_left*jacob_left) * (-lambda_dot_jntlmt_l);


        //if(norm((Xd_R - X_R),"fro") > 0.01)//Right arm (TODO)
        //delta_q.rows(0,7)= K_inv* pinv_jacob_right* (dXd_R + K_clik*(Xd_R - X_R) ) + K_null * (I_8 - pinv_jacob_right*jacob_right) * (-lambda_dot_jntlmt_r);

         //if(norm((Xd_L - X_L),"fro") > 0.01)//Left arm (TODO)
        //delta_q.rows(8,15)= K_inv* pinv_jacob_left* (dXd_L + K_clik*(Xd_L - X_L) ) + K_null * (I_8 - pinv_jacob_left*jacob_left) * (-lambda_dot_jntlmt_l);


}

void ManipulationVars::pushing(u_int64_t dt_ns){

    double Roff = 0.05;     //offset, 5cm
    mat ROTv(3,3), L_rot_R, L_rot_L;

    if(flag_init_pushing == false){
        //t_traj_start = get_time_ns();
        Xd_R<<fkin_po_right(0)<<fkin_po_right(1)+ Roff<<fkin_po_right(2)<<0.0<<0.0<<0.0;
        Xd_L<<fkin_po_left(0)<<fkin_po_left(1) - Roff<<fkin_po_left(2)<<0.0<<0.0<<0.0;
        flag_init_pushing = true;

    }

    vec Xv(6);
    Xv<<0.0<<0.0<<0.0<<M_PI<<0.0<<0.0;

    //Rotation matrix from ZYX Euler angle (alpha-beta-gamma)
    ROTv<< cos(Xv(3))*cos(Xv(4)) << cos(Xv(3))*sin(Xv(4))*sin(Xv(5)) - sin(Xv(3))*cos(Xv(5)) << cos(Xv(3))*sin(Xv(4))*cos(Xv(5)) + sin(Xv(3))*sin(Xv(5)) <<endr
        << sin(Xv(3))*cos(Xv(4)) << sin(Xv(3))*sin(Xv(4))*sin(Xv(5)) + cos(Xv(3))*cos(Xv(5)) << sin(Xv(3))*sin(Xv(4))*cos(Xv(5)) - cos(Xv(3))*sin(Xv(5)) <<endr
        << -sin(Xv(4)) << cos(Xv(4))*sin(Xv(5)) << cos(Xv(4))*cos(Xv(5)) <<endr;

    //Orientation Error
    Eo_r.zeros();
    Eo_l.zeros();


    mat Sn_e, Ss_e, Sa_e, Sn_d, Ss_d, Sa_d;
    vec n_e, s_e, a_e, n_d, s_d, a_d;

    n_d=ROTv.col(0);
    s_d=ROTv.col(1);
    a_d=ROTv.col(2);

    calcSkew(n_d, Sn_d);
    calcSkew(s_d, Ss_d);
    calcSkew(a_d, Sa_d);


    n_e=fkin_or_right.col(0);
    s_e=fkin_or_right.col(1);
    a_e=fkin_or_right.col(2);

    calcSkew(n_e, Sn_e);
    calcSkew(s_e, Ss_e);
    calcSkew(a_e, Sa_e);
    L_rot_R=-0.5*(Sn_d*Sn_e + Ss_d*Ss_e + Sa_d*Sa_e);
    Eo_r=0.5*(cross(n_e,n_d) + cross(s_e,s_d) + cross(a_e,a_d));


    n_e=fkin_or_left.col(0);
    s_e=fkin_or_left.col(1);
    a_e=fkin_or_left.col(2);

    calcSkew(n_e, Sn_e);
    calcSkew(s_e, Ss_e);
    calcSkew(a_e, Sa_e);
    L_rot_L=-0.5*(Sn_d*Sn_e + Ss_d*Ss_e + Sa_d*Sa_e);
    Eo_l=0.5*(cross(n_e,n_d) + cross(s_e,s_d) + cross(a_e,a_d));


    //IK solution: CLICK
    vec V_R(6), V_L(6);

    V_R.subvec(0,2) =  K_clik*(Xd_R.subvec(0,2) - X_R.subvec(0,2));
    V_R.subvec(3,5) =  5 * Eo_r;

    V_L.subvec(0,2) =  K_clik*(Xd_L.subvec(0,2) - X_L.subvec(0,2));
    V_L.subvec(3,5) =  5 * Eo_l;


        delta_q.rows(0,7)=  K_inv* pinv_jacob_right*V_R + K_null * (I_8 - pinv_jacob_right*jacob_right) * (-lambda_dot_jntlmt_r);
        delta_q.rows(8,15)= K_inv* pinv_jacob_left *V_L + K_null * (I_8 - pinv_jacob_left*jacob_left) * (-lambda_dot_jntlmt_l);

}


void ManipulationVars::movingfar(u_int64_t dt_ns){

    if(flag_init_movingfar == false){
        //t_traj_start = get_time_ns();
        Xd_R<<fkin_po_right(0)<<fkin_po_right(1)-0.15<<fkin_po_right(2)+0.15<<0.0<<0.0<<0.0;
        Xd_L<<fkin_po_left(0)<<fkin_po_left(1)+0.15<<fkin_po_left(2)-0.15<<0.0<<0.0<<0.0;
        flag_init_movingfar = true;

    }

    delta_q.rows(0,7)=  K_inv* pinv_jacob_right*(Xd_R - X_R) + K_null * (I_8 - pinv_jacob_right*jacob_right) * (-lambda_dot_jntlmt_r);
    delta_q.rows(8,15)= K_inv* pinv_jacob_left *(Xd_L - X_L) + K_null * (I_8 - pinv_jacob_left*jacob_left) * (-lambda_dot_jntlmt_l);

}


/*void ManipulationVars::rotating(u_int64_t dt_ns){

    mat Sn_e, Ss_e, Sa_e, Sn_d, Ss_d, Sa_d;
    vec n_e, s_e, a_e, n_d, s_d, a_d;
    vec Xv(6);
    double angle_d_R, angle_d_L;
    Eo_r.zeros();//Orientation Error
    mat ROTv_i(3,3), ROTv(3,3), ROTthe_R(3,3), ROTthe_L(3,3), L_rot_R, L_rot_L;

    if(init_rot_po == false){
        Xi_R << fkin_po_right(0) << fkin_po_right(1) << fkin_po_right(2)<<0.0<<0.0<<0.0;
        Xi_L<<fkin_po_left(0)<<fkin_po_left(1)<<fkin_po_left(2)<<0.0<<0.0<<0.0;
        init_rot_po = true;

        Xd_R = Xi_R;
        dXd_R.zeros();
        Xd_L = Xi_L;
        dXd_L.zeros();

    }

    //right hand trajectory
    angle_d_R = circle_traj( Xi_R, -45.0*M_PI/180.0 , 10.0, (dt_ns/1e9), 0.1 , Xd_R, dXd_R);
    angle_d_L = circle_traj( Xi_L, 45.0*M_PI/180.0 , 10.0, (dt_ns/1e9), 0.1 , Xd_L, dXd_L);


    Xv<<0<<0<<0<<M_PI<<0<<0;
    //Rotation matrix from ZYX Euler angle (alpha-beta-gamma)
    ROTv_i<< cos(Xv(3))*cos(Xv(4)) << cos(Xv(3))*sin(Xv(4))*sin(Xv(5)) - sin(Xv(3))*cos(Xv(5)) << cos(Xv(3))*sin(Xv(4))*cos(Xv(5)) + sin(Xv(3))*sin(Xv(5)) <<endr
        << sin(Xv(3))*cos(Xv(4)) << sin(Xv(3))*sin(Xv(4))*sin(Xv(5)) + cos(Xv(3))*cos(Xv(5)) << sin(Xv(3))*sin(Xv(4))*cos(Xv(5)) - cos(Xv(3))*sin(Xv(5)) <<endr
        << -sin(Xv(4)) << cos(Xv(4))*sin(Xv(5)) << cos(Xv(4))*cos(Xv(5)) <<endr;


    //RIGHT ARM
    ROTthe_R <<1<<0<<0<<endr
             <<0<<cos(angle_d_R)<<-sin(angle_d_R)<<endr
             <<0<<sin(angle_d_R)<<cos(angle_d_R)<<endr;

    ROTv=ROTv_i * ROTthe_R;

    n_d=ROTv.col(0);
    s_d=ROTv.col(1);
    a_d=ROTv.col(2);

    calcSkew(n_d, Sn_d);
    calcSkew(s_d, Ss_d);
    calcSkew(a_d, Sa_d);

    n_e=fkin_or_right.col(0);
    s_e=fkin_or_right.col(1);
    a_e=fkin_or_right.col(2);

    calcSkew(n_e, Sn_e);
    calcSkew(s_e, Ss_e);
    calcSkew(a_e, Sa_e);

    L_rot_R=-0.5*(Sn_d*Sn_e + Ss_d*Ss_e + Sa_d*Sa_e);
    Eo_r=0.5*(cross(n_e,n_d) + cross(s_e,s_d) + cross(a_e,a_d));


    //LEFT ARM
    ROTthe_L <<1<<0<<0<<endr
           <<0<<cos(angle_d_L)<<-sin(angle_d_L)<<endr
           <<0<<sin(angle_d_L)<<cos(angle_d_L)<<endr;

    ROTv=ROTv_i * ROTthe_L;

    n_d=ROTv.col(0);
    s_d=ROTv.col(1);
    a_d=ROTv.col(2);

    calcSkew(n_d, Sn_d);
    calcSkew(s_d, Ss_d);
    calcSkew(a_d, Sa_d);

    n_e=fkin_or_left.col(0);
    s_e=fkin_or_left.col(1);
    a_e=fkin_or_left.col(2);

    calcSkew(n_e, Sn_e);
    calcSkew(s_e, Ss_e);
    calcSkew(a_e, Sa_e);

    L_rot_L=-0.5*(Sn_d*Sn_e + Ss_d*Ss_e + Sa_d*Sa_e);
    Eo_l=0.5*(cross(n_e,n_d) + cross(s_e,s_d) + cross(a_e,a_d));

    //IK solution: CLICK
    vec V_R(6), V_L(6);

    V_R.subvec(0,2) =  K_clik*(Xd_R.subvec(0,2) - X_R.subvec(0,2));
    V_R.subvec(3,5) =  1 * Eo_r;

    V_L.subvec(0,2) =  K_clik*(Xd_L.subvec(0,2) - X_L.subvec(0,2));
    V_L.subvec(3,5) =  1 * Eo_l;


    delta_q.rows(0,7)=  K_inv* pinv_jacob_right*V_R + K_null * (I_8 - pinv_jacob_right*jacob_right) * (-lambda_dot_jntlmt_r);
    delta_q.rows(8,15)= K_inv* pinv_jacob_left *V_L + K_null * (I_8 - pinv_jacob_left*jacob_left) * (-lambda_dot_jntlmt_l);

}*/

void ManipulationVars::rotating(u_int64_t dt_ns){

    mat Sn_e, Ss_e, Sa_e, Sn_d, Ss_d, Sa_d;
    vec n_e, s_e, a_e, n_d, s_d, a_d;
    vec Xv(6);
    double angle_d;
    Eo_r.zeros();//Orientation Error
    mat ROTv_i(3,3), ROTv(3,3), ROTthe(3,3), L_rot_R;

    if(init_rot_po == false){
        Xi_R << fkin_po_right(0) << fkin_po_right(1) << fkin_po_right(2)<<0.0<<0.0<<0.0;
        Xi_L<<fkin_po_left(0)<<fkin_po_left(1)<<fkin_po_left(2)<<0.0<<0.0<<0.0;
        init_rot_po = true;
        Xd_R = Xi_R;
        dXd_R.zeros();

        Xd_D_init<<obj_r_T(0)<<obj_r_T(1)<<obj_r_T(2)<<0.0<<0.0<<0.0;

       //cout<<dt_ns/1e9<<endl;

    }

    //right hand trajectory
    //angle_d = circle_traj( Xi_R, -45.0*M_PI/180.0 , 10.0, (dt_ns/1e9), get_radius(), Xd_R, dXd_R);
    angle_d = circle_traj( Xi_R, -45.0*M_PI/180.0 , 10.0, (dt_ns/1e9), 0.1 , Xd_R, dXd_R);
    //relative pos/ori trajectory
    Xd_D = Xd_D_init; dXd_D.zeros();


    Xv<<0<<0<<0<<M_PI<<0<<0;
    //Rotation matrix from ZYX Euler angle (alpha-beta-gamma)
    ROTv_i<< cos(Xv(3))*cos(Xv(4)) << cos(Xv(3))*sin(Xv(4))*sin(Xv(5)) - sin(Xv(3))*cos(Xv(5)) << cos(Xv(3))*sin(Xv(4))*cos(Xv(5)) + sin(Xv(3))*sin(Xv(5)) <<endr
        << sin(Xv(3))*cos(Xv(4)) << sin(Xv(3))*sin(Xv(4))*sin(Xv(5)) + cos(Xv(3))*cos(Xv(5)) << sin(Xv(3))*sin(Xv(4))*cos(Xv(5)) - cos(Xv(3))*sin(Xv(5)) <<endr
        << -sin(Xv(4)) << cos(Xv(4))*sin(Xv(5)) << cos(Xv(4))*cos(Xv(5)) <<endr;

    ROTthe <<1<<0<<0<<endr
           <<0<<cos(angle_d)<<-sin(angle_d)<<endr
           <<0<<sin(angle_d)<<cos(angle_d)<<endr;

    ROTv=ROTv_i * ROTthe;

    n_d=ROTv.col(0);
    s_d=ROTv.col(1);
    a_d=ROTv.col(2);


    calcSkew(n_d, Sn_d);
    calcSkew(s_d, Ss_d);
    calcSkew(a_d, Sa_d);


    n_e=fkin_or_right.col(0);
    s_e=fkin_or_right.col(1);
    a_e=fkin_or_right.col(2);

    calcSkew(n_e, Sn_e);
    calcSkew(s_e, Ss_e);
    calcSkew(a_e, Sa_e);

    L_rot_R=-0.5*(Sn_d*Sn_e + Ss_d*Ss_e + Sa_d*Sa_e);
    Eo_r=0.5*(cross(n_e,n_d) + cross(s_e,s_d) + cross(a_e,a_d));

    //IK solution: CLICK
    vec V_R(6);
    V_R.subvec(0,2) = dXd_R.subvec(0,2) + K_clik*(Xd_R.subvec(0,2) - X_R.subvec(0,2));
    V_R.subvec(3,5) =  Eo_r;


    //relative motion
    delta_q = K_inv * pinv_Js_1 * V_R + K_inv * pinv_Jhat_2 * (dXd_D + K_clik * (Xd_D-X_D) - Js_2 * pinv_Js_1 * V_R);// + K_null * (I_16-pinv_Js_1*Js_1)*(I_16-pinv_Jhat_2*Jhat_2)*(-lambda_dot_jntlmt);

    //rest:::testing
    //delta_q = K_inv * pinv_Js_1 * (dXd_D + K_clik * (Xd_D-X_D)) + K_inv * pinv_Jhat_2 * (V_R - Js_2 * pinv_Js_1 * (dXd_D + K_clik * (Xd_D-X_D)));// + (I_16-pinv_Js_1*Js_1)*(I_16-pinv_Jhat_2*Jhat_2)*(-lambda_dot_jntlmt);

    //delta_q_cds=delta_t_cds*(pinv_Js_1*(x_dot_1 + k_lim*(x_1_d-x_1_r)) + pinv_Jhat_2*(x_dot_2 + k_lim*(x_2_d-x_2_r) - Js_2*pinv_Js_1*(x_dot_1+ k_lim*(x_1_d-x_1_r))) + (Ident.eye()-pinv_Js_1*Js_1)*(Ident.eye()-pinv_Jhat_2*Jhat_2)*lambda_dot*null_control);

    //delta_q.rows(0,7)=  K_inv* pinv_jacob_right*V_R + K_null * (I_8 - pinv_jacob_right*jacob_right) * (-lambda_dot_jntlmt_r);
    //delta_q.rows(8,15)= K_inv* pinv_jacob_left *V_L + K_null * (I_8 - pinv_jacob_left*jacob_left) * (-lambda_dot_jntlmt_l);

    //Relative between two-arms

    // delta_q= K_inv* pinv_jacob_R* (Xd_D-X_D);
    /*mat jacob_R_new;
    jacob_R_new=jacob_R;
    jacob_R_new.col(7)=z6;
    jacob_R_new.col(15)=z6;
    //cout<<jacob_R_new<<endl;

    null_vel(7)=0;
    null_vel(15)=0;
    delta_q= 0.00005* (I_16 - pinv(jacob_R_new)*jacob_R_new)*null_vel;*/


    //vec delta_q_resized;
    //delta_q_resized= - 0.00005* (I_14 - pinv(jacob_R_resized)*jacob_R_resized)*null_vel;

    //delta_q.rows(0,6)=delta_q_resized.rows(0,6);
    //delta_q.rows(8,14)=delta_q_resized.rows(7,13);

    // delta_q(14)=  delta_q(14) - 0.0000001;
    //delta_q.rows(8,15)= -100 * K_inv* pinv_jacob_left*C_vel;

}

hand_pos ManipulationVars::grasping(){

    //right hand
    if ((q_ref(7) < 1.3))
    {
        q_ref(7) =q_ref(7) + 0.0005;
    }

    //left hand
    if ((q_ref(15) < 1.3))
    {
        q_ref(15) =q_ref(15) + 0.0005;
    }

    hand_pos temp;
    temp.r=q_ref(7);
    temp.l=q_ref(15);
    return temp;
}

hand_pos ManipulationVars::openning(){

    //right hand
    if ((q_ref(7) > 0.30))
    {
        q_ref(7) =q_ref(7) - 0.0005;

    }

    //left hand
    if ((q_ref(15) > 0.30))
    {
        q_ref(15) =q_ref(15) - 0.0005;

    }


    hand_pos temp;
    temp.r=q_ref(7);
    temp.l=q_ref(15);
    return temp;
}

const vec &ManipulationVars::get_valve_data(){
    vec temp(6);
    for (unsigned int i=0;i<6;i++)
        temp(i)=valve_data(i);
    return temp;
}

double ManipulationVars::get_radius(){
    return valve_data(6);
}

void ManipulationVars::set_valve_data(vec valve_data){
    this->valve_data=valve_data;
}

bool ManipulationVars::testsafety(){

    safety_flag=0;
    vec q_max_all = join_cols(q_max_r,q_max_l);
    vec q_min_all = join_cols(q_min_r,q_min_l);

    for (int my_jnt_n =0; my_jnt_n<15; my_jnt_n++)
        {
            if (q_l(my_jnt_n) > 0.95 * q_max_all(my_jnt_n)){
                   safety_flag=safety_flag+1;
                   std::cout << "Max. Joint= " << my_jnt_n
                             << " with value " << q_l(my_jnt_n)
                             << " out of " << q_max_all(my_jnt_n)
                             << endl;
            }

            if (q_l(my_jnt_n) <0.95 * q_min_all(my_jnt_n)){
                   safety_flag=safety_flag+1;
                   std::cout << "Min. Joint= " << my_jnt_n
                             << " with value " << q_l(my_jnt_n)
                             << " out of " << q_min_all(my_jnt_n)
                             << endl;
            }
        }


      /* for (int my_jnt_n =0; my_jnt_n<14; my_jnt_n++)
       {
          if ((_pos[two_arms_nohands[my_jnt_n]-1] - _ts_bc_data[two_arms_nohands[my_jnt_n]-1].raw_bc_data.mc_bc_data.Position)  > 100000.0 * 5 * M_PI/180.0)
                  safety_flag=safety_flag+1;
       }*/

    if(safety_flag == 0)
        return true;
    return false;
}

Boards_ctrl_basic::Boards_ctrl_basic(const char * config): Boards_ctrl_ext(config) {

    name = "boards_ctrl_basic";
    period.period = {0,1000};

#ifdef __XENO__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max(schedpolicy);
    //stacksize = PTHREAD_STACK_MIN;

    xddp_test = new Write_XDDP_pipe(std::string(name), 4096);

    log_user_buff_Cart.set_capacity(LOG_SIZE);

    for (int i=0; i<16; i++)
    {
        log_user_buff[i].set_capacity(LOG_SIZE);
    }

}

Boards_ctrl_basic::~Boards_ctrl_basic() {

    //Save Cartesian data
    std::string filename = str(boost::format("/home/coman/AJ/COMAN_shared/examples/Arash_arma/Expdata/user_log_Cart.txt") );
    std::ofstream log_file(filename.c_str());

    for (boost::circular_buffer<log_user_t>::iterator it=log_user_buff_Cart.begin(); it!=log_user_buff_Cart.end(); it++) {
        log_file << boost::format("%1%\t") % (*it).ts;          //1

        log_file << boost::format("%1%\t") % (*it).CartXd_R;     //2     //currently, it is Eo_r
        log_file << boost::format("%1%\t") % (*it).CartYd_R;
        log_file << boost::format("%1%\t") % (*it).CartZd_R;

        log_file << boost::format("%1%\t") % (*it).CartXd_L;     //5     //currently, it is Eo_l
        log_file << boost::format("%1%\t") % (*it).CartYd_L;
        log_file << boost::format("%1%\t") % (*it).CartZd_L;

        log_file << boost::format("%1%\t") % (*it).CartX_R;     //8
        log_file << boost::format("%1%\t") % (*it).CartY_R;
        log_file << boost::format("%1%\t") % (*it).CartZ_R;

        log_file << boost::format("%1%\t") % (*it).CartX_L;     //11
        log_file << boost::format("%1%\t") % (*it).CartY_L;
        log_file << boost::format("%1%\t") % (*it).CartZ_L;


        log_file << boost::format("%1%\t") % (*it).rel_x;       //14
        log_file << boost::format("%1%\t") % (*it).rel_y;
        log_file << boost::format("%1%\n") % (*it).rel_z;
    }
    log_file << std::flush;
    log_file.close();


/*
    //Save data for joints
    for (int i=0; i<16; i++){
            std::string filename = str(boost::format("/home/coman/AJ/COMAN_shared/examples/Arash_arma/Expdata/user_log_%1%.txt") % (int)i);
            std::ofstream log_file(filename.c_str());

            for (boost::circular_buffer<log_user_t>::iterator it=log_user_buff[i].begin(); it!=log_user_buff[i].end(); it++) {
                //while ( ! dsp_log.empty() ) {
                log_file << boost::format("%1%\t") % (*it).ts;
                log_file << boost::format("%1%\n") % (*it).qm;
            }
            log_file << std::flush;
            log_file.close();
        }
*/

    delete xddp_test;

    std::cout << "~" << typeid(this).name() << std::endl;
}

void Boards_ctrl_basic::th_init(void *) {

    init();
    homing();// read current position and set as homing
    test();
    trj_flag = 0;
    reach_flag = 0;
    push_flag=0;
    open_hand_flag=0;
    close_hand_flag = 0;
    g_tStart = 0;
    t_traj_start= 0;
    valve_rotate_flag = 0;
    moving_far_flag=0;

    //initialize variables
    Xi_R.zeros(); Xi_L.zeros();
    Xd_R.zeros(); Xd_L.zeros(); Xd_D.zeros();
    dXd_R.zeros(); dXd_L.zeros(); dXd_D.zeros();

}

void ManipulationVars::init_manip(vec _q_l)
{

    q_ref=_q_l;
    q_h=_q_l;
    delta_q.zeros();
    delta_q_sum.zeros();

    C_vel.zeros();
    null_vel.ones();

    C_vel(1)= 0.0001;

    flag_run_once = true;

}

void Boards_ctrl_basic::th_loop(void * ) { 


    uint8_t cmd;
    log_user_t tmp_log;
    log_user_t tmp_log_Cart;

    static char console_buffer[1024];
    static char user_data[1024];
    try {

       sense();

       for (int i=0; i<16; i++)
        {
            q_l(i)=double(_ts_bc_data[two_arms[i]-1].raw_bc_data.mc_bc_data.Position/100000.0);
            q_dot(i)=double(_ts_bc_data[two_arms[i]-1].raw_bc_data.mc_bc_data.Velocity/1000.0);

            if (i<14) tau(i)=double(_ts_bc_data[two_arms[i]-1].raw_bc_data.mc_bc_data.Torque/1000.0);
        }


        if(flag_run_once == false){
            init_manip(q_l);

            /////////////LPF TEST
            qf_l = q_l;
            qf_l_old = q_l;
        }


        qf_l = (lamda*0.001*q_l + qf_l_old)/(1.0+0.001*lamda);
        qf_l_old = qf_l;
        q_l.subvec(4,6) = qf_l.subvec(4,6);     //LPF activated
        q_l.subvec(12,14) = qf_l.subvec(12,14);

        q_l_resized.rows(0,6) = q_l.rows(0,6);
        q_l_resized.rows(7,13) = q_l.rows(8,14);

        bzero((void*)console_buffer, sizeof(console_buffer));
        user_input(console_buffer, sizeof(console_buffer));

        user_loop();

        sprintf(user_data, "%ld\n", get_time_ns());
        xddp_test->write((void*)user_data, strlen(user_data));

    } catch ( boards_error &e ){
        DPRINTF("FATAL ERROR in %s ... %s\n", __FUNCTION__, e.what());
        // handle error .... exit thread
        // exi_ts_bc_datat {rt,nrt}_periodic_thread function
        _run_loop = 0;

    } catch ( boards_warn &e ) {
        DPRINTF("WARNING in %s ... %s\n", __FUNCTION__, e.what());
        // handle warning
    }


    // ***** SAVE USER LOG

    // log user data
    tmp_log_Cart.ts = get_time_ns();//(get_time_ns() - g_tStart);

    tmp_log_Cart.CartXd_R =  X_D(0);
    tmp_log_Cart.CartYd_R =  X_D(1);
    tmp_log_Cart.CartZd_R =  X_D(2);

    tmp_log_Cart.CartXd_L =  X_D(3);
    tmp_log_Cart.CartYd_L =  X_D(4);
    tmp_log_Cart.CartZd_L =  X_D(5);

    tmp_log_Cart.CartX_R  = Xd_D(0);
    tmp_log_Cart.CartY_R  = Xd_D(1);
    tmp_log_Cart.CartZ_L = Xd_D(2);


    tmp_log_Cart.rel_x   = Xd_D(3);
    tmp_log_Cart.rel_y   = Xd_D(4);
    tmp_log_Cart.rel_z   = Xd_D(5);

    //tmp_log_Cart.CartXd_R = Xd_R(0);    tmp_log_Cart.CartYd_R = Xd_R(1);    tmp_log_Cart.CartZd_R = Xd_R(2);
    //tmp_log_Cart.CartXd_L = Xd_L(0);    tmp_log_Cart.CartYd_L = Xd_L(1);    tmp_log_Cart.CartZd_L = Xd_L(2);

    //tmp_log_Cart.CartXd_R = Eo_r(0);    tmp_log_Cart.CartYd_R = Eo_r(1);    tmp_log_Cart.CartZd_R = Eo_r(2);
    //tmp_log_Cart.CartXd_L = Eo_l(0);    tmp_log_Cart.CartYd_L = Eo_l(1);    tmp_log_Cart.CartZd_L = Eo_l(2);

    //tmp_log_Cart.CartX_R = fkin_po_right(0);
    //tmp_log_Cart.CartY_R = fkin_po_right(1);
    //tmp_log_Cart.CartZ_R = fkin_po_right(2);

    //tmp_log_Cart.CartX_L = fkin_po_left(0);
    //tmp_log_Cart.CartY_L = fkin_po_left(1);
    //tmp_log_Cart.CartZ_L = fkin_po_left(2);


    //tmp_log_Cart.rel_x = obj_r_T(0);    tmp_log_Cart.rel_y = obj_r_T(1);    tmp_log_Cart.rel_z = obj_r_T(2);
    //tmp_log_Cart.rel_x = Eo_r(0);    tmp_log_Cart.rel_y = Eo_r(1);    tmp_log_Cart.rel_z = Eo_r(2);

    log_user_buff_Cart.push_back(tmp_log_Cart);


    // log user data
    /*
    for (int i=0;i<16; i++)
    {
            tmp_log.ts = get_time_ns();//(get_time_ns() - g_tStart);
            tmp_log.des_pos = (_pos[two_arms_nohands[i]-1])/100000.0*180.0/M_PI;
            tmp_log.qm = q_l(i)*180.0/M_PI;

            log_user_buff[i].push_back(tmp_log);
    }
    */


}

void Boards_ctrl_basic::reset()
{
    g_tStart = -1;
    cout<<"reset() is called!="<<g_tStart<<endl;
}

int Boards_ctrl_basic::user_loop(void) {


    manip_kine(); //Calculate kinematics

    hand_pos hand_delta_q; //Grasping/Openning requirements
    hand_delta_q.l=0;
    hand_delta_q.r=0;


    if (count_loop_1==500)
    {

       // cout<<"traj flag="<<trj_flag<<endl;

       // cout<<"reach flag="<<reach_flag<<endl;
       // cout<<"rotation flag="<<valve_rotate_flag<<endl;
       // cout<<endl;
               //close_hand_flag
        //open_hand_flag
        //(dXd_D.t() + K_clik * (Xd_D.t()-X_D.t())).print();

        //(joint_error_ar.t()).print();

       // cout<< det(jacob_right*jacob_right.t()) <<endl;
       // (Qr.t()).print("Qr=");
       // (Ql.t()).print("Ql=");
        //(180/M_PI*q_l.t()).print("q_l=");
        //(Eo_l.t()).print("Eo_l=");
        //(Eo_r.t()).print("Eo_r=");
      //  cout<<endl;
        /*(Eo_r.t()).print("Eo_r=");
        (Eo_l.t()).print("Eo_l=");
        cout<<endl;*/
        /*Rd.print("Rd=");
        //R.print("Rot=");
        Rfkin.print("Rfkin=");
        (Qd.t()).print("Qd=");
        (Qe.t()).print("Qe=");



        (Orien_Err.t()).print("OErr=");
        cout<<endl;*/

        //(round(1000*(obj_r_T.t()))).print();
        //(round((180/M_PI)*((join_cols(q_bar_r,q_bar_l)).t() - q_l.t()))).print();
        //(round((180/M_PI)*((q_l.t())))).print("q_l");
        //
        count_loop_1=0;
    }
    ++count_loop_1;



    //Trajectrory--- let the robot move------------------------------//
        if ( trj_flag == 1 ) {


            //Reaching-------------------------------------------------------//
                if ( reach_flag == 1 ) {                
                u_int64_t dt_ns= 0;
                if ( g_tStart <= 0 ) {
                    g_tStart = get_time_ns();
                }
                dt_ns = get_time_ns() - g_tStart;
                    reaching(dt_ns);
                }
            //Reaching--------------------------------------------------------//


            //GRASPING  ------------------------------------------------------//
                if ( close_hand_flag == 1 )
                {
                    hand_delta_q = grasping();
                }
            //GRASPING  -----------------------------------------------------//

            //OPENNING ------------------------------------------------------//
                if ( open_hand_flag == 1 )
                {
                    hand_delta_q = openning();
                }
            //OPENNING ------------------------------------------------------//


            //ROTATING-------------------------------------------------------//
                if ( valve_rotate_flag == 1 )
                {                    
                    u_int64_t dt_ns= 0;
                    if ( g_tStart <= 0 ) {
                        g_tStart = get_time_ns();
                    }
                    dt_ns = get_time_ns() - g_tStart;
                    rotating(dt_ns);
                }
            //ROTATING---------------------------------------------------------//

            //PUSHING  --------------------------------------------------------//
                if ( push_flag == 1 )
                {
                    u_int64_t dt_ns= 0;
                    if ( g_tStart <= 0 ) {
                        g_tStart = get_time_ns();
                    }
                    dt_ns = get_time_ns() - g_tStart;
                        pushing(dt_ns);
                }
            //PUSHING ----------------------------------------------------------//

            //MOVING FAR--------------------------------------------------------//
                if ( moving_far_flag == 1 )
                { u_int64_t dt_ns= 0;
                    if ( g_tStart <= 0 ) {
                        g_tStart = get_time_ns();
                    }
                    dt_ns = get_time_ns() - g_tStart;
                        movingfar(dt_ns);
                }
            //MOVING FAR--------------------------------------------------------//


            //CALCULATE JOINT REFERENCES----------------------------------------//
                delta_q_sum = delta_q_sum + delta_q;
                delta_q_sum(7)=0;
                delta_q_sum(15)=0;


                for (int my_jnt_n =0; my_jnt_n<15; my_jnt_n++)
                {
                    _pos[two_arms[my_jnt_n]-1] = _home[two_arms[my_jnt_n]-1] + 100000.0 * delta_q_sum(my_jnt_n);

                    //joint_error_ar(my_jnt_n) = _pos[two_arms[my_jnt_n]-1] -(_home[two_arms[my_jnt_n]-1] + 100000.0 * delta_q_sum(my_jnt_n));
                }

                delta_q_sum(7)=hand_delta_q.r;//exclude hand_right
                delta_q_sum(15)=hand_delta_q.l;//exclude hand_left

                _pos[two_arms[7]-1] = 100000.0 * delta_q_sum(7);
                _pos[two_arms[15]-1] = 100000.0 * delta_q_sum(15);
            //CALCULATE JOINT REFERENCES---------------------------------------//

                // testsafety();//Safety for joint limits

                   if(safety_flag==0)
                     {
                           move(MV_POS|MV_VEL|MV_TOR|MV_STF);
                     }
                 else
                    {
                       if(safety_flag!=0)
                          DPRINTF("Exceeding Joint Limits/Speed -- Control Stopped... \n");
                    }

    //Trajectrory--- stop the robot movements------------------------//
      }
    return 0;
}

int Boards_ctrl_basic::user_input(void *buffer, ssize_t buff_size) {

    static int toggle = 1;
    McBoard * b;
    uint8_t cmd;

    // fakes ids ... used for test group functions
    uint8_t bIds[2] = { 88, 99};
    int pos[2];
    short vel[2];

    int nbytes = Boards_ctrl_ext::user_input(buffer, buff_size);

    if ( nbytes <= 0 ) {
        return nbytes;
    }

    // parse input
    std::vector<int> parsed_int;
    cmd = parse_console(buffer, nbytes, &parsed_int);
    if ( parsed_int.size() ) {
        for (auto it = parsed_int.begin(); it != parsed_int.end(); it++) {
            DPRINTF("%d ", *it);
        }
        DPRINTF("\n");
    }

    switch ( cmd ) {
        case 'S':
            DPRINTF("Start control ...two_arms\n");
            start_stop_set_control(two_arms,true);
            break;

        case 'h':
            DPRINTF("Set home pos\n");
            homing(homePos, homeVel);        
            break;

        case 't':
            DPRINTF("trajectory\n");
            homing();
            reset();
            //g_tStart = get_time_ns();
            trj_flag = ! trj_flag;
            break;

        case 'r':
            DPRINTF("reaching\n");
            reset();
            push_flag=0;
            moving_far_flag=0;
            valve_rotate_flag = 0;
            flag_init_hands=false;
            init_rot_po=false;
            flag_init_pushing=false;
            flag_init_movingfar=false;
            reach_flag = ! reach_flag;
            break;

        case 'p':
            DPRINTF("pushing towards the valve\n");
            reset();
            valve_rotate_flag = 0;
            moving_far_flag=0;
            reach_flag = 0;
            flag_init_hands=false;
            init_rot_po=false;
            flag_init_pushing=false;
            flag_init_movingfar=false;
            push_flag = ! push_flag;
            break;

        case 'o':
            DPRINTF("openning the hand\n");
            close_hand_flag = 0;
            open_hand_flag = ! open_hand_flag;
        break;

        case 'm':
            DPRINTF("moving far from the valve\n");
            reset();
            reach_flag=0;
            push_flag=0;
            valve_rotate_flag=0;
            flag_init_hands=false;
            init_rot_po=false;
            flag_init_pushing=false;
            flag_init_movingfar=false;
            moving_far_flag = ! moving_far_flag;
        break;

        case 'c':
            DPRINTF("closing hands\n");
            open_hand_flag = 0;
            close_hand_flag = ! close_hand_flag;
            break;

        case 'v':
            DPRINTF("valve rotation\n");
            reset();
            reach_flag = 0;
            push_flag=0;
            moving_far_flag=0;
            flag_init_hands=false;
            init_rot_po=false;
            flag_init_pushing=false;
            flag_init_movingfar=false;
            valve_rotate_flag = ! valve_rotate_flag;
            break;

        default:
            DPRINTF("Stop control ...\n");
            start_stop_control(false);
            clear_mcs_faults();
            break;

    }

    return nbytes;
}


uint8_t Boards_ctrl_basic::parse_console(void * buffer, ssize_t buff_size, void * parsed) {

    char tmp[1024];
    uint8_t ret;

    // parse input
    bzero((void*)tmp, sizeof(tmp));
    memcpy((void*)tmp, buffer, buff_size);
    printf("%d : %s\n", buff_size, tmp);

    typedef boost::tokenizer< boost::char_separator<char> > Tokenizer;
    boost::char_separator<char> sep(";");
    std::string line(tmp);

    Tokenizer info(line, sep); // tokenize the line of data

    int cnt = 0;
    for (Tokenizer::iterator it = info.begin(); it != info.end(); ++it) {

        switch (cnt) {
            case 0:
                if (it->length() > 0) {
                    //std::cout << it->c_str() << std::endl;
                    ret = it->c_str()[0];
                }
                break;
            default:
                ((std::vector<int>*)parsed)->push_back(atoi(it->c_str()));
                break;
        }

        cnt ++;
    }

    return ret;


}

