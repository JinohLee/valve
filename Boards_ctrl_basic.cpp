/*
   Boards_ctrl_basic.cpp

   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2013-, alessio.margan@iit.it)

*/

/**
 *
 * @author Alessio Margan (2013-, alessio.margan@iit.it)
*/
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


#include <Boards_ctrl_basic.h>
#include <Boards_exception.h>

#include <boost/tokenizer.hpp>
//#include <boost/numeric/ublas/mat.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/format.hpp>

#include <armadillo>
using namespace arma;
//#include "calc_jacob.h"
#include "mySkew.h"
#include "Traj_gen.h" //trajectory generation

//For kinematics
mat jacob_right(6, 8), jacob_left(6, 8), jacob_R(6, 16), jacob_R_1(3,16), jacob_R_2(3,16), zeros6b8(6,8);
mat fkin_or_left(3,3), fkin_or_right(3,3), fkin_po_left(3,1), fkin_po_right(3,1);
mat  I_16(16,16), I_8(8,8), I_6(6,6), obj_R_G(3,3), o_R_A(3,3), o_R_B(3,3);
mat o_base(3,1), o_r_A(3,1), o_r_B(3,1), obj_r_G(3,1);
mat A_R_hat_BA(3,3), A_R_hat_T(3,3), A_R_hat_G(3,3), J_11(3,8), J_12(3,8), J_21(3,8), J_22(3,8);
mat J_p_L(3,8), J_p_R(3,8), J_o_L(3,8), J_o_R(3,8);
mat pinv_jacob_right, pinv_jacob_left, pinv_jacob_R, Js_1, Js_2, pinv_Js_1, Jhat_2, pinv_Jhat_2;

//For sine, cos
double L0=0.15154, L1=0.1375, L2=0.19468, a3=0.005, Le=0.010;
double S1=0,S2=0,S3=0,S4=0,S5=0,S6=0,S7=0,S8=0,S9=0,S10=0,S11=0,S12=0,S13=0,S14=0,S15=0,S16=0;
double C1=0,C2=0,C3=0,C4=0,C5=0,C6=0,C7=0,C8=0,C9=0,C10=0,C11=0,C12=0,C13=0,C14=0,C15=0,C16=0;
vec Theta_0_R(8), Theta_0_L(8), q_l(16), q_dot(16), tau(14), C_vel(6), q_ref(16), q_h(16), delta_q(16), delta_q_sum(16), null_vel(16), obj_r_T(3), obj_w_T(3);
vec lambda_dot_jntlmt_r(8),lambda_dot_jntlmt_l(8), lambda_dot_jntlmt(16), q_max_r(8), q_min_r(8), q_bar_r(8), q_max_l(8), q_min_l(8), q_bar_l(8);
//vec SCin;

//for trajectory
vec Xi_R(6), Xi_L(6);
vec dXd_D(6), Xd_D(6);
vec Xd_R(6), Xd_L(6);
vec dXd_R(6), dXd_L(6);
vec X_R, X_L, X_D;
vec Xd_D_init(6);

bool flag_init_hands = false;

double K_inv   = 0.001;     //Tuned (0.02* 0.05)
double K_clik  = 1.5;       //Tuned
double K_null  = 0.001;     //not tuned


//flags
bool flag_run_once = false;
int safety_flag=0, jntlmt_flag=0;
int count_loop_1=0, hand_flag_control_r=0, hand_flag_control_l=0;
using namespace boost::numeric::ublas;


static const std::vector<float> homeVel(25,25);

/*static const std::vector<float> homePos = {
    // lower body #15
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
//  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15
    // upper body #10
    0, 87,  0, -3,  0,-87,  0, -3,  0,  0};
// 16, 17, 18, 19, 20, 21, 22, 23, 24, 25 */

static const std::vector<float> homePos = {
    // lower body #15
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
//  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15
    // upper body #10
    0, 60,  0, -45, 0, -60, 0,  -45, 0,  0,  0,  0, 0,  0 , 0,  0 };
// 16, 17, 18, 19, 20,  21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31

// boards ID
std::vector<int> r_leg = {  4,  6,  7,  8,  9, 10};
std::vector<int> l_leg = {  5, 11, 12, 13, 14, 15};
std::vector<int> waist = {  1,  2, 3};
std::vector<int> r_arm = { 16, 17, 18 ,19};
std::vector<int> l_arm = { 20, 21, 22, 23};
std::vector<int> neck  = {}; //{ 24, 25};

std::vector<int> two_arms = {16, 17, 18 ,19, 26, 27, 28, 32,
                             20, 21, 22, 23, 29, 30, 31, 33};

std::vector<int> two_arms_nohands = {16, 17, 18 ,19, 26, 27, 28,
                                     20, 21, 22, 23, 29, 30, 31};
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

        log_file << boost::format("%1%\t") % (*it).CartXd_R;     //2
        log_file << boost::format("%1%\t") % (*it).CartYd_R;
        log_file << boost::format("%1%\t") % (*it).CartZd_R;

        log_file << boost::format("%1%\t") % (*it).CartXd_L;     //5
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



    // configure dsp and start bc data
    // NOT start motor controller
    init();
    // read current position and set as homing
    homing();
    test();
    trj_flag = 0;
    close_hand_flag = 0;
    g_tStart = 0;
    t_traj_start= 0;
    rotate_valve_flag = 0;

    //initialize variables
    Xi_R.zeros(); Xi_L.zeros();
    Xd_R.zeros(); Xd_L.zeros(); Xd_D.zeros();
    dXd_R.zeros(); dXd_L.zeros(); dXd_D.zeros();


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

            q_ref=q_l;
            q_h=q_l;
            delta_q.zeros();
            delta_q_sum.zeros();

            C_vel.zeros();
            null_vel.ones();

            C_vel(2)= 0.0001;

            flag_run_once = true;
        }

        bzero((void*)console_buffer, sizeof(console_buffer));
        user_input(console_buffer, sizeof(console_buffer));

        user_loop();

        sprintf(user_data, "%ld\n", get_time_ns());
        xddp_test->write((void*)user_data, strlen(user_data));

    } catch ( boards_error &e ){
        DPRINTF("FATAL ERROR in %s ... %s\n", __FUNCTION__, e.what());
        // handle error .... exit thread
        // exit {rt,nrt}_periodic_thread function
        _run_loop = 0;

    } catch ( boards_warn &e ) {
        DPRINTF("WARNING in %s ... %s\n", __FUNCTION__, e.what());
        // handle warning
    }


    // ***** SAVE USER LOG

    // log user data
    tmp_log_Cart.ts = get_time_ns();//(get_time_ns() - g_tStart);
    tmp_log_Cart.CartXd_R = Xd_R(0);    tmp_log_Cart.CartYd_R = Xd_R(1);    tmp_log_Cart.CartZd_R = Xd_R(2);
    tmp_log_Cart.CartXd_L = Xd_L(0);    tmp_log_Cart.CartYd_L = Xd_L(1);    tmp_log_Cart.CartZd_L = Xd_L(2);

    //tmp_log_Cart.CartX_R = dXd_R(0);
    //tmp_log_Cart.CartY_R = dXd_R(1);
    //tmp_log_Cart.CartZ_R = dXd_R(2);

    //tmp_log_Cart.CartX_L = dXd_L(0);
    //tmp_log_Cart.CartY_L = dXd_L(1);
    //tmp_log_Cart.CartZ_L = dXd_L(2);

    tmp_log_Cart.CartX_R = fkin_po_right(0);
    tmp_log_Cart.CartY_R = fkin_po_right(1);
    tmp_log_Cart.CartZ_R = fkin_po_right(2);

    tmp_log_Cart.CartX_L = fkin_po_left(0);
    tmp_log_Cart.CartY_L = fkin_po_left(1);
    tmp_log_Cart.CartZ_L = fkin_po_left(2);


    tmp_log_Cart.rel_x = obj_r_T(0);    tmp_log_Cart.rel_y = obj_r_T(1);    tmp_log_Cart.rel_z = obj_r_T(2);


    log_user_buff_Cart.push_back(tmp_log_Cart);


    // log user data
    for (int i=0;i<16; i++)
    {
            tmp_log.ts = get_time_ns();//(get_time_ns() - g_tStart);
            tmp_log.des_pos = (_pos[two_arms_nohands[i]-1])/100000.0*180.0/M_PI;
            tmp_log.qm = q_l(i)*180.0/M_PI;

            log_user_buff[i].push_back(tmp_log);
    }


}

int Boards_ctrl_basic::user_loop(void) {

    int bId;

    I_16.eye();
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
    q_bar_r(6) = 5.0*M_PI/180.0;    //wrist

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
    q_bar_l(6) = -5.0*M_PI/180.0;

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

    J_p_L<<a3*(C9*S11 - C10*C11*S9) - L2*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) + Le*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11))) - L1*S9*S10<< C9*(L2*(C10*C12 + C11*S10*S12) + L1*C10 - Le*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12))) - a3*C11*S10)<< S9*S10*(Le*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12))) - L2*(C10*C12 + C11*S10*S12) + a3*C11*S10) - C10*(a3*(C9*S11 - C10*C11*S9) - L2*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) + Le*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11))))<< - (L2*(C10*C12 + C11*S10*S12) - Le*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12))))*(C9*C11 + C10*S9*S11) - S10*S11*(L2*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - Le*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11))))<< Le*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10)*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12))) - Le*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11)))*(C10*C12 + C11*S10*S12)<< - Le*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12)))*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11)) - Le*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11)*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11)))<< - Le*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11)))*(S14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) - C14*(C10*C12 + C11*S10*S12)) - Le*(C14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) + S14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11)))*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12)))<< 0<<endr
     <<0<< S9*(a3*(C9*S11 - C10*C11*S9) - L2*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) + Le*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11))) - L1*S9*S10) - C9*(a3*(S9*S11 + C9*C10*C11) - L2*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) + Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11))) + L1*C9*S10)<< S9*S10*(a3*(S9*S11 + C9*C10*C11) - L2*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) + Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11)))) + C9*S10*(a3*(C9*S11 - C10*C11*S9) - L2*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) + Le*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11))))<< (C11*S9 - C9*C10*S11)*(L2*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - Le*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11)))) - (C9*C11 + C10*S9*S11)*(L2*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11))))<< Le*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10)*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11))) - Le*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11)))*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10)<< Le*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11))*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11))) - Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11)))*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11))<< Le*(C14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) + S14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11)))*(C15*(S14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - C14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))) - S15*(S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) - C13*(C9*C11 + C10*S9*S11))) - Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11)))*(C14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) + S14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11)))<< 0<<endr
     <<L2*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - a3*(S9*S11 + C9*C10*C11) - Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11))) - L1*C9*S10<< -S9*(L2*(C10*C12 + C11*S10*S12) + L1*C10 - Le*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12))) - a3*C11*S10)<< C10*(a3*(S9*S11 + C9*C10*C11) - L2*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) + Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11)))) + C9*S10*(Le*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12))) - L2*(C10*C12 + C11*S10*S12) + a3*C11*S10)<< (L2*(C10*C12 + C11*S10*S12) - Le*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12))))*(C11*S9 - C9*C10*S11) + S10*S11*(L2*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11))))<< Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11)))*(C10*C12 + C11*S10*S12) - Le*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12)))*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10)<< Le*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12)))*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11)) + Le*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11)*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11)))<< Le*(C15*(S14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - C14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))) - S15*(S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) - C13*(C11*S9 - C9*C10*S11)))*(S14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) - C14*(C10*C12 + C11*S10*S12)) + Le*(C14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) + S14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11)))*(S15*(S13*(C10*S12 - C11*C12*S10) - C13*S10*S11) + C15*(C14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13) + S14*(C10*C12 + C11*S10*S12)))<< 0<<endr;

    J_o_L<<0<< -S9<< -C9*S10<< C11*S9 - C9*C10*S11<< S12*(S9*S11 + C9*C10*C11) - C9*C12*S10<< C13*(C11*S9 - C9*C10*S11) - S13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12)<< - C14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - S14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))<< - C14*(S12*(S9*S11 + C9*C10*C11) - C9*C12*S10) - S14*(C13*(C12*(S9*S11 + C9*C10*C11) + C9*S10*S12) + S13*(C11*S9 - C9*C10*S11))<<endr
     <<1<< 0<< -C10<< S10*S11<< - C10*C12 - C11*S10*S12<< C13*S10*S11 - S13*(C10*S12 - C11*C12*S10)<< C14*(C10*C12 + C11*S10*S12) - S14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13)<< C14*(C10*C12 + C11*S10*S12) - S14*(C13*(C10*S12 - C11*C12*S10) + S10*S11*S13)<<endr
     <<0<< -C9<< S9*S10<< C9*C11 + C10*S9*S11<< S12*(C9*S11 - C10*C11*S9) + C12*S9*S10<< C13*(C9*C11 + C10*S9*S11) - S13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12)<< - C14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - S14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))<< - C14*(S12*(C9*S11 - C10*C11*S9) + C12*S9*S10) - S14*(C13*(C12*(C9*S11 - C10*C11*S9) - S9*S10*S12) + S13*(C9*C11 + C10*S9*S11))<<endr;


    J_p_R<<L2*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - a3*(C1*S3 + C2*C3*S1) - Le*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3))) + L1*S1*S2<< -C1*(L2*(C2*C4 - C3*S2*S4) + L1*C2 - Le*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4))) + a3*C3*S2)<< - C2*(a3*(C1*S3 + C2*C3*S1) - L2*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) + Le*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3)))) - S1*S2*(L2*(C2*C4 - C3*S2*S4) - Le*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4))) + a3*C3*S2)<< S2*S3*(L2*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - Le*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3)))) - (L2*(C2*C4 - C3*S2*S4) - Le*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4))))*(C1*C3 - C2*S1*S3)<< Le*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2)*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4))) - Le*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3)))*(C2*C4 - C3*S2*S4)<< - Le*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4)))*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3)) - Le*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3)*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3)))<< - Le*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3)))*(S6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) - C6*(C2*C4 - C3*S2*S4)) - Le*(C6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) + S6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3)))*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4)))<< 0<<endr
     <<0<< C1*(a3*(S1*S3 - C1*C2*C3) - L2*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) + Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3))) + L1*C1*S2) - S1*(a3*(C1*S3 + C2*C3*S1) - L2*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) + Le*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3))) - L1*S1*S2)<< S1*S2*(a3*(S1*S3 - C1*C2*C3) - L2*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) + Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3)))) + C1*S2*(a3*(C1*S3 + C2*C3*S1) - L2*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) + Le*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3))))<< (C3*S1 + C1*C2*S3)*(L2*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - Le*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3)))) - (C1*C3 - C2*S1*S3)*(L2*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3))))<< Le*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2)*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3))) - Le*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3)))*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2)<< Le*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3))*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3))) - Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3)))*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3))<< Le*(C6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) + S6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3)))*(C7*(S6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) - C6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))) - S7*(S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3))) - Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3)))*(C6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) + S6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3)))<< 0<<endr
     <<a3*(S1*S3 - C1*C2*C3) - L2*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) + Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3))) + L1*C1*S2<< S1*(L2*(C2*C4 - C3*S2*S4) + L1*C2 - Le*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4))) + a3*C3*S2)<< C2*(a3*(S1*S3 - C1*C2*C3) - L2*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) + Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3)))) - C1*S2*(L2*(C2*C4 - C3*S2*S4) - Le*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4))) + a3*C3*S2)<< (L2*(C2*C4 - C3*S2*S4) - Le*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4))))*(C3*S1 + C1*C2*S3) - S2*S3*(L2*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3))))<< Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3)))*(C2*C4 - C3*S2*S4) - Le*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4)))*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2)<< Le*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4)))*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3)) + Le*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3)*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3)))<< Le*(C7*(S6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) - C6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))) - S7*(S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3)))*(S6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) - C6*(C2*C4 - C3*S2*S4)) + Le*(C6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) + S6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3)))*(S7*(S5*(C2*S4 + C3*C4*S2) + C5*S2*S3) + C7*(C6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) + S6*(C2*C4 - C3*S2*S4)))<< 0<<endr;

    J_o_R<<0<< -S1<< C1*S2<< - C3*S1 - C1*C2*S3<< C1*C4*S2 - S4*(S1*S3 - C1*C2*C3)<< S5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) - C5*(C3*S1 + C1*C2*S3)<< C6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) + S6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))<< C6*(S4*(S1*S3 - C1*C2*C3) - C1*C4*S2) + S6*(C5*(C4*(S1*S3 - C1*C2*C3) + C1*S2*S4) + S5*(C3*S1 + C1*C2*S3))<<endr
     <<1<< 0<< C2<< S2*S3<< C2*C4 - C3*S2*S4<< S5*(C2*S4 + C3*C4*S2) + C5*S2*S3<< S6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) - C6*(C2*C4 - C3*S2*S4)<< S6*(C5*(C2*S4 + C3*C4*S2) - S2*S3*S5) - C6*(C2*C4 - C3*S2*S4)<<endr
     <<0<< -C1<< -S1*S2<< C2*S1*S3 - C1*C3<< - S4*(C1*S3 + C2*C3*S1) - C4*S1*S2<< S5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) - C5*(C1*C3 - C2*S1*S3)<< C6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) + S6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))<< C6*(S4*(C1*S3 + C2*C3*S1) + C4*S1*S2) + S6*(C5*(C4*(C1*S3 + C2*C3*S1) - S1*S2*S4) + S5*(C1*C3 - C2*S1*S3))<<endr;



    //-------RELATIVE KINEMATICS

    //calcSkew(fkin_or_right.t() * o_R_A.t() * (o_r_B - o_r_A), A_R_hat_BA);
    A_R_hat_BA.zeros();
    calcSkew(fkin_or_right.t() * o_R_A.t() * o_R_B * fkin_po_left, A_R_hat_T);
    calcSkew(fkin_or_right.t() * fkin_po_right, A_R_hat_G);

    J_11=-obj_R_G * (fkin_or_right.t() * J_p_R + A_R_hat_BA * fkin_or_right.t() * J_o_R + A_R_hat_T * fkin_or_right.t() * J_o_R - A_R_hat_G * fkin_or_right.t() * J_o_R);
    J_12= obj_R_G * fkin_or_right.t() *  o_R_A.t() * o_R_B * J_p_L;
    J_21=-obj_R_G * fkin_or_right.t() * J_o_R;
    J_22= obj_R_G * fkin_or_right.t() * o_R_A.t() * o_R_B * J_o_L;

    jacob_R_1=join_rows(J_11,J_12);
    jacob_R_2=join_rows(J_21,J_22);

    jacob_R=join_cols(jacob_R_1,jacob_R_2);

    obj_r_T=obj_R_G * fkin_or_right.t() * o_R_A.t() * (o_r_B - o_r_A  + o_R_B * fkin_po_left) - obj_R_G * fkin_or_right.t() * fkin_po_right + obj_r_G;
    obj_w_T=obj_R_G * fkin_or_right.t() *(o_R_A.t() * o_R_B * J_o_L *delta_q.rows(8,15)  -  J_o_R * delta_q.rows(0,7));

    pinv_jacob_right=pinv(jacob_right);
    pinv_jacob_left=pinv(jacob_left);
    pinv_jacob_R=pinv(jacob_R);

    //   START: PRIRORITY KINEMATICS
    Js_1=join_rows(jacob_right,zeros6b8);
    Js_2 = jacob_R;

     if (det(Js_1*Js_1.t())>0.00001)
         pinv_Js_1=pinv(Js_1);
     else
        pinv_Js_1=Js_1.t() * inv(Js_1*Js_1.t() + 0.001*I_6);


     Jhat_2=Js_2*(I_16- pinv_Js_1*Js_1);

     if (det(Jhat_2*Jhat_2.t())>0.00001)
        pinv_Jhat_2=pinv(Jhat_2);
     else
        pinv_Jhat_2=Jhat_2.t() * inv(Jhat_2*Jhat_2.t() + 0.001*I_6);
     //   END: PRIRORITY KINEMATICS


    //static double freq_Hz = 0.4;
    //double  trj = sin((2.0 * M_PI * freq_Hz * dt_ns)/1e9); // -1 .. 1

    //vec X_R(6), X_L(6); //current position
    //X_R << fkin_po_right(0)<<fkin_po_right(1)<<fkin_po_right(2)<<0.0<<0.0<<0.0;
    //X_L<<fkin_po_left(0)<<fkin_po_left(1)<<fkin_po_left(2)<<0.0<<0.0<<0.0;

    //Orientation
    //fkin_


/*
    if (count_loop_1==500)
    {
        //(round(1000*(obj_r_T.t()))).print();
        //(round((180/M_PI)*((join_cols(q_bar_r,q_bar_l)).t() - q_l.t()))).print();
        //(round(((q_l.t())))).print("q_l");
        //(round((180/M_PI)*((join_cols(q_bar_r,q_bar_l)).t()))).print("q_b");
        //(dXd_R.t()).print("dXd_R=");
        //(dXd_L.t()).print("dXd_L=");
        //cout << "dt_ns = " << dt_ns/1e9 << "\t"<< "traj_start = " << (double)(dt_ns-t_traj_start)/1000000000.0 << "\n";

        //---------------------------------
        count_loop_1=0;
    }
    ++count_loop_1;
*/

    //-**********************************************************************************
    // ****** Trajectory Generation
    //-**********************************************************************************
    u_int64_t dt_ns= 0;

    vec Xf_R(6), Xf_L(6);
    Xf_R.zeros(); Xf_L.zeros();

    if(flag_init_hands == false){
        //t_traj_start = get_time_ns();
        Xi_R<<fkin_po_right(0)<<fkin_po_right(1)<<fkin_po_right(2)<<0.0<<0.0<<0.0;
        Xi_L<<fkin_po_left(0)<<fkin_po_left(1)<<fkin_po_left(2)<<0.0<<0.0<<0.0;
        flag_init_hands = true;
        Xd_R = Xi_R;
        Xd_L = Xi_L;

        Xd_D_init<<obj_r_T(0)<<obj_r_T(1)<<obj_r_T(2)<<0.0<<0.0<<0.0;


    }

    if ( trj_flag == 1 ) {
        if ( g_tStart <= 0 ) {
            g_tStart = get_time_ns();
        }
        dt_ns = get_time_ns() - g_tStart;

        //Get valve position/orientation
        vec Xv(6);
        Xv<<0.25<<0.0<<-0.0<<0.0<<0.0<<0.0;

        //Get valve radius
        double Rv=0.1;

        //Calculate target position/orientations for two hands
        vec Xt_R(6), Xt_L(6);
        double Roff = 0.05;     //offset, 5cm
        Xt_R=Xv;    Xt_R(1)=Xv(1) - Rv-Roff;
        Xt_L=Xv;    Xt_L(1)=Xv(1) + Rv+Roff;

        //displacement: target - init
        //TEST:
        //Xf_R<<0.10<<0.0<<0.30<<0.0<<0.0<<0.0;
        //Xf_L<<0.10<<0.0<<0.30<<0.0<<0.0<<0.0;
        Xf_R = Xt_R - Xi_R;
        Xf_L = Xt_L - Xi_L;

       //Arms Trajectory
        line_traj( Xi_R, Xf_R , 10.0, (dt_ns/1e9), Xd_R, dXd_R);
        line_traj( Xi_L, Xf_L , 10.0, (dt_ns/1e9), Xd_L, dXd_L);
        //circle_traj( Xi_R, 60.0*M_PI/180.0 , 10.0, (dt_ns/1e9), Rv, Xd_R, dXd_R);

    }

///*
    if (count_loop_1==500)
    {
        cout<<(dt_ns/1e9)<<endl;
        //(Xd_R.t()).print("Xd_R=");
        //(Xd_L.t()).print("Xf_L=");
        //---------------------------------
        count_loop_1=0;
    }
    ++count_loop_1;
//*/

    if ( trj_flag == 1 ) {

        if ( close_hand_flag == 1 )
        {
                //Hand Control
                //right hand
                if ((q_ref(7) < 1.3) && (hand_flag_control_r==0))
                {
                    q_ref(7) =q_ref(7) + 0.0005;
                }
                else
                    hand_flag_control_r=1;


                if ((q_ref(7) > 0.26)&&(hand_flag_control_r==1))
                {
                    q_ref(7) =q_ref(7) - 0.0005;

                }
                else
                    hand_flag_control_r=0;


                //left hand
                if ((q_ref(15) < 1.3) && (hand_flag_control_l==0))
                {
                    q_ref(15) =q_ref(15) + 0.0005;
                }
                else
                    hand_flag_control_l=1;


                if ((q_ref(15) > 0.33)&&(hand_flag_control_l==1))
                {
                    q_ref(15) =q_ref(15) - 0.0005;

                }
                else
                    hand_flag_control_l=0;

                _pos[two_arms[7]-1]=100000.0 * q_ref(7);
                _pos[two_arms[15]-1]=100000.0 * q_ref(15);
        }

       if ( rotate_valve_flag == 0 )
       {
           // *************************************
           // IK resolution
           //   (O) CLIK
           //   (X) Singularity avoidance
           //   (?) Joint limit avoidance
           // *************************************
            K_inv   = 0.001;     //Tuned (0.02* 0.05)
            K_clik  = 1.5;       //Tuned
            K_null  = 0.001;     //not tuned


            X_R<<fkin_po_right(0)<<fkin_po_right(1)<<fkin_po_right(2)<<0.0<<0.0<<0.0;
            X_L<<fkin_po_left(0)<<fkin_po_left(1)<<fkin_po_left(2)<<0.0<<0.0<<0.0;
            X_D<<0.0<<0.0<<0.0<<0.0<<0.0<<0.0;

            //Right arm
            delta_q.rows(0,7)= K_inv* pinv_jacob_right* (dXd_R + K_clik*(Xd_R - X_R) ) + K_null * (I_8 - pinv_jacob_right*jacob_right) * (-lambda_dot_jntlmt_r);

            //Left arm
            delta_q.rows(8,15)= K_inv* pinv_jacob_left* (dXd_L + K_clik*(Xd_L - X_L) ) + K_null * (I_8 - pinv_jacob_left*jacob_left) * (-lambda_dot_jntlmt_l);

            X_D<<obj_r_T(0)<<obj_r_T(1)<<obj_r_T(2)<<0.0<<0.0<<0.0;
            Xd_D = Xd_D_init;
            dXd_D.zeros();

            //delta_q = K_inv * pinv_Js_1 * (dXd_R + K_clik * (Xd_R - X_R)) +  K_inv * pinv_Jhat_2 * (dXd_D + K_clik * (Xd_D-X_D) - Js_2 * pinv_Js_1 * (dXd_R + K_clik * (Xd_R - X_R)));


        }


        if ( rotate_valve_flag == 1 )
        {
            delta_q = K_inv * pinv_Js_1 * (dXd_R + K_clik * (Xd_R - X_R)) + pinv_Jhat_2 * (dXd_D + K_clik * (Xd_D-X_D) - Js_2 * pinv_Js_1 * (dXd_R + K_clik * (Xd_R - X_R)));

            //Relative between two-arms
            //delta_q= 0.2* pinv_jacob_R* C_vel- 0.00001 * (I_16 - pinv_jacob_R*jacob_R)*(-lambda_dot_jntlmt);
        }


        delta_q_sum = delta_q_sum + delta_q;
        delta_q_sum(7)=0.0;//exclude hand_right
        delta_q_sum(15)=0.0;//exclude hand_left

        for (int my_jnt_n =0; my_jnt_n<15; my_jnt_n++)
        {
            _pos[two_arms[my_jnt_n]-1] = _home[two_arms[my_jnt_n]-1] + 100000.0 * delta_q_sum(my_jnt_n);
        }

    // *********************************START: SAFETY************************************************************************
        safety_flag=0;
        for (int my_jnt_n =0; my_jnt_n<14; my_jnt_n++)
        {
           if ((_pos[two_arms_nohands[my_jnt_n]-1] - _ts_bc_data[two_arms_nohands[my_jnt_n]-1].raw_bc_data.mc_bc_data.Position)  > 100000.0 * 5 * M_PI/180.0)
                   safety_flag=safety_flag+1;
        }


        jntlmt_flag=0;
           /*if ((abs(_pos[two_arms[0]-1])  > 100000.0 * 190 * M_PI/180.0)||((abs(_pos[two_arms[0 + 8]-1])  > 100000.0 * 190 * M_PI/180.0))||(abs(_pos[two_arms[0]-1])  < 100000.0 * 100 * M_PI/180.0)||((abs(_pos[two_arms[0 + 8]-1])  > 100000.0 * 100 * M_PI/180.0)))
                   jntlmt_flag=jntlmt_flag+1;

           if ((abs(_pos[two_arms[1]-1])  > 100000.0 * 87 * M_PI/180.0)||((abs(_pos[two_arms[1 + 8]-1])  > 100000.0 * 87 * M_PI/180.0)))
                   jntlmt_flag=jntlmt_flag+1;

           if ((abs(_pos[two_arms[2]-1])  > 100000.0 * 87 * M_PI/180.0)||((abs(_pos[two_arms[2 + 8]-1])  > 100000.0 * 87 * M_PI/180.0)))
                   jntlmt_flag=jntlmt_flag+1;

           if ((abs(_pos[two_arms[3]-1])  > 100000.0 * 87 * M_PI/180.0)||((abs(_pos[two_arms[3 + 8]-1])  > 100000.0 * 87 * M_PI/180.0)))
                   jntlmt_flag=jntlmt_flag+1;

           if ((abs(_pos[two_arms[4]-1])  > 100000.0 * 85 * M_PI/180.0)||((abs(_pos[two_arms[4 + 8]-1])  > 100000.0 * 85 * M_PI/180.0)))
                   jntlmt_flag=jntlmt_flag+1;

           if ((abs(_pos[two_arms[5]-1])  > 100000.0 * 25 * M_PI/180.0)||((abs(_pos[two_arms[5 + 8]-1])  > 100000.0 * 25 * M_PI/180.0)))
                   jntlmt_flag=jntlmt_flag+1;

           if ((_pos[two_arms[6]-1]  > 100000.0 * 40 * M_PI/180.0)||((_pos[two_arms[6 + 8]-1]  > 100000.0 * 40 * M_PI/180.0)) || (_pos[two_arms[6]-1]  < 100000.0 * -75 * M_PI/180.0)||((_pos[two_arms[6 + 8]-1]  < 100000.0 * -75 * M_PI/180.0)))
                   jntlmt_flag=jntlmt_flag+1;*/

           // Hands
           if ((_pos[two_arms[7]-1]  > 100000.0 * 1.4)||((_pos[two_arms[7 + 8]-1]  > 100000.0 * 1.4)))
                   jntlmt_flag=jntlmt_flag+1;

           if ((_pos[two_arms[7]-1]  < 100000.0 * 0.25)||((_pos[two_arms[7 + 8]-1]  < 100000.0 * 0.25)))
                   jntlmt_flag=jntlmt_flag+1;

    // -*********************************END: SAFETY************************************************************************
           safety_flag=0;
           //jntlmt_flag=0;

           if((safety_flag==0)&(jntlmt_flag==0))
             {
                   move(MV_POS|MV_VEL|MV_TOR|MV_STF);
             }
         else
            {
               if(safety_flag!=0)
                  DPRINTF("Very High Joint Velocity Detected--Control Stopped... \n");
               if(jntlmt_flag!=0)
                  DPRINTF("Exceeding Joint Limits -- Control Stopped... \n");
            }


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
            /*
            DPRINTF("Start control ...r_arm\n");
            start_stop_set_control(r_arm,true);
            DPRINTF("Start control ...l_leg\n");
            start_stop_set_control(l_leg,true);*/
            DPRINTF("Start control ...two_arms\n");
            start_stop_set_control(two_arms,true);
            break;
        case '1':
            //DPRINTF("Start control single \n");
            //start_stop_single_control(8,true);
            DPRINTF("Start control single \n");
            start_stop_single_control(15,true);
            break;
        case '2':
            DPRINTF("Start control ...l_arm\n");
            start_stop_set_control(l_arm,true);
            DPRINTF("Start control ...r_arm\n");
            start_stop_set_control(r_arm,true);
            break;
        case '3':
            DPRINTF("Start control ...l_leg\n");
            start_stop_set_control(l_leg,true);
            DPRINTF("Start control ...r_leg\n");
            start_stop_set_control(r_leg,true);
            break;
        case 'h':
            DPRINTF("Set home pos\n");
            homing(homePos, homeVel);
            //test();
            break;
        case 'A':
            DPRINTF("Set pos ref to median point of range pos\n");
            for ( auto it = _mcs.begin(); it != _mcs.end(); it++ ) {
                b = it->second;
                _pos[b->bId-1] = (b->_max_pos + b->_min_pos) / 2;
            }
            move();
            break;
        case 'a':
            DPRINTF("Do something else\n");
            toggle *= -1;
            for ( auto it = _mcs.begin(); it != _mcs.end(); it++ ) {
                b = it->second;
                _pos[b->bId-1] = _home[b->bId-1] + DEG2mRAD(5) * toggle;
            }
            move();
            break;
        case '[':
            DPRINTF("Do something ++++ \n");
            for ( auto it = _mcs.begin(); it != _mcs.end(); it++ ) {
                b = it->second;
                _pos[b->bId-1] = _ts_bc_data[b->bId-1].raw_bc_data.mc_bc_data.Position + DEG2mRAD(0.5);
            }
            move();
            break;
        case ']':
            DPRINTF("Do something ----\n");
            for ( auto it = _mcs.begin(); it != _mcs.end(); it++ ) {
                b = it->second;
                _pos[b->bId-1] = _ts_bc_data[b->bId-1].raw_bc_data.mc_bc_data.Position - DEG2mRAD(0.5);
            }
            move();
            break;

        case 't':

            DPRINTF("trajectory\n");
            homing();
            //g_tStart = get_time_ns();
            trj_flag = ! trj_flag;
            break;

        case 'c':

            DPRINTF("closing hands\n");
            close_hand_flag = ! close_hand_flag;
            break;

        case 'r':

            DPRINTF("rotating hands\n");
            rotate_valve_flag = ! rotate_valve_flag;
            break;

        case 'X':
            pos_group.clear();
            pos_group[88] = 0x00DEAD00;
            pos_group[99] = 0x11BEEF11;
            set_position_group(pos_group);
            pos_vel_group.clear();
            pos_vel_group[88] = std::make_pair(0x00DEAD00, 0xCACA);
            pos_vel_group[99] = std::make_pair(0x11BEEF11, 0x7777);
            set_position_velocity_group(pos_vel_group);
            break;

        case 'x':
            pos[88] = 0x00DEAD00;
            pos[99] = 0x11BEEF11;
            set_position_group(bIds,pos,2);
            vel[88] = 0xCACA;
            vel[99] = 0x7777;
            set_position_velocity_group(bIds,pos,vel,2);
            break;

        case 'j':
            pos[88] = 0x00DEAD00;
            pos[99] = 0x11BEEF11;
            set_gravity_compensation(pos,sizeof(pos));
            break;

        case 'P':
            _mcs[19]->set_PID_increment(POSITION_GAINS, 1000, 0, 0);
            break;
        case 'p':
            _mcs[19]->set_PID_increment(POSITION_GAINS, -1000, 0, 0);
            break;
        case 'I':
            _mcs[19]->set_PID_increment(POSITION_GAINS, 0, 1, 0);
            break;
        case 'i':
            _mcs[19]->set_PID_increment(POSITION_GAINS, 0, -1, 0);
            break;
        case 'D':
            _mcs[19]->set_PID_increment(POSITION_GAINS, 0, 0, 50);
            break;
        case 'd':
            _mcs[19]->set_PID_increment(POSITION_GAINS, 0, 0, -50);
            break;


        case 'U':
            // works !!! need to handle dsp reboot ... exit app 
            _mcs[1]->setItem(CMD_UPGRADE, 0, 0);
            break;


        case '@':
            throw(boards_warn(std::string("Hi this is a boards warning")));
            // ... never break .....
            //break;

        case '#':
            throw(boards_error(std::string("Hi this is an boards error")));
            // ... never break .....
            //break;

        case '!':
            throw(std::runtime_error(std::string("Hi this is a not handled except")));
            // ... never break .....
            //break;

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

