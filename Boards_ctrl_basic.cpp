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


#include "Boards_ctrl_basic.h"
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



int Boards_ctrl_basic::user_loop(void) {

    int bId;

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

