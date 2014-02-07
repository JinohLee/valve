/*
   Boards_ctrl_basic.h

   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2013-, alessio.margan@iit.it)
   
*/

/**
 *
 * @author Alessio Margan (2013-, alessio.margan@iit.it)
*/

#ifndef __BOARDS_CTRL_BASIC_H__
#define __BOARDS_CTRL_BASIC_H__

#include <utils.h>
#include <thread_util.h>
#include <armadillo>
#include <Boards_ctrl_ext.h>
#include <boost/circular_buffer.hpp>

using namespace arma;

typedef struct{
    uint64_t ts;
    double des_pos;
    double qm;

    double rel_x;
    double rel_y;
    double rel_z;

    double CartXd_R;
    double CartYd_R;
    double CartZd_R;

    double CartXd_L;
    double CartYd_L;
    double CartZd_L;

    double CartX_R;
    double CartY_R;
    double CartZ_R;

    double CartX_L;
    double CartY_L;
    double CartZ_L;

} log_user_t;

struct hand_pos{
 double l;
 double r;
};

//For kinematics
class ManipulationVars {

protected:

    vec valve_data;


public:

    ManipulationVars();
    void manip_kine();
    void reaching(u_int64_t dt_ns);
    void pushing(u_int64_t dt_ns);
    void movingfar(u_int64_t dt_ns);
    hand_pos grasping();
    hand_pos openning();
    bool testsafety();
    void rotating(u_int64_t dt_ns);
    void init_manip(vec _q_l);

    const vec& get_valve_data();
    inline double get_radius();
    void set_valve_data(vec valve_data);
    mat jacob_right, jacob_left, jacob_R, jacob_R_1, jacob_R_2, zeros6b8,jacob_R_resized;
    mat fkin_or_left, fkin_or_right, fkin_po_left, fkin_po_right;
    mat  I_16, I_14, I_8, I_6, obj_R_G, o_R_A, o_R_B;
    mat o_base, o_r_A, o_r_B, obj_r_G;
    mat A_R_hat_BA, A_R_hat_T, A_R_hat_G, J_11, J_12, J_21, J_22;
    mat J_p_L, J_p_R, J_o_L, J_o_R;
    mat pinv_jacob_right, pinv_jacob_left, pinv_jacob_R, Js_1, Js_2, pinv_Js_1, Jhat_2, pinv_Jhat_2;
    vec Theta_0_R, Theta_0_L, q_l, q_l_resized, q_dot, tau, C_vel, q_ref, q_h, delta_q, delta_q_sum, null_vel, obj_r_T, obj_w_T, joint_error_ar;
    vec lambda_dot_jntlmt_r,lambda_dot_jntlmt_l, lambda_dot_jntlmt, q_max_r, q_min_r, q_bar_r, q_max_l, q_min_l, q_bar_l;
    vec Qr, Ql, Qi_r, Qi_l, Qd_r, Qd_l,orient_e_R;
    vec Eo_r, Eo_l, z6;
    double K_inv;     //Tuned (0.02* 0.05)
    double K_clik;       //Tuned
    double K_null;     //not tuned
    double L0, L1, L2, a3, Le;
    double S1,S2,S3,S4,S5,S6,S7,S8,S9,S10,S11,S12,S13,S14,S15,S16;
    double C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14,C15,C16;

    //for trajectory
    vec Xi_R, Xi_L;
    vec dXd_D, Xd_D;
    vec Xd_R, Xd_L;
    vec dXd_R, dXd_L;
    vec X_R, X_L, X_D;
    vec Xd_D_init;

    //flags
    bool flag_run_once;
    int safety_flag, jntlmt_flag;
    int count_loop_1, hand_flag_control_r, hand_flag_control_l;
    bool flag_init_hands, init_rot_po, flag_init_pushing, flag_init_movingfar;

    bool trj_flag;
    bool reach_flag;
    bool push_flag;
    bool open_hand_flag;
    bool moving_far_flag;
    bool valve_rotate_flag;



};




/**
 * @class Boards_ctrl_basic
 * @defgroup Boards_controller Boards_controller
 * @ingroup RoboLLI
 * @brief Boards_ctrl_basic class
 */

class Boards_ctrl_basic : public ManipulationVars, public Thread_hook, public Boards_ctrl_ext  {

private:

    int64_t g_tStart;
    /*uint8_t trj_flag;
    uint8_t reach_flag;
    uint8_t push_flag;
    uint8_t open_hand_flag;
    uint8_t moving_far_flag;
    //uint8_t close_hand_flag;
    uint8_t valve_rotate_flag;*/



    uint64_t t_traj_start;

    boost::circular_buffer<log_user_t> log_user_buff[16];
    boost::circular_buffer<log_user_t> log_user_buff_Cart;

    group_ref_t pos_group;
    group_ref_comp_t pos_vel_group;

    double control_old, vel_old;

    Write_XDDP_pipe * xddp_test;


public:
    Boards_ctrl_basic(const char * config);
    virtual ~Boards_ctrl_basic();
    void reset();
    bool close_hand_flag;

    virtual void th_init(void *);
    virtual void th_loop(void *);

    virtual int user_input(void *buffer, ssize_t buff_size);
    virtual uint8_t parse_console(void * buffer, ssize_t buff_size, void * parsed = 0);
    int user_loop(void);

};


#endif
