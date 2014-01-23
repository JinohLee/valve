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

#include <Boards_ctrl_ext.h>
#include <boost/circular_buffer.hpp>

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

/**
 * @class Boards_ctrl_basic
 * @defgroup Boards_controller Boards_controller
 * @ingroup RoboLLI
 * @brief Boards_ctrl_basic class
 */

class Boards_ctrl_basic : public Thread_hook, public Boards_ctrl_ext  {

private:
    uint64_t g_tStart;
    uint8_t trj_flag;
    uint8_t close_hand_flag;
    uint8_t rotate_valve_flag;

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

    virtual void th_init(void *);
    virtual void th_loop(void *);

    virtual int user_input(void *buffer, ssize_t buff_size);
    virtual uint8_t parse_console(void * buffer, ssize_t buff_size, void * parsed = 0);
    int user_loop(void);

};


#endif
