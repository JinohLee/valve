#include <ValveModule.h>
#include <boost/numeric/ublas/io.hpp>
#include "Boards_ctrl_basic.h"

using namespace boost::numeric::ublas;
using namespace arma;

std::vector<int> two_arms = {16, 17, 18 ,19, 26, 27, 28, 32,
                             20, 21, 22, 23, 29, 30, 31, 33};

ValveModule::ValveModule() {
    count_loop_1=0;
    _robolli_legacy = NULL;
}

void ValveModule::init(robolli_legacy* robolli) {
    _robolli_legacy = robolli;

    homingRobolli();

    g_tStart = 0;
    trj_flag = 0;
    reach_flag = 0;
    push_flag=0;
    open_hand_flag=0;
    close_hand_flag = 0;

    t_traj_start= 0;
    valve_rotate_flag = 0;
    moving_far_flag=0;

    mVars.init();
}

void ValveModule::controlLaw() {
    /** ALWAYS call this after a sense(), so that q_l is meaningful */
    if(mVars.isManipInit() == false)
        mVars.init_manip(mVars.q_l);

    mVars.manip_kine(); //Calculate kinematics

    hand_pos hand_delta_q; //Grasping/Openning requirements
    hand_delta_q.l=0;
    hand_delta_q.r=0;


    if (count_loop_1==500)
    {
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


        //cout<<delta_q_sum(7)<<endl;

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
                    g_tStart = getTimeNsRobolli();
                }
                dt_ns = getTimeNsRobolli() - g_tStart;
                    mVars.reaching(dt_ns);
                }
            //Reaching--------------------------------------------------------//


            //GRASPING  ------------------------------------------------------//
                if ( close_hand_flag == 1 )
                    hand_delta_q = mVars.grasping();
            //GRASPING  -----------------------------------------------------//

            //OPENNING ------------------------------------------------------//
                if ( open_hand_flag == 1 )
                    hand_delta_q = mVars.openning();
            //OPENNING ------------------------------------------------------//


            //ROTATING-------------------------------------------------------//
                if ( valve_rotate_flag == 1 )
                {
                    u_int64_t dt_ns= 0;
                    if ( g_tStart <= 0 ) {
                        g_tStart = getTimeNsRobolli();
                    }
                    dt_ns = getTimeNsRobolli() - g_tStart;
                    mVars.rotating(dt_ns);
                }
            //ROTATING---------------------------------------------------------//

            //PUSHING  --------------------------------------------------------//
                if ( push_flag == 1 )
                {
                    u_int64_t dt_ns= 0;
                    if ( g_tStart <= 0 ) {
                        g_tStart = getTimeNsRobolli();
                    }
                    dt_ns = getTimeNsRobolli() - g_tStart;
                        mVars.pushing(dt_ns);
                }
            //PUSHING ----------------------------------------------------------//

            //MOVING FAR--------------------------------------------------------//
                if ( moving_far_flag == 1 )
                { u_int64_t dt_ns= 0;
                    if ( g_tStart <= 0 ) {
                        g_tStart = getTimeNsRobolli();
                    }
                    dt_ns = getTimeNsRobolli() - g_tStart;
                        mVars.movingfar(dt_ns);
                }
            //MOVING FAR--------------------------------------------------------//


            //CALCULATE JOINT REFERENCES----------------------------------------//
                mVars.delta_q_sum = mVars.delta_q_sum + mVars.delta_q;
                mVars.delta_q_sum(7)=hand_delta_q.r;//exclude hand_right
                mVars.delta_q_sum(15)=hand_delta_q.l;//exclude hand_left

            //CALCULATE JOINT REFERENCES---------------------------------------//
      }
}

void ValveModule::homingRobolli() {
    _robolli_legacy->homing();
}

void ValveModule::homingRobolli(const std::vector<float> &pos, const std::vector<float> &vel) {
    _robolli_legacy->homing(pos,vel);
}

void ValveModule::updateFromRobolli(int* _pos, int* _vel, int* _torque) {
//    for (int i=0; i<16; i++)
//     {
//         mVars.q_l(i)=double(_ts_bc_data[two_arms[i]-1].raw_bc_data.mc_bc_data.Position/100000.0);
//         mVars.q_dot(i)=double(_ts_bc_data[two_arms[i]-1].raw_bc_data.mc_bc_data.Velocity/1000.0);

//         if (i<14) mVars.tau(i)=double(_ts_bc_data[two_arms[i]-1].raw_bc_data.mc_bc_data.Torque/1000.0);
//     }
}

bool ValveModule::updateToRobolli(int _pos[MAX_MC_BOARDS], int _home[MAX_MC_BOARDS]) {
//    if(trj_flag == 1) {
//        for (int my_jnt_n =0; my_jnt_n<15; my_jnt_n++)
//        {
//            _pos[two_arms[my_jnt_n]-1] = _home[two_arms[my_jnt_n]-1] + 100000.0 * mVars.delta_q_sum(my_jnt_n);
//        }
//        _pos[two_arms[7]-1] = 100000.0 * mVars.delta_q_sum(7);
//        _pos[two_arms[15]-1] = 100000.0 * mVars.delta_q_sum(15);

//        //Safety for joint limits
//        if(mVars.isSafe()) {
//            return true;
//        } else {
//            DPRINTF("Exceeding Joint Limits/Speed -- Control Stopped... \n");
//            return false;
//        }
//    } return false;
    return false;
}

bool ValveModule::rotateValve() {
    reset();
    valve_rotate_flag = ! valve_rotate_flag;
    return valve_rotate_flag;
}

bool ValveModule::closeHands() {
    /*** TODO should we call reset() here?!? */
    close_hand_flag = ! close_hand_flag;
    return close_hand_flag;
}

bool ValveModule::moveFarFromValve() {
    reset();
    moving_far_flag = ! moving_far_flag;
    return moving_far_flag;
}

bool ValveModule::reset() {
    g_tStart = -1;
    cout<<"reset() is called!="<<g_tStart<<endl;
    return true;
}

bool ValveModule::doTrajectory() {
    reset();
    homingRobolli();
    //g_tStart = getTimeNs();
    trj_flag = ! trj_flag;
    return trj_flag;
}

bool ValveModule::doReaching() {
    reset();
    reach_flag = ! reach_flag;
    return reach_flag;
}

bool ValveModule::doPushing() {
    reset();
    push_flag = ! push_flag;
    return push_flag;
}

bool ValveModule::doOpenHand() {
    open_hand_flag = ! open_hand_flag;
    return open_hand_flag;
}

