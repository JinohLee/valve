#ifndef __VALVEMODULE_H__
#define __VALVEMODULE_H__

#include <ManipulationVars.h>
#include <Boards_ctrl_ext.h>

class Boards_ctrl_basic;

class ValveModule {
    int count_loop_1;

    bool trj_flag;
    bool reach_flag;
    bool push_flag;
    bool open_hand_flag;
    bool moving_far_flag;
    bool valve_rotate_flag;

    unsigned long int t_traj_start;
    long int g_tStart;

    Boards_ctrl_basic* _robolli;

    bool reset();

    long int getTimeNsRobolli() { return get_time_ns(); }

    void homingRobolli();
    void homingRobolli(const std::vector<float> &pos, const std::vector<float> &vel);
public:

    ManipulationVars mVars;
    bool close_hand_flag;

    ValveModule();
    void init(Boards_ctrl_basic *robolli);

    void controlLaw();

    void updateFromRobolli(ts_bc_data_t _ts_bc_data[MAX_MC_BOARDS]);
    bool updateToRobolli(int _pos[MAX_MC_BOARDS], int _home[MAX_MC_BOARDS]);

    bool rotateValve();
    bool closeHands();
    bool moveFarFromValve();
    bool doTrajectory();
    bool doReaching();
    bool doOpenHand();
    bool doPushing();
};

#endif
