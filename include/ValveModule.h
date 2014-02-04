#ifndef __VALVEMODULE_H__
#define __VALVEMODULE_H__

#include <ManipulationVars.h>
#include <yarp/os/all.h>
#include <robolli_legacy.h>

namespace walkman {
namespace drc {
namespace valve {

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

    bool reset();

    long int getTimeNsRobolli() { return _robolli_legacy->get_time_ns(); }

    void homingRobolli();
    void homingRobolli(const std::vector<float> &pos, const std::vector<float> &vel);
public:

    robolli_legacy* _robolli_legacy;

    ManipulationVars mVars;
    bool close_hand_flag;

    ValveModule();
    void init(robolli_legacy *robolli);

    void controlLaw();

    void updateFromRobolli(int _mc_bc_data_Position[MAX_MC_BOARDS],
                           int _mc_bc_data_Velocity[MAX_MC_BOARDS],
                           int _mc_bc_data_Torque[MAX_MC_BOARDS]);

    bool updateToRobolli(int _pos[MAX_MC_BOARDS],
                         int _home[MAX_MC_BOARDS]);

    bool rotateValve();
    bool closeHands();
    bool moveFarFromValve();
    bool doTrajectory();
    bool doReaching();
    bool doOpenHand();
    bool doPushing();
};

}
}
}

#endif
