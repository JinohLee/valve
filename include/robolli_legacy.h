#ifndef __ROBOLLI_LEGACY__
#define __ROBOLLI_LEGACY__

#include <vector>
#include <yarp_interface.h>
#define MAX_MC_BOARDS   64
#define POSITION_SCALE 1E5
#define VELOCITY_SCALE 1E3
#define TORQUE_SCALE 1E3

namespace walkman
{
namespace drc
{
namespace valve
{

class robolli_legacy {


public:
    robolli_legacy(walkman::drc::valve::yarp_interface& iYarp);
    ~robolli_legacy();

    void init();

    /* updates _home = current_position, tells control to go to current position */
    void homing();

    unsigned long int get_time_ns();

    void updateFromYarp(const robot_state_input& inputs);
    void updateToYarp(robot_joints_output& outputs);

    /* COMMANDS for the move() */
    int     _home[MAX_MC_BOARDS];
    int     _pos[MAX_MC_BOARDS];

    /* READINGS after sense() */
    int     _mc_bc_data_Position[MAX_MC_BOARDS];
    int     _mc_bc_data_Velocity[MAX_MC_BOARDS];
    int     _mc_bc_data_Torque[MAX_MC_BOARDS];

    private:
        unsigned int l_arm_size;
        unsigned int l_farm_size;
        unsigned int r_arm_size;
        unsigned int r_farm_size;

        void getRobolliLeftArm(const int* robolliVec,
                               double* left_arm_data,
                               const unsigned int scale);
        void getRobolliRightArm(const int* robolliVec,
                                double* right_arm_data,
                                const unsigned int scale);
        void setRobolliLeftArm(int* robolliVec,
                               const double* left_arm_data,
                               const unsigned int scale);
        void setRobolliRightArm(int* robolliVec,
                                const double* right_arm_data,
                                const unsigned int scale);

        yarp_interface& _iYarp;
};

}
}
}

#endif
