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

/** NOTE this are board ids, not array positions */
std::vector<int> r_arm = { 16, 17, 18 ,19};
std::vector<int> l_arm = { 20, 21, 22, 23};
std::vector<int> r_farm = { 26, 27, 28, 32};
std::vector<int> l_farm = { 29, 30, 31, 33};

static const std::vector<float> homeVel(33,25);


// home position in degree
static const std::vector<float> homePos = {
    // lower body #15
//  0, -1,  0,  0,  0,  0,   0,   0,  0,  0,  0,   0,   0,  0,  0,
    0,  0,  0,  0,  0,  -2,  0,  0,  0,   0,   2,   0,   0,  0,  0,
//  1,  2,  3,  4,  5,   6,  7,  8,  9,   10, 11,  12,  13, 14, 15
    // upper body #10 right arm to left arm, last 2 are right and left neck
   0,   60,  0,  -80,  0,  -60,  0, -80,  0,  0,
// 16,  17, 18,  19,   20,  21,  22,  23, 24, 25
   0,  0,  0,
// 26, 27, 28
    0,  0,  0,
// 29, 30, 31
    0,  0};
// 32, 33

class robolli_legacy {


public:
    robolli_legacy();
    ~robolli_legacy();

    void init(walkman::drc::valve::yarp_interface& iYarp);

    /* updates _home = current_position, tells control to go to current position */
    void homing();
    /* position control to desired position */
    void homing(const std::vector<float> &pos, const std::vector<float> &vel);
    unsigned long int get_time_ns();

    static void getRobolliLeftArm(int* robolliVec,
                                  yarp::sig::Vector& left_arm_q,
                                  const unsigned int scale);
    static void getRobolliRightArm(int* robolliVec,
                                   yarp::sig::Vector& right_arm_q,
                                   const unsigned int scale);
    static void setRobolliLeftArm(int* robolliVec,
                                  yarp::sig::Vector& left_arm_q,
                                  const unsigned int scale);
    static void setRobolliRightArm(int* robolliVec,
                                   yarp::sig::Vector& right_arm_q,
                                   const unsigned int scale);

    int     _home[MAX_MC_BOARDS];
    int     _pos[MAX_MC_BOARDS];

    private:
        unsigned int l_arm_size;
        unsigned int l_farm_size;
        unsigned int r_arm_size;
        unsigned int r_farm_size;

        yarp_interface _iYarp;
};

}
}
}

#endif
