#ifndef __ROBOLLI_LEGACY__
#define __ROBOLLI_LEGACY__

#include <vector>
#define MAX_MC_BOARDS   64

class robolli_legacy {

public:
    robolli_legacy();
    ~robolli_legacy();

    void init(void);

    /* updates _home = current_position, tells control to go to current position */
    void homing();
    /* position control to desired position */
    void homing(const std::vector<float> &pos, const std::vector<float> &vel);
    unsigned long int get_time_ns();

//    static void getRobolliLeftArm(int* robolliVec, yarp::sig::Vector& left_arm_q);
//    static void getRobolliRightArm(int* robolliVec, yarp::sig::Vector& right_arm_q);
//    static void getRobolliLeftLeg(int* robolliVec, yarp::sig::Vector& left_leg_q);
//    static void getRobolliRightLeg(int* robolliVec, yarp::sig::Vector& right_leg_q);
//    static void getRobolliTorso(int* robolliVec, yarp::sig::Vector& right_leg_q);


private:


    int     _home[MAX_MC_BOARDS];
    int     _pos[MAX_MC_BOARDS];
    short   _vel[MAX_MC_BOARDS];
    short   _tor[MAX_MC_BOARDS];
    int     _stiff[MAX_MC_BOARDS];
    int     _damp[MAX_MC_BOARDS];
    short   _mVolt[MAX_MC_BOARDS];
};

#endif
