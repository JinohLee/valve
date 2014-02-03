#include <robolli_legacy.h>
#include <vector>
#include <iostream>

using namespace walkman::drc::valve;

robolli_legacy::robolli_legacy() {
    _iYarp = NULL;
    l_arm_size = sizeof(l_arm)/sizeof(l_arm[0]);
    r_arm_size = sizeof(r_arm)/sizeof(r_arm[0]);
    l_farm_size = sizeof(l_farm)/sizeof(l_farm[0]);
    r_farm_size = sizeof(r_farm)/sizeof(r_farm[0]);
}


robolli_legacy::~robolli_legacy() {
    _iYarp = NULL;
}

void robolli_legacy::init(walkman::drc::valve::yarp_interface& iYarp) {
    _iYarp = &iYarp;
}

/**
 * use actual joints position and set as 'home position' and 
 * move towards it 
 * 
 */
void robolli_legacy::homing() {
    int         dummy, bId = 0;
    std::vector<float> pos(MAX_MC_BOARDS);
    std::vector<float> vel(MAX_MC_BOARDS);
/* original code here */
//    get_bc_data(_ts_bc_data);

//    for (auto it = _mcs.begin(); it != _mcs.end(); it++) {
//        bId = it->first;
//        pos[bId-1] = mRAD2DEG(_ts_bc_data[bId-1].raw_bc_data.mc_bc_data.Position);
//        vel[bId-1] = 25;
//        it->second->get_PID(POSITION_GAINS, _stiff[bId-1], dummy, _damp[bId-1]);
//        //std::cout << pos[bId-1] << std::endl;
//    }
        
//    homing(pos, vel);
}

/**
 * set 'home position' and move towards it
 *  
 * @param pos_deg 
 * @param vel_deg_s 
 */ 
void robolli_legacy::homing(const std::vector<float> &pos_deg, const std::vector<float> &vel_deg_s) {

    int bId = 0;
//    if ( pos_deg.size() > MAX_MC_BOARDS) {
//        throw(std::runtime_error(std::string("wrong size pos_deg")));
//    }
//    if ( vel_deg_s.size() > MAX_MC_BOARDS) {
//        throw(std::runtime_error(std::string("wrong size vel_deg_s")));
//    }

//    for (auto it = _mcs.begin(); it != _mcs.end(); it++) {
//        bId = it->first;
//        _pos[bId-1]  = DEG2mRAD(pos_deg[bId-1]);
//        _home[bId-1] = DEG2mRAD(pos_deg[bId-1]);
//        _vel[bId-1]  = DEG2RAD(vel_deg_s[bId-1])*1000;
//    }

//    move();
}

unsigned long int robolli_legacy::get_time_ns() {
    return yarp::os::Time::now()*1E9;
}

void robolli_legacy::getRobolliLeftArm(int* robolliVec, yarp::sig::Vector& left_arm_q, const unsigned int scale) {
    if(left_arm_q.size() != l_arm_size+l_farm_size)
        left_arm_q.resize(l_arm_size+l_farm_size);
    for(unsigned int i = 0; i < l_arm_size; ++i)
        left_arm_q[i] = robolliVec[l_arm[i]]/scale;
    for(unsigned int i = 0; i < l_farm_size; ++i)
        left_arm_q[i+l_arm_size] = robolliVec[l_farm[i]]/scale;
}

void robolli_legacy::getRobolliRightArm(int* robolliVec, yarp::sig::Vector& right_arm_q, const unsigned int scale) {
    if(right_arm_q.size() != r_arm_size+r_farm_size)
        right_arm_q.resize(r_arm_size+r_farm_size);
    for(unsigned int i = 0; i < r_arm_size; ++i)
        right_arm_q[i] = robolliVec[r_arm[i]]/scale;
    for(unsigned int i = 0; i < r_farm_size; ++i)
        right_arm_q[i+r_arm_size] = robolliVec[r_farm[i]]/scale;
}

void robolli_legacy::setRobolliLeftArm(int* robolliVec, yarp::sig::Vector& left_arm_q, const unsigned int scale) {
    if(left_arm_q.size() != l_arm_size+l_farm_size) {
        std::cout << "Error copying from yarp to robolli" << std::endl;
        return;
    }
    for(unsigned int i = 0; i < l_arm_size; ++i)
        robolliVec[l_arm[i]] = left_arm_q[i]*scale;
    for(unsigned int i = 0; i < l_farm_size; ++i)
        robolliVec[l_farm[i]] = left_arm_q[i+l_arm_size]*scale;
}

void robolli_legacy::setRobolliRightArm(int* robolliVec, yarp::sig::Vector& right_arm_q, const unsigned int scale) {
    if(right_arm_q.size() != r_arm_size+r_farm_size) {
        std::cout << "Error copying from yarp to robolli" << std::endl;
        return;
    }
    for(unsigned int i = 0; i < r_arm_size; ++i)
        robolliVec[r_arm[i]] = right_arm_q[i]*scale;
    for(unsigned int i = 0; i < r_farm_size; ++i)
        robolliVec[r_farm[i]] = right_arm_q[i+r_arm_size]*scale;
}
