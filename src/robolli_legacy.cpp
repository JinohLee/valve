#include <robolli_legacy.h>
#include <vector>
#include <iostream>
#include <iCub/ctrl/math.h>

using namespace walkman::drc::valve;

/** NOTE this are board ids, not array positions */
int r_arm[] =       { 16, 17, 18 ,19};
int l_arm[] =       { 20, 21, 22, 23};
int r_arm_offs[] =  { 90,  0,  0,  0};  // offset in y2r(x)
int l_arm_offs[] =  {-90,  0,  0,  0};  // offset in y2r(x)
int no_offs[] =     {  0,  0,  0,  0};
int r_farm[] = { 26, 27, 28, 32};
int l_farm[] = { 29, 30, 31, 33};

const float legacy_homeVel[] = {
    25, 25, 25, 25, 25,  25, 25, 25, 25,  25, 25,  25,  25, 25, 25,
//  1,   2,  3,  4,  5,   6,  7,  8,  9,  10, 11,  12,  13, 14, 15
// upper body #10 right arm to left arm, last 2 are right and left neck
   25,  25, 25, 25, 25, 25, 25, 25, 25, 25,
// 16,  17, 18, 19, 20, 21, 22, 23, 24, 25
   25, 25, 25,
// 26, 27, 28
   25, 25, 25,
// 29, 30, 31
   25, 25};
// 32, 33

// home position in degree
const float legacy_homePos[] = {
// lower body #15
    0,  0,  0,  0,  0,  -2,  0,  0,  0,   0,   2,   0,   0,  0,  0,
//  1,  2,  3,  4,  5,   6,  7,  8,  9,   10, 11,  12,  13, 14, 15
    // upper body #10 right arm to left arm, last 2 are right and left neck
    0,  60,  0, -80,    0, -60,   0, -80,  0,  0,
// 16,  17, 18,  19,   20,  21,  22,  23, 24, 25
    0,  0,  0,
// 26, 27, 28
    0,  0,  0,
// 29, 30, 31
    0,  0};
// 32, 33

robolli_legacy::robolli_legacy(yarp_interface& iYarp) : _iYarp(iYarp){
    l_arm_size = sizeof(l_arm)/sizeof(l_arm[0]);
    r_arm_size = sizeof(r_arm)/sizeof(r_arm[0]);
    l_farm_size = sizeof(l_farm)/sizeof(l_farm[0]);
    r_farm_size = sizeof(r_farm)/sizeof(r_farm[0]);

    for(unsigned int i = 0; i < sizeof(legacy_homePos)/sizeof(legacy_homePos[0]); ++i) {
        _mc_bc_data_Position[i] = 0;
        _mc_bc_data_Velocity[i] = 0;
        _mc_bc_data_Torque[i] = 0;
        _home[i] = legacy_homePos[i];
    }
}


robolli_legacy::~robolli_legacy() {
}

void robolli_legacy::init() {
}

/**
 * use actual joints position and set as 'home position' and 
 * move towards it (TODO.. at the moment this is not needed)
 */
void robolli_legacy::homing() {
    for(unsigned int i = 0; i < MAX_MC_BOARDS; ++i) {
        _pos[i] = _mc_bc_data_Position[i];
        _home[i] = _pos[i];
    }
}

unsigned long int robolli_legacy::get_time_ns() {
    return yarp::os::Time::now()*1E9;
}

void robolli_legacy::getRobolliLeftArm(const int* robolliVec,
                                       double* left_arm_data,
                                       const unsigned int scale,
                                       const int* y2r_offset) {
    for(unsigned int i = 0; i < l_arm_size; ++i)
        left_arm_data[i] = (robolliVec[l_arm[i]] -  y2r_offset[i])/scale;
    for(unsigned int i = 0; i < l_farm_size; ++i)
        left_arm_data[i+l_arm_size] = robolliVec[l_farm[i]]/scale;
}

void robolli_legacy::getRobolliRightArm(const int* robolliVec,
                                        double* right_arm_data,
                                        const unsigned int scale,
                                        const int* y2r_offset) {
    for(unsigned int i = 0; i < r_arm_size; ++i)
        right_arm_data[i] = (robolliVec[r_arm[i]] - y2r_offset[i])/scale;
    for(unsigned int i = 0; i < r_farm_size; ++i)
        right_arm_data[i+r_arm_size] = robolliVec[r_farm[i]]/scale;
}

void robolli_legacy::setRobolliLeftArm(int* robolliVec,
                                       const double* left_arm_data,
                                       const unsigned int scale,
                                       const int* y2r_offset) {
    for(unsigned int i = 0; i < l_arm_size; ++i)
        robolliVec[l_arm[i]] = (left_arm_data[i] + y2r_offset[i])*scale;
    for(unsigned int i = 0; i < l_farm_size; ++i)
        robolliVec[l_farm[i]] = left_arm_data[i+l_arm_size]*scale;
}

void robolli_legacy::setRobolliRightArm(int* robolliVec,
                                        const double* right_arm_data,
                                        const unsigned int scale,
                                        const int* y2r_offset) {
    for(unsigned int i = 0; i < r_arm_size; ++i)
        robolliVec[r_arm[i]] = (right_arm_data[i] + y2r_offset[i])*scale;
    for(unsigned int i = 0; i < r_farm_size; ++i)
        robolliVec[r_farm[i]] = right_arm_data[i+r_arm_size]*scale;
}


void robolli_legacy::updateFromYarp(const robot_state_input& inputs) {
    if(inputs.q.size() != r_arm_size+r_farm_size+l_arm_size+l_farm_size) {
        std::cout << "Error copying from yarp to robolli: q size error" << std::endl;
        return;
    }

    if(inputs.q_dot.size() != r_arm_size+r_farm_size+l_arm_size+l_farm_size) {
        std::cout << "Error copying from yarp to robolli: q_dot size error" << std::endl;
        return;
    }

    if(inputs.tau_left.size() != l_arm_size+l_farm_size) {
        std::cout << "Error copying from yarp to robolli: tau_left size error" << std::endl;
        return;
    }

    if(inputs.tau_right.size() != r_arm_size+r_farm_size) {
        std::cout << "Error copying from yarp to robolli: tau_right size error" << std::endl;
        return;
    }

    setRobolliLeftArm(_mc_bc_data_Position,
                      inputs.q.data(),
                      1E5*CTRL_DEG2RAD,
                      l_arm_offs);
    setRobolliLeftArm(_mc_bc_data_Velocity,
                      inputs.q_dot.data(),
                      1E3*CTRL_DEG2RAD,
                      no_offs);
    setRobolliLeftArm(_mc_bc_data_Torque,
                      inputs.tau_left.data(),
                      1E3,
                      no_offs);
    setRobolliRightArm(_mc_bc_data_Position,
                       &inputs.q[_iYarp.left_arm_dofs +
                                 _iYarp.left_hand_dofs],
                       1E5*CTRL_DEG2RAD,
                       r_arm_offs);
    setRobolliRightArm(_mc_bc_data_Velocity,
                       &inputs.q_dot[_iYarp.left_arm_dofs +
                                     _iYarp.left_hand_dofs],
                       1E3*CTRL_DEG2RAD,
                       no_offs);
    setRobolliRightArm(_mc_bc_data_Torque,
                       inputs.tau_right.data(),
                       1E3,
                       no_offs);
}

void robolli_legacy::updateToYarp(robot_joints_output& outputs) {
    if(outputs.q.size() != l_arm_size + l_farm_size + r_arm_size + r_farm_size) {
        std::cout << "Error copying from robolli to yarp: q size error" << std::endl;
        return;
    }

    getRobolliLeftArm(_pos,
                      outputs.q.data(),
                      1E-5*CTRL_RAD2DEG,
                      r_arm_offs);
    getRobolliRightArm(_pos,
                       outputs.q.data()+_iYarp.left_arm_dofs,
                       1E-5*CTRL_RAD2DEG,
                       l_arm_offs);
}
