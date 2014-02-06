#include <yarp_interface.h>
#include <iterator>
#include <algorithm>
#include <vector>
#include <iostream>
#include "interface.h"
using namespace walkman::drc::valve;

yarp_interface::yarp_interface()
{
    command_i = NONE;
    amount_i = 0;
    seq_num_i = -1;
    start_i = false;
    stop_i = false;

    if(createPolyDriver("left_arm", polyDriver_left_arm))
    {
        polyDriver_left_arm.view(encodersMotor_left_arm);
        polyDriver_left_arm.view(positionControl_left_arm);
        polyDriver_left_arm.view(controlMode_left_arm);
        polyDriver_left_arm.view(positionDirect_left_arm);
    }
    if(createPolyDriver("right_arm", polyDriver_right_arm))
    {
        polyDriver_right_arm.view(encodersMotor_right_arm);
        polyDriver_right_arm.view(positionControl_right_arm);
        polyDriver_right_arm.view(controlMode_right_arm);
        polyDriver_right_arm.view(positionDirect_right_arm);
    }

    if(createPolyDriver("left_hand", polyDriver_left_hand))
    {
        polyDriver_left_hand.view(encodersMotor_left_hand);
        polyDriver_left_hand.view(positionControl_left_hand);
        polyDriver_left_hand.view(controlMode_left_hand);
        polyDriver_left_hand.view(positionDirect_left_hand);
    }
    if(createPolyDriver("right_hand", polyDriver_right_hand))
    {
        polyDriver_right_hand.view(encodersMotor_right_hand);
        polyDriver_right_hand.view(positionControl_right_hand);
        polyDriver_right_hand.view(controlMode_right_hand);
        polyDriver_right_hand.view(positionDirect_right_hand);
    }

#if(FT_PORT)
    FT_left_arm_port.open("/coman/left_arm/analog:o/forceTorque");
    FT_right_arm_port.open("/coman/right_arm/analog:o/forceTorque");
#else
    yarp::os::Property FT_left_prop;
    FT_left_prop.put("device", "analogsensorclient");
    FT_left_prop.put("robotName", "coman");
    FT_left_prop.put("remote", "/coman/left_arm/analog:o/forceTorque");
    FT_left_prop.put("local", "/FT_client_left_arm:o");
    FT_left_prop.put("rate", 1);

    polyDriver_left_arm_FT.open(FT_left_prop);
    if(!polyDriver_left_arm_FT.isValid())
        printf("error opening analogSensor");

    polyDriver_left_arm_FT.view(this->FT_left_arm);


    yarp::os::Property FT_right_prop;
    FT_right_prop.put("device", "analogsensorclient");
    FT_right_prop.put("robotName", "coman");
    FT_right_prop.put("remote", "/coman/right_arm/analog:o/forceTorque");
    FT_right_prop.put("local", "/FT_client_right_arm:o");
    FT_right_prop.put("rate", 1);

    polyDriver_right_arm_FT.open(FT_right_prop);
    if(!polyDriver_right_arm_FT.isValid())
        printf("error opening analogServer");

    polyDriver_right_arm_FT.view(this->FT_right_arm);

#endif

#if TESTING_ENABLED
    command_KBD_port.open("/turn_valve/cmd_kbd:i");
#endif    

    valve_data_port.open("/turn_valve/valve_data:i");
    command_port.open("/turn_valve/control:i");
    status_port.open("/turn_valve/status:o");       
    
    left_hand_dofs = -1;
    right_hand_dofs = -1;

    left_arm_dofs = -1;
    right_arm_dofs = -1;
    int temp_storage=0;
    if(left_arm_dofs == -1) {
        positionControl_left_arm->getAxes(&temp_storage);
    }
    left_arm_dofs=temp_storage;
    
    if(right_arm_dofs == -1) {
        positionControl_right_arm->getAxes(&temp_storage);
    }
    right_arm_dofs=temp_storage;
    

    temp_storage=0;
    if(left_hand_dofs == -1) {
        positionControl_left_hand->getAxes(&temp_storage);
    }
    left_hand_dofs=temp_storage;

    if(right_hand_dofs == -1) {
        positionControl_right_hand->getAxes(&temp_storage);
    }
    right_hand_dofs=temp_storage;


    joint_numbers.push_back(left_arm_dofs);
    joint_numbers.push_back(right_arm_dofs);
    joint_numbers.push_back(left_hand_dofs);
    joint_numbers.push_back(right_hand_dofs);

    int total_dofs =    left_arm_dofs +
                        left_hand_dofs +
                        right_arm_dofs +
                        right_hand_dofs;
    int left_arm_total_dofs =   left_arm_dofs +
                                left_hand_dofs;
    int right_arm_total_dofs =  right_arm_dofs +
                                right_hand_dofs;
    if(input.q.size()!= total_dofs) {
        input.q.resize(total_dofs, 0.0);
    }

    if(outputs.q.size()!=   total_dofs) {
        outputs.q.resize(total_dofs,
                         0.0);
    }
    
    if(input.q_dot.size()!=total_dofs) {
        input.q_dot.resize(total_dofs,
                           0.0);
    }
    input.tau_left.resize(left_arm_total_dofs);
    input.tau_right.resize(right_arm_total_dofs);
    //TODO place in the right pSiece of code (flat_walk_yarp) in the state machine
    positionControl_left_arm->setPositionMode();
    positionControl_right_arm->setPositionMode();

    positionControl_left_hand->setPositionMode();
    positionControl_right_hand->setPositionMode();
}

void yarp_interface::setMaxSpeed(double max_speed)
{
    yarp::sig::Vector max_speeds_l,max_speeds_r;
    max_speeds_l.resize(joint_numbers[0],max_speed);
    max_speeds_r.resize(joint_numbers[1],max_speed);
    positionControl_left_arm->setRefSpeeds(max_speeds_l.data());
    positionControl_right_arm->setRefSpeeds(max_speeds_r.data());
    max_speeds_l.resize(joint_numbers[2],max_speed);
    max_speeds_r.resize(joint_numbers[3],max_speed);
    positionControl_left_hand->setRefSpeeds(max_speeds_l.data());
    positionControl_right_hand->setRefSpeeds(max_speeds_r.data());
}


yarp_interface::~yarp_interface()
{
    status_port.close();
    command_port.close();
    start_port.close();
    stop_port.close();
    pause_port.close();
    polyDriver_left_arm.close();
    polyDriver_right_arm.close();
    polyDriver_left_hand.close();
    polyDriver_right_hand.close();
}

bool yarp_interface::createPolyDriver(const std::string& kinematic_chain, yarp::dev::PolyDriver& polyDriver)
{
    yarp::os::Property options;
    options.put("robot", "coman");
    options.put("device", "remote_controlboard");

    yarp::os::ConstString s;
    s = "/turn_valve/" + kinematic_chain;
    options.put("local", s.c_str());

    yarp::os::ConstString ss;
    ss = "/coman/" + kinematic_chain;
    options.put("remote", ss.c_str());

    polyDriver.open(options);
    if (!polyDriver.isValid()){
        std::cout<<"Device "<<kinematic_chain<<" not available."<<std::endl;
        return false;
    }
    else{
        std::cout<<"Device "<<kinematic_chain<<" available."<<std::endl;
        return true;
    }
}


bool yarp_interface::getStart()
{
    yarp::os::Bottle* bot_start = start_port.read(false);
    if(!bot_start->isNull()) {
        start_i = bot_start->get(0).asBool();
        if(start_i == true)
            stop_i = false;
    }
    return start_i;
}

bool yarp_interface::getStop()
{
    yarp::os::Bottle* bot_stop = stop_port.read(false);
    if(!bot_stop->isNull()) {
        stop_i = bot_stop->get(0).asBool();
        if(stop_i == true)
            start_i = false;
    }
}

void yarp_interface::getCommand(command& cmd, int& seq_num) {
    yarp::os::Bottle* bot_command = command_port.read(false);
    if(!bot_command->isNull()) {
        std::string command_string = bot_command->get(0).asString();
        if(command_string==WALKMAN_DRC_VALVE_COMMAND_NONE) {
            
        } else if(command_string==WALKMAN_DRC_VALVE_COMMAND_REACH) {
            
        } else if(command_string==WALKMAN_DRC_VALVE_COMMAND_APPROACH) {
            
        } else if(command_string==WALKMAN_DRC_VALVE_COMMAND_GRASP) {
            
        } else if(command_string==WALKMAN_DRC_VALVE_COMMAND_TURN) {

        } else if(command_string==WALKMAN_DRC_VALVE_COMMAND_RELEASE) {            
            
        } else if(command_string==WALKMAN_DRC_VALVE_COMMAND_MOVE_AWAY) {
            
        }
        seq_num_i = bot_command->get(1).asInt();
    }
    cmd = command_i;
    seq_num = seq_num_i;
}

#ifdef TESTING_ENABLED
bool yarp_interface::getKBDCommand(char& cmd) {
    yarp::os::Bottle* bot_command = NULL;
    bot_command = command_KBD_port.read(false);
    if(bot_command != NULL && bot_command->size() > 0) {
        std::cout << "Reiceved command from keyboard: ";
        std::string command_string = bot_command->get(0).asString();
        if( command_string.size() > 0 &&
            command_string.at(0) != 0) {
            cmd = command_string.at(0);
            std::cout << cmd << std::endl;
            return true;
        }
    }
    return false;
}
#endif

void yarp_interface::setStatus(status status_o, int seq_num_o)
{
    yarp::os::Bottle bot;
    std::string status_string;
    switch(status_o) {
        case STOPPED:
            status_string = WALKMAN_DRC_VALVE_STATUS_WAITING;
            break;
        case MOVING:
            status_string = WALKMAN_DRC_VALVE_STATUS_REACHING;
            break;
        case READY:
        default:
            status_string = WALKMAN_DRC_VALVE_STATUS_READY;
            break;
    }

    bot.addString(status_string.c_str());
    bot.addInt(seq_num_o);
    status_port.write(bot);
}

//void yarp_interface::setSteps(int n_steps) {
//    yarp::os::Bottle bot;
//    bot.addInt(n_steps);
//    steps_port.write(bot);
//}


void yarp_interface::getValveData()
{
    yarp::os::Bottle* bot_valve = valve_data_port.read(false);
    valve_data.clear();
    if(!bot_valve->isNull()) {
        valve_data.push_back(bot_valve->get(0).asDouble());
        valve_data.push_back(bot_valve->get(1).asDouble());
        valve_data.push_back(bot_valve->get(2).asDouble());
        valve_data.push_back(bot_valve->get(3).asDouble());
        valve_data.push_back(bot_valve->get(4).asDouble());
        valve_data.push_back(bot_valve->get(5).asDouble());
        valve_data.push_back(bot_valve->get(6).asDouble());
    }    
}

const robot_state_input& yarp_interface::sense() 
{
    encodersMotor_left_arm->getEncoders(input.q.data());
    encodersMotor_left_hand->getEncoders(&input.q[  left_arm_dofs]);

    encodersMotor_right_arm->getEncoders(&input.q[  left_arm_dofs +
                                                    left_hand_dofs]);
    encodersMotor_right_hand->getEncoders(&input.q[ left_arm_dofs +
                                                    left_hand_dofs +
                                                    right_arm_dofs]);

    encodersMotor_left_arm->getEncoderSpeeds(input.q_dot.data());
    encodersMotor_left_hand->getEncoderSpeeds(&input.q_dot[  left_arm_dofs]);
    encodersMotor_right_arm->getEncoderSpeeds(&input.q_dot[  left_arm_dofs +
                                                                    left_hand_dofs]);
    encodersMotor_right_hand->getEncoderSpeeds(&input.q_dot[ left_arm_dofs +
                                                             left_hand_dofs +
                                                             right_arm_dofs]);
//    std::cout<<"q:     "<<input.q.toString()<<std::endl;
//    std::cout<<"q_dot: "<<input.q_dot.toString()<<std::endl;

#ifdef FT_ENABLED
#if(FT_PORT)
    yarp::os::Bottle* ft_arm_bot = FT_left_arm_port.read(false);
    if(!(ft_arm_bot == NULL) && !ft_arm_bot->isNull())
    {
        for(unsigned int i = 0; i < ft_arm_bot->size(); ++i)
            input.tau_left[i] = ft_arm_bot->get(i).asDouble();
    }
    ft_arm_bot = FT_right_arm_port.read(false);
    if(!(ft_arm_bot == NULL) && !ft_arm_bot->isNull())
    {
        for(unsigned int i = 0; i < ft_arm_bot->size(); ++i)
            input.tau_right[i] = ft_arm_bot->get(i).asDouble();
    }
#else
    FT_left_arm->read(input.tau_left);
    FT_right_arm->read(input.tau_right);
#endif
#endif
    return input;
}


void yarp_interface::move(const robot_joints_output& outputs) {
    positionDirect_left_arm->setPositions(outputs.q.data());
    positionDirect_left_hand->setPositions(&outputs.q[left_arm_dofs]);

    positionDirect_right_arm->setPositions(&outputs.q[  left_arm_dofs +
                                                        left_hand_dofs]);
    positionDirect_right_hand->setPositions(&outputs.q[ left_arm_dofs +
                                                        left_hand_dofs +
                                                        right_arm_dofs]);
//    positionControl_left_arm->positionMove(outputs.q.data());
//    positionControl_right_arm->positionMove(&outputs.q[left_arm_dofs]);
}

