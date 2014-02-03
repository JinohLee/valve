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

#if(FT_PORT)
    FT_left_arm_port.open("/coman/forceTorque/analog/left_arm:o");
    FT_right_arm_port.open("/coman/forceTorque/analog/right_arm:o");
#else
    yarp::os::Property FT_left_prop;
    FT_left_prop.put("device", "analogsensorclient");
    FT_left_prop.put("robotName", "coman");
    FT_left_prop.put("remote", "/coman/forceTorque/analog/left_arm");
    FT_left_prop.put("local", "/FT_client_left");
    FT_left_prop.put("rate", 1);

    polyDriver_left_arm_FT.open(FT_left_prop);
    if(!polyDriver_left_arm_FT.isValid())
        printf("error opening analogSensor");

    polyDriver_left_arm_FT.view(this->FT_left_arm);


    yarp::os::Property FT_right_prop;
    FT_right_prop.put("device", "analogsensorclient");
    FT_right_prop.put("robotName", "coman");
    FT_right_prop.put("remote", "/coman/forceTorque/analog/right_arm");
    FT_right_prop.put("local", "/FT_client_right");
    FT_right_prop.put("rate", 1);

    polyDriver_right_arm_FT.open(FT_right_prop);
    if(!polyDriver_right_arm_FT.isValid())
        printf("error opening analogServer");
#endif

#if TESTING_ENABLED
    command_KBD_port.open("/turn_valve/cmd_kbd:i");
#endif

    speed_port_left.open("/turn_valve/speed_left:i");
    yarp.connect("/turn_valve/speed_left:i","/coman/speed/analog/left_arm:o");

    speed_port_right.open("/turn_valve/speed_right:i");
    yarp.connect("/turn_valve/speed_right:i","/coman/speed/analog/right_arm:o");
    
    
    polyDriver_right_arm_FT.view(this->FT_right_arm);

    valve_data_port.open("/turn_valve/valve_data:i");
    command_port.open("/turn_valve/control:i");
    status_port.open("/turn_valve/status:o");       
    
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
    
    joint_numbers.push_back(left_arm_dofs);
    joint_numbers.push_back(right_arm_dofs);
    

    if(input.q.size()!=left_arm_dofs+right_arm_dofs) {
        input.q.resize(left_arm_dofs+right_arm_dofs,0.0);
    }

    if(outputs.q.size()!=left_arm_dofs+right_arm_dofs) {
        outputs.q.resize(left_arm_dofs+right_arm_dofs,0.0);
    }
    
    if(input.q_dot.size()!=left_arm_dofs+right_arm_dofs) {
        input.q_dot.resize(left_arm_dofs+right_arm_dofs,0.0);
    }
    input.tau_left.resize(8);
    input.tau_right.resize(8);
    //TODO place in the right piece of code (flat_walk_yarp) in the state machine
    positionControl_left_arm->setPositionMode();
    positionControl_right_arm->setPositionMode();
    
    
}

void yarp_interface::setMaxSpeed(double max_speed)
{
    yarp::sig::Vector max_speeds_l,max_speeds_r;
    max_speeds_l.resize(joint_numbers[0],max_speed);
    max_speeds_r.resize(joint_numbers[1],max_speed);
    positionControl_left_arm->setRefSpeeds(max_speeds_l.data());
    positionControl_right_arm->setRefSpeeds(max_speeds_r.data());
}


yarp_interface::~yarp_interface()
{
    status_port.close();
    command_port.close();
    start_port.close();
    stop_port.close();
    pause_port.close();
//    steps_port.close();
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

bool yarp_interface::getKBDCommand(char& cmd) {
    yarp::os::Bottle* bot_command = command_port.read(false);
    if(bot_command!= NULL && !bot_command->isNull()) {
        std::string command_string = bot_command->get(0).asString();
        if( command_string.size() > 0 &&
            command_string.at(0) != 0) {
            cmd = command_string.at(0);
            return true;
        }
    }
    return false;
}

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
    encodersMotor_right_arm->getEncoders(&input.q[left_arm_dofs]);
    std::cout<<input.q.toString()<<std::endl;
    //speed_port_left.read(input.q_dot.data());
    //speed_port_right.read(&input.q_dot[left_leg_dofs]);

#ifdef FT_ENABLED
#if(FT_PORT)
    yarp::os::Bottle* ft_arm_bot = FT_left_arm_port.read(false);
    if(!(ft_arm_bot == NULL) && !ft_arm_bot->isNull())
    {
        for(unsigned int i = 0; i < ft_arm_bot->size(); ++i)
            left_arm_FT[i] = ft_arm_bot->get(i).asDouble();
    }
    ft_arm_bot = FT_right_arm_port.read(false);
    if(!(ft_arm_bot == NULL) && !ft_arm_bot->isNull())
    {
        for(unsigned int i = 0; i < ft_arm_bot->size(); ++i)
            right_arm_FT[i] = ft_arm_bot->get(i).asDouble();
    }
#else
    FT_left_arm->read(input.tau_left);
    FT_right_arm->read(input.tau_right);
#endif
#endif
    return input;
}


void yarp_interface::move(const robot_joints_output& outputs) {
    //positionDirect_left_leg->setPositions(outputs.q.data());
    //positionDirect_right_leg->setPositions(&outputs.q[left_leg_dofs]);
    positionControl_left_arm->positionMove(outputs.q.data());
    positionControl_right_arm->positionMove(&outputs.q[left_arm_dofs]);
}

