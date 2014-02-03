#ifndef _YARP_INTERFACE_H_
#define _YARP_INTERFACE_H_

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/IAnalogSensor.h>
#include <vector>
#include <interface.h>

#define TESTING_ENABLED true
#define FT_ENABLED true
#define FT_PORT true

namespace walkman
{
namespace drc
{
namespace valve
{

struct robot_state_input
{
  yarp::sig::Vector q;
  yarp::sig::Vector q_dot;
  yarp::sig::Vector tau_right;
  yarp::sig::Vector tau_left;
};

struct robot_joints_output
{
  yarp::sig::Vector q;
};


class yarp_interface: interface
{
public:
  yarp_interface();
  ~yarp_interface();
  void getCommand ( command& cmd, int& seq_num );

#ifdef TESTING_ENABLED
  void getKBDCommand( char& cmd);
#endif

  void setMaxSpeed ( double max_speed );
  void getValveData();
  bool getStart();
  bool getStop();
  void setStatus ( status status_o, int seq_num_o );
//    void setSteps(int n_steps);

  const robot_state_input& sense();

  void move ( const robot_joints_output& outputs );
  
  inline robot_joints_output& getOutputs()
  {
      return outputs;
  }
  
  inline const std::vector<int>& getNumberOfJoints()
  {
    return joint_numbers;
  };

private:

  robot_state_input input;
  robot_joints_output outputs;
  
  std::vector<int> joint_numbers;
  std::vector<double> valve_data;
  yarp::os::Network yarp;
  
  yarp::dev::IEncodersTimed *encodersMotor_left_arm;
  yarp::dev::IEncodersTimed *encodersMotor_right_arm;
  yarp::dev::IPositionControl2 *positionControl_left_arm;
  yarp::dev::IPositionControl2 *positionControl_right_arm;
  yarp::dev::IPositionDirect *positionDirect_left_arm;
  yarp::dev::IPositionDirect *positionDirect_right_arm;
  
  yarp::dev::IControlMode *controlMode_left_arm;
  yarp::dev::IControlMode *controlMode_right_arm;

  yarp::dev::PolyDriver polyDriver_left_arm;
  yarp::dev::PolyDriver polyDriver_right_arm;

#ifdef FT_ENABLED

#if(FT_PORT)
    yarp::os::BufferedPort<yarp::os::Bottle> FT_left_arm_port;
    yarp::os::BufferedPort<yarp::os::Bottle> FT_right_arm_port;
#else
  yarp::dev::IAnalogSensor *FT_left_arm;
  yarp::dev::IAnalogSensor *FT_right_arm;

  yarp::dev::PolyDriver polyDriver_left_arm_FT;
  yarp::dev::PolyDriver polyDriver_right_arm_FT;
#endif

#endif

  yarp::os::BufferedPort<yarp::sig::Vector> speed_port_left;
  yarp::os::BufferedPort<yarp::sig::Vector> speed_port_right;
  
  
  yarp::os::BufferedPort<yarp::os::Bottle> valve_data_port;
  yarp::os::Port status_port;

  yarp::os::BufferedPort<yarp::os::Bottle> command_port;
#ifdef TESTING_ENABLED
  yarp::os::BufferedPort<yarp::os::Bottle> command_KBD_port;
#endif
  yarp::os::BufferedPort<yarp::os::Bottle> start_port;
  yarp::os::BufferedPort<yarp::os::Bottle> stop_port;
  yarp::os::BufferedPort<yarp::os::Bottle> pause_port;
  int left_arm_dofs;
  int right_arm_dofs;

  bool createPolyDriver ( const std::string &kinematic_chain, yarp::dev::PolyDriver &polyDriver );
};

}
}
}

#endif
