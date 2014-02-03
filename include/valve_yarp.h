#ifndef _VALVE_YARP_H_
#define _VALVE_YARP_H_

#include <string>
#include <pthread.h>
#include <sys/time.h>
#include <yarp/os/RateThread.h>
#include <yarp/sig/Vector.h>
#include <ValveModule.h>
#include <yarp_interface.h>
// #include "valve_ctrl.h"
// #include <utils.h>

#define CTRL_RAD2DEG    (180.0/M_PI)
#define CTRL_DEG2RAD    (M_PI/180.0)



namespace walkman
{
namespace drc
{
namespace valve
{

class valve_yarp: public yarp::os::RFModule
{

public:
  double getPeriod();
  bool configure ( yarp::os::ResourceFinder &rf );

  bool updateModule();

private:
  
  const robot_joints_output& controlLaw ( const walkman::drc::valve::robot_state_input& inputs, double RTtime );
  void makeRealTime();
  void clearCommand();
  status computeStatus();
  double tStart;
  
  yarp_interface iYarp;
  ValveModule manip_module;
  

  status current_status;
  command last_command;
  double _period;

};

}
}
}


#endif
