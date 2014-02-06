#ifndef _VALVE_YARP_H_
#define _VALVE_YARP_H_

#include <string>
#include <pthread.h>
#include <sys/time.h>
#include <yarp/os/RateThread.h>
#include <yarp/sig/Vector.h>
#include <ValveModule.h>
#include <yarp_interface.h>
#include <iCub/ctrl/math.h>

#define CTRL_RAD2DEG    (180.0/M_PI)
#define CTRL_DEG2RAD    (M_PI/180.0)
#define TESTING_ENABLED true


namespace walkman
{
namespace drc
{
namespace valve
{

class valve_yarp: public yarp::os::RateThread
{

public:
  valve_yarp(const double period, int argc, char *argv[], yarp_interface &yarpInterface);

  virtual bool threadInit();
  virtual void threadRelease();

  virtual void run();

  bool setReady();
  bool setStopped();

private:
  
  const robot_joints_output& controlLaw ( const robot_state_input& inputs, unsigned long int RTtime );
  void makeRealTime();
  void clearCommand();
  status computeStatus();
  double tStart;
  bool bIsRT;
  
  yarp_interface& iYarp;
  ValveModule manip_module;
  robolli_legacy _robolli_legacy;
  
  status current_status;
  command last_command;
  double _period;

};

}
}
}


#endif
