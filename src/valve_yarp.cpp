#include <yarp/os/all.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <assert.h>
#include "valve_yarp.h"
#include <yarp/os/Time.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace walkman::drc::valve;


const std::vector<float> homePos = {
    // lower body #15
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    //  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15
    // upper body #10
    0, 60,  0, -45, 0, -60, 0,  -45, 0,  0,  0,  0, 0,  0 , 0,  0 };
    // 16, 17, 18, 19, 20,  21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
    
    
std::vector<int> two_arms = {16, 17, 18 ,19, 26, 27, 28, 32,
        20, 21, 22, 23, 29, 30, 31, 33};
        
std::vector<int> two_arms_nohands = {16, 17, 18 ,19, 26, 27, 28,
            20, 21, 22, 23, 29, 30, 31};
            

  double valve_yarp::getPeriod()
  {
    return .001;
  }

  bool valve_yarp::configure ( yarp::os::ResourceFinder &rf )
  {
      return true;
    Value value=rf.find ( "period" );
    if ( value.isNull() )
      {
        assert ( false );
        return false;
      }
      else
      {
	_period =  value.asDouble()*1000.0;
      }

    makeRealTime();
      
        _robolli_legacy.init(/* interface goes here */);
        //initialize variables
        manip_module.init(&_robolli_legacy);
      }
  

bool valve_yarp::updateModule()
{

#ifdef TESTING_ENABLED
    char cmd;
    if(iYarp.getKBDCommand(cmd)) {
        switch ( cmd ) {
            case 'h':
                DPRINTF("Set home pos\n");
                manip_module._robolli_legacy->homing(homePos, homeVel);
                break;

            case 't':
                DPRINTF("trajectory\n");
                manip_module.doTrajectory();
                break;

            case 'r':
                DPRINTF("reaching\n");
                manip_module.doReaching();
                break;

            case 'p':
                DPRINTF("pushing towards the valve\n");
                manip_module.doPushing();
                break;

            case 'o':
                DPRINTF("openning the hand\n");
                manip_module.doOpenHand();
            break;

            case 'm':
                DPRINTF("moving far from the valve\n");
                manip_module.moveFarFromValve();
            break;

            case 'c':
                DPRINTF("closing hands\n");
                manip_module.closeHands();
                break;

            case 'v':
                DPRINTF("valve rotation\n");
                manip_module.rotateValve();
                break;
        }
    }
#endif

    const robot_state_input& inputs= iYarp.sense();

  
    double tCurrent=yarp::os::Time::now();
    unsigned long int tElapsedNs = (tCurrent - tStart)*1E9;

    const robot_joints_output& desired_outputs=controlLaw(inputs, tElapsedNs);

    iYarp.move(desired_outputs);

    return true;
}

  const robot_joints_output& valve_yarp::controlLaw (const robot_state_input& inputs ,unsigned long int RTtime )
  {
    robot_joints_output& outputs=iYarp.getOutputs();

    // TODO need to call the robolli legacy code here,
    //      what about _ts_bc_data ?
    manip_module.updateFromRobolli(_ts_bc_data);

    manip_module.controlLaw();

    if(manip_module.updateToRobolli(manip_module._robolli_legacy->_pos,
                                    manip_module._robolli_legacy->_home))
        ;   // TODO copy the module _pos output to the outputs var

    return outputs;
  }


  
   void valve_yarp::makeRealTime()
  {
    if ( !bIsRT )
    {
        struct sched_param thread_param;
        thread_param.sched_priority = 99;
        pthread_setschedparam ( pthread_self(), SCHED_FIFO, &thread_param );
        bIsRT = true;
    }
  }
  
    void valve_yarp::clearCommand()
  {
    last_command = NONE;
  }

