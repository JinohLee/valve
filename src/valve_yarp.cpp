#include <yarp/os/all.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <assert.h>
#include <valve_yarp.h>
#include <yarp/os/Time.h>
#include <iostream>

using namespace yarp::os;
using namespace yarp::sig;
using namespace walkman::drc::valve;

const float homePos[] = {
    // lower body #15
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    //  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15
    // upper body #10
    0, 60,  0, -45, 0, -60, 0,  -45, 0,  0,  0,  0, 0,  0 , 0,  0 };
    // 16, 17, 18, 19, 20,  21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31        

const int two_arms_rl[] = {16, 17, 18 ,19, 26, 27, 28, 32,
                          20, 21, 22, 23, 29, 30, 31, 33};

  valve_yarp::valve_yarp(const double period, int argc, char* argv[], yarp_interface& yarpInterface) :
      RateThread(int(period*1000.0)),
      iYarp(yarpInterface),
      _robolli_legacy(iYarp) {
      ;
  }


    bool valve_yarp::threadInit() {
        bIsRT = false;
        makeRealTime();
      
        tStart = yarp::os::Time::now();
        _robolli_legacy.init();
        //initialize variables
        manip_module.init(&_robolli_legacy);
        return true;
    }

    void valve_yarp::threadRelease() {
        ;
    }
  

void valve_yarp::run()
{

#ifdef TESTING_ENABLED
    char cmd;
    if(iYarp.getKBDCommand(cmd)) {
        switch ( cmd ) {
            case 'h':
                std::cout << "Set home pos\n";
                /** TODO homing!!!
                         to homePos
                         with velocity homeVel */
                break;

            case 't':
                std::cout << "trajectory\n";
                manip_module.doTrajectory();
                break;

            case 'r':
                std::cout << "reaching\n";
                manip_module.doReaching();
                break;

            case 'p':
                std::cout << "pushing towards the valve\n";
                manip_module.doPushing();
                break;

            case 'o':
                std::cout << "openning the hand\n";
                manip_module.doOpenHand();
            break;

            case 'm':
                std::cout << "moving far from the valve\n";
                manip_module.moveFarFromValve();
            break;

            case 'c':
                std::cout << "closing hands\n";
                manip_module.closeHands();
                break;

            case 'v':
                std::cout << "valve rotation\n";
                manip_module.rotateValve();
                break;
        }
    }
#endif

    const robot_state_input& inputs= iYarp.sense();

  
    double tCurrent=yarp::os::Time::now();
    unsigned long int tElapsedNs = (tCurrent - tStart)*1E9;

    const robot_joints_output& desired_outputs=controlLaw(inputs, tElapsedNs);

    if(desired_outputs.doMove) {
        iYarp.move(desired_outputs);
    }
}

  const robot_joints_output& valve_yarp::controlLaw (const robot_state_input& inputs, unsigned long int RTtime )
  {
//    static int i = 0;
    robot_joints_output& outputs=iYarp.getOutputs();

    manip_module._robolli_legacy->updateFromYarp(inputs);
    manip_module.updateFromRobolli(manip_module._robolli_legacy->_mc_bc_data_Position,
                                   manip_module._robolli_legacy->_mc_bc_data_Velocity,
                                   manip_module._robolli_legacy->_mc_bc_data_Torque);

//    i++;
//    if(i==1000) {
//        std::cout << "q_y:" << inputs.q.toString() << std::endl;
//        std::cout << "q_r:" << manip_module._robolli_legacy->_mc_bc_data_Position[two_arms_rl[0]-1];
//        for(unsigned int j = j; j<inputs.q.size(); ++j)
//            std::cout << " "
//                      <<  manip_module._robolli_legacy->_mc_bc_data_Position[two_arms_rl[j]-1];
//        std::cout << std::endl;
//        std::cout << "q_l:";
//        manip_module.mVars.q_l.print(std::cout);
//    }

    manip_module.controlLaw();

    if(manip_module.updateToRobolli(manip_module._robolli_legacy->_pos,
                                    manip_module._robolli_legacy->_home)) {
        manip_module._robolli_legacy->updateToYarp(outputs);
        outputs.doMove = true;

//        if(i==1000) {
//            std::cout << "q_y:" << outputs.q.toString() << std::endl;
//            std::cout << "pos:" << manip_module._robolli_legacy->_pos[two_arms_rl[0]-1];
//            for(unsigned int j = j; j<inputs.q.size(); ++j)
//                std::cout << " "
//                          <<  manip_module._robolli_legacy->_pos[two_arms_rl[j]-1];
//            std::cout << std::endl;
//            std::cout << "home:" << manip_module._robolli_legacy->_home[two_arms_rl[0]-1];
//            for(unsigned int j = j; j<inputs.q.size(); ++j)
//                std::cout << " "
//                          <<  manip_module._robolli_legacy->_home[two_arms_rl[j]-1];
//            std::cout << std::endl;
//            std::cout << "q_s:";
//            manip_module.mVars.delta_q_sum.print(std::cout);
//        }
    } else {
        outputs.doMove = false;
    }

//    if(i==1000)
//        i = 0;

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

  /** TODO implement */
  bool valve_yarp::setReady() {
      return true;
  }

  /** TODO implement */
  bool valve_yarp::setStopped() {
      return true;
  }

