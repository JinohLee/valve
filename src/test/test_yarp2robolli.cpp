#include <yarp/os/all.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <assert.h>
#include <valve_yarp.h>
#include <yarp/os/Time.h>
#include <iostream>
#include <string>
#include <pthread.h>
#include <sys/time.h>
#include <yarp/os/RateThread.h>
#include <yarp/sig/Vector.h>
#include <yarp_interface.h>
#include <iCub/ctrl/math.h>
#define dT 0.001 //[s]


namespace walkman
{
namespace drc
{
namespace valve
{
namespace test
{

class valve_yarp: public yarp::os::RateThread
{

public:
  valve_yarp(const double period, int argc, char *argv[], yarp_interface &yarpInterface);

  virtual bool threadInit();
  virtual void threadRelease();

  virtual void run();

private:
  
  const walkman::drc::valve::robot_joints_output& controlLaw ( const walkman::drc::valve::robot_state_input& inputs, unsigned long int RTtime );
  double tStart;  
  walkman::drc::valve::yarp_interface& iYarp;
  walkman::drc::valve::robolli_legacy _robolli_legacy;
  
  double _period;

};

}
}
}
}


using namespace walkman::drc::valve;
using namespace yarp::os;
using namespace yarp::sig;

const int two_arms_rl[] = {16, 17, 18 ,19, 26, 27, 28, 32,
                          20, 21, 22, 23, 29, 30, 31, 33};

  test::valve_yarp::valve_yarp(const double period, int argc, char* argv[], yarp_interface& yarpInterface) :
      RateThread(int(period*1000.0)),
      iYarp(yarpInterface),
      _robolli_legacy(yarpInterface) {
      ;
  }


    bool test::valve_yarp::threadInit() {
        tStart = yarp::os::Time::now();
        _robolli_legacy.init();
        return true;
    }

    void test::valve_yarp::threadRelease() {
        ;
    }
  

void test::valve_yarp::run()
{
    const robot_state_input& inputs= iYarp.sense();

  
    double tCurrent=yarp::os::Time::now();
    unsigned long int tElapsedNs = (tCurrent - tStart)*1E9;

    const robot_joints_output& desired_outputs=controlLaw(inputs, tElapsedNs);

    iYarp.move(desired_outputs);
}

  const robot_joints_output& test::valve_yarp::controlLaw (const robot_state_input& inputs, unsigned long int RTtime )
  {
    static int i = 0;
    robot_joints_output& outputs=iYarp.getOutputs();

    _robolli_legacy.updateFromYarp(inputs);

    std::cout << _robolli_legacy._mc_bc_data_Position[two_arms_rl[0]-1];
    for(unsigned j = 1; j < 16; ++j)
        std::cout << " " << _robolli_legacy._mc_bc_data_Position[two_arms_rl[j]-1];
    std::cout << std::endl;
    i++;
//    if(i==1000) {
//        std::cout << "q_y:" << inputs.q.toString() << std::endl;
//        std::cout << "q_r:" << _robolli_legacy._mc_bc_data_Position[two_arms_y[0]-1];
//        for(unsigned int j = j; j<inputs.q.size(); ++j)
//            std::cout << " "
//                      <<  _robolli_legacy._mc_bc_data_Position[two_arms_y[j]-1];
//        std::cout << std::endl;
//    }

    
    /** control here */

    for(unsigned j= 0; j < 16; ++j)
        _robolli_legacy._pos[two_arms_rl[j]-1] = 0;
    _robolli_legacy._pos[two_arms_rl[0]-1] = -M_PI_2*1E5;
    _robolli_legacy._pos[two_arms_rl[1]-1] = M_PI_2*1E5;
    _robolli_legacy._pos[two_arms_rl[8]-1] = -M_PI_2*1E5;
    _robolli_legacy._pos[two_arms_rl[9]-1] = -M_PI_2*1E5;
    /*****************/

    _robolli_legacy.updateToYarp(outputs);


//    if(i==1000) {
//        std::cout << "q_y:" << outputs.q.toString() << std::endl;
//        std::cout << "pos:" << _robolli_legacy._pos[two_arms_y[0]-1];
//        for(unsigned int j = j; j<inputs.q.size(); ++j)
//            std::cout << " "
//                      <<  _robolli_legacy._pos[two_arms_y[j]-1];
//        std::cout << std::endl;
//        std::cout << "home:" << _robolli_legacy._home[two_arms_y[0]-1];
//        for(unsigned int j = j; j<inputs.q.size(); ++j)
//            std::cout << " "
//                      <<  _robolli_legacy._home[two_arms_y[j]-1];
//        std::cout << std::endl;
//    }

//    if(i==1000)
//        i = 0;

    return outputs;
  }


namespace walkman {
namespace drc {
namespace valve {

class valve_module: public yarp::os::RFModule
{
protected:
    test::valve_yarp* thr;
    yarp_interface iYarp;
public:
    valve_module() :iYarp() {
        this->thr = NULL;
    }

    ~valve_module() {
    }


    bool my_configure(int argc, char* argv[])
    {
        std::cout<<"Starting Module valve_yarp"<<std::endl;

        if(thr == NULL) {
            thr = new test::valve_yarp(dT, argc, argv, iYarp);
            if(!thr->start())
            {
                delete thr;
                std::cout << "Error starting valve_yarp" << std::endl;
                return true;
            }
        }
        return true;
    }


    virtual bool close()
    {
        if(thr != NULL) {
            thr->stop();
            delete thr;
            thr = NULL;
        }
        return true;
    }

    virtual double getPeriod(){return 1.0;}
    virtual bool updateModule(){return true;}
};

}
}
}


int main(int argc, char* argv[])
{
    yarp::os::Network yarp;
    while(!yarp.checkNetwork()){
        std::cout<<"yarpserver not running, pls run yarpserver"<<std::endl;
        sleep(1);
    }

    valve_module mod;
    mod.my_configure(argc,argv);
    return mod.runModule();
}
