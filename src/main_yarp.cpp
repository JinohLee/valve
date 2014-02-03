#include <yarp/os/all.h>
#include <unistd.h>
#include <valve_yarp.h>

#define dT "0.001" //[s]


using namespace walkman::drc::valve;

int main(int argc, char* argv[])
{
    yarp::os::Network yarp;
    while(!yarp.checkNetwork()){
        std::cout<<"yarpserver not running, pls run yarpserver"<<std::endl;
        sleep(1);
    }
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefault("period",dT);
    rf.setDefaultConfigFile("initial_config.ini");
    rf.configure("",argc,argv);

    valve_yarp mod;
    return mod.runModule(rf);
}
