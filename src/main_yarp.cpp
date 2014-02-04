#include <yarp/os/all.h>
#include <unistd.h>
#include <valve_yarp.h>

#define dT 0.001 //[s]


namespace walkman {
namespace drc {
namespace valve {

class valve_module: public yarp::os::RFModule
{
protected:
    valve_yarp* thr;
    yarp::os::RpcServer switch_rpc;
    yarp_interface iYarp;
public:
    valve_module() :iYarp() {
        this->thr = NULL;
        switch_rpc.open("/turn_valve/switch:i");
        this->attach(switch_rpc);
    }

    ~valve_module() {
        switch_rpc.close();
    }

    /** TODO, do we need this?
    bool configure ( yarp::os::ResourceFinder &rf )
    {
        return true;
        Value value=rf.find ( "period" );
        if ( value.isNull() ) {
            assert ( false );
            return false;
        } else {
            _period = value.asDouble()*1000.0;
        }
    } */

    /** TODO if we create the thread from respond, we need
             to copy argc and argv somewhere */
    bool my_configure(int argc, char* argv[])
    {
        std::cout<<"Starting Module valve_yarp"<<std::endl;

        if(thr == NULL) {
            thr = new valve_yarp(dT, argc, argv, iYarp);
            if(!thr->start())
            {
                delete thr;
                std::cout << "Error starting valve_yarp" << std::endl;
                return true;
            }
        }
        return true;
    }

    /** TODO in the future we will create the valve thread
             in here */
    virtual bool respond(const yarp::os::Bottle& command,
                        yarp::os::Bottle& reply) {
        std::string cmd_string = command.get(0).asString();
        if(cmd_string.compare(WALKMAN_DRC_VALVE_SWITCH_START) == 0) {
            if(thr->setReady()) {
                reply.addInt(1);
                return true;
            } else {
                reply.addInt(0);
                reply.addString("No command specified to robot");
                return false;
            }
        } else if(cmd_string.compare(WALKMAN_DRC_VALVE_SWITCH_STOP) == 0) {
            thr->setStopped();
            reply.addInt(1);
            return true;
        } else if(cmd_string.compare(WALKMAN_DRC_VALVE_SWITCH_PAUSE) == 0) {
            reply.addInt(0);
            reply.addString("Method not supported");
            return false;
        } else {
            reply.addInt(0);
            reply.addString("Unknown command");
            return false;
        }
        return false;
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


using namespace walkman::drc::valve;

int main(int argc, char* argv[])
{
    yarp::os::Network yarp;
    while(!yarp.checkNetwork()){
        std::cout<<"yarpserver not running, pls run yarpserver"<<std::endl;
        sleep(1);
    }

    valve_module mod;
    /** TODO do we need this?
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefault("period",dT);
    rf.setDefaultConfigFile("initial_config.ini");
    rf.configure("",argc,argv);

    return mod.runModule(rf); */

    mod.my_configure(argc,argv);
    return mod.runModule();
}
