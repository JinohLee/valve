#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <errno.h>
#include <assert.h>
#include <execinfo.h>
#include <exception>


//#include "armadillo"
#include <armadillo>
using namespace arma;

#include <boost/assign.hpp>

#include <thread_util.h>
#include "Boards_ctrl_basic.h"
#ifdef USE_ZMQ
    #include <zmq_publisher.h>
    zmq::context_t zmq_global_ctx(1);
#endif

static int loop = 1;  //Loop counter

////////////////////////////////////////////////////
// Static functions
////////////////////////////////////////////////////

static void stop_loop(int sig)
{
    loop = 0;
    printf("going down ... loop %d\n", loop);
}

static void warn_upon_switch(int sig __attribute__((unused)))
{
    // handle rt to nrt contex switch
    void *bt[32];
    int nentries;

    /* Dump a backtrace of the frame which caused the switch to
       secondary mode: */
    nentries = backtrace(bt,sizeof(bt)/sizeof(bt[0]));
    // dump backtrace 
    //backtrace_symbols_fd(bt,nentries,fileno(stdout));
}

static void set_signal_handler(void)
{
    signal(SIGINT,  stop_loop);
    //signal(SIGKILL, stop_loop);
    signal(SIGTERM, stop_loop);
#ifdef __XENO__
    // call pthread_set_mode_np(0, PTHREAD_WARNSW) to cause a SIGXCPU
    // signal to be sent when the calling thread involontary switches to secondary mode
    signal(SIGXCPU, warn_upon_switch);
#endif
}


////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////

int main(int argc, char *argv[])
{


try {

    ////////NORMAL ROBOLI CODE STARTS FROM HERE//////
    std::map<std::string, Thread_hook*> threads;

    // set signal handler
    set_signal_handler();   

#ifdef __XENO__
    /* Prevent any memory-swapping for this program */
    int ret = mlockall(MCL_CURRENT | MCL_FUTURE);
    if (ret < 0) {
        printf("mlockall failed (ret=%d) %s\n", ret, strerror(ret));
        exit(1);
    }
    /*
     * This is a real-time compatible printf() package from
     * Xenomai's RT Development Kit (RTDK), that does NOT cause//#include "armadillo"
#include <armadillo>
using namespace arma;
     * any transition to secondary (i.e. non real-time) mode when
     * writing output.
     */
    rt_print_auto_init(1);
#endif
    
    /////////////////////////////////////////////////////////////////
    //
    /////////////////////////////////////////////////////////////////
    threads["boards_ctrl"] = new Boards_ctrl_basic("config.yaml");
    threads["boards_ctrl"]->create();

#ifdef USE_ZMQ
    std::map<int, std::string> uri_list = boost::assign::map_list_of 
        //(JSON_EMITTER,"tcp://*:6666")
        //(TEXT_EMITTER,"tcp://*:6667")
        (CSTRUCT_EMITTER,"tcp://*:6668");
        //(CSTRUCT_EMITTER,"ipc:///tmp/6668");
    threads["zpub"] = new Zmq_pub_thread(std::string("boards_bc_data"), new Bc_Publisher(uri_list)); 
    threads["zpub"]->create(false);

    std::map<int, std::string> uri_list_hello = boost::assign::map_list_of 
        (0,"ipc:///tmp/HELLO_WORLD");
    //threads["zpub_test"] = new Zmq_pub_thread(std::string("random"), new Info_Publisher(uri_list_hello)); 
    //threads["zpub_test"]->create(false);
#endif

    while (loop) {
        sleep(1);
    }

    for (auto it = threads.begin(); it != threads.end(); it++) {
        it->second->stop();
        it->second->join();
        delete it->second;
    }

    std::cout << "Exit main" << std::endl;

    return 0;

} catch (std::exception& e) {

    std::cout << "Main catch .... " <<  e.what() << std::endl;

}
}


