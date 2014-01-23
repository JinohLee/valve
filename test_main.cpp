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
#include <Boards_ctrl_basic.h>
#ifdef USE_ZMQ
    #include <zmq_publisher.h>
    zmq::context_t zmq_global_ctx(1);
#endif

static int loop = 1;
/////////////////////////optimization and Matrix libraries////////
/*#include "optimization.h"
using namespace alglib;

#define WANT_STREAM                  // include.h will get stream fns
#define WANT_MATH                    // include.h will get math fns
                                     // newmatap.h will get include.h
#include "newmatap.h"                // need matrix applications
#include "newmatio.h"                // need matrix output routines

#ifdef use_namespace
using namespace NEWMAT;              // access NEWMAT namespace
#endif

Matrix jacob_right(6, 8);
Matrix jacob_left(6, 8);
Matrix jacob_R(6, 16);
IdentityMatrix I(3);*/

/////////////START: optimization //////////
/*void function1_grad(const real_1d_array &x, double &func, real_1d_array &grad, void *ptr)
{
    func = 100*pow(x[0]+3,4) + pow(x[1]-3,4);
    grad[0] = 400*pow(x[0]+3,3);
    grad[1] = 4*pow(x[1]-3,3);
}

real_1d_array x = "[5,5]";
real_2d_array c = "[[1,0,2],[1,1,6]]";
integer_1d_array ct = "[1,1]";
minbleicstate state;
minbleicreport rep;

double epsg = 0.000001;
double epsf = 0;
double epsx = 0;
ae_int_t maxits = 0;*/
/////////////END:  optimization //////////

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

   /* minbleiccreate(x, state);
    minbleicsetlc(state, c, ct);
    minbleicsetcond(state, epsg, epsf, epsx, maxits);
    alglib::minbleicoptimize(state, function1_grad);
    minbleicresults(state, x, rep);*/

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


