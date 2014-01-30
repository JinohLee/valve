#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <Boards_exception.h>
#include <boost/tokenizer.hpp>
#include <boost/format.hpp>
#include "Boards_ctrl_basic.h"
#include <ValveModule.h>


// *************************ALL GLOBAL VARIABLES*******************************
static const std::vector<float> homeVel(25,25);

static const std::vector<float> homePos = {
    // lower body #15
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
//  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15
    // upper body #10
    0, 60,  0, -45, 0, -60, 0,  -45, 0,  0,  0,  0, 0,  0 , 0,  0 };
// 16, 17, 18, 19, 20,  21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31

std::vector<int> two_arms_ctrl = {16, 17, 18 ,19, 26, 27, 28, 32,
                                  20, 21, 22, 23, 29, 30, 31, 33};

std::vector<int> two_arms_nohands_ctrl = {16, 17, 18 ,19, 26, 27, 28,
                                     20, 21, 22, 23, 29, 30, 31};

Boards_ctrl_basic::Boards_ctrl_basic(const char * config): Boards_ctrl_ext(config) {
    name = "boards_ctrl_basic";
    period.period = {0,1000};

#ifdef __XENO__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max(schedpolicy);
    //stacksize = PTHREAD_STACK_MIN;

    xddp_test = new Write_XDDP_pipe(std::string(name), 4096);

    log_user_buff_Cart.set_capacity(LOG_SIZE);

    for (int i=0; i<16; i++)
    {
        log_user_buff[i].set_capacity(LOG_SIZE);
    }

}

Boards_ctrl_basic::~Boards_ctrl_basic() {

    //Save Cartesian data
    std::string filename = str(boost::format("/home/coman/AJ/COMAN_shared/examples/Arash_arma/Expdata/user_log_Cart.txt") );
    std::ofstream log_file(filename.c_str());

    for (boost::circular_buffer<log_user_t>::iterator it=log_user_buff_Cart.begin(); it!=log_user_buff_Cart.end(); it++) {
        log_file << boost::format("%1%\t") % (*it).ts;          //1

        log_file << boost::format("%1%\t") % (*it).CartXd_R;     //2
        log_file << boost::format("%1%\t") % (*it).CartYd_R;
        log_file << boost::format("%1%\t") % (*it).CartZd_R;

        log_file << boost::format("%1%\t") % (*it).CartXd_L;     //5
        log_file << boost::format("%1%\t") % (*it).CartYd_L;
        log_file << boost::format("%1%\t") % (*it).CartZd_L;

        log_file << boost::format("%1%\t") % (*it).CartX_R;     //8
        log_file << boost::format("%1%\t") % (*it).CartY_R;
        log_file << boost::format("%1%\t") % (*it).CartZ_R;

        log_file << boost::format("%1%\t") % (*it).CartX_L;     //11
        log_file << boost::format("%1%\t") % (*it).CartY_L;
        log_file << boost::format("%1%\t") % (*it).CartZ_L;

        log_file << boost::format("%1%\t") % (*it).rel_x;       //14
        log_file << boost::format("%1%\t") % (*it).rel_y;
        log_file << boost::format("%1%\n") % (*it).rel_z;
    }
    log_file << std::flush;
    log_file.close();


/*
    //Save data for joints
    for (int i=0; i<16; i++){
            std::string filename = str(boost::format("/home/coman/AJ/COMAN_shared/examples/Arash_arma/Expdata/user_log_%1%.txt") % (int)i);
            std::ofstream log_file(filename.c_str());

            for (boost::circular_buffer<log_user_t>::iterator it=log_user_buff[i].begin(); it!=log_user_buff[i].end(); it++) {
                //while ( ! dsp_log.empty() ) {
                log_file << boost::format("%1%\t") % (*it).ts;
                log_file << boost::format("%1%\n") % (*it).qm;
            }
            log_file << std::flush;
            log_file.close();
        }
*/

    delete xddp_test;

    std::cout << "~" << typeid(this).name() << std::endl;
}

void Boards_ctrl_basic::th_init(void *) {

    init();
    test();

    //initialize variables
    manip_module.init(this);
}



void Boards_ctrl_basic::th_loop(void * ) { 


    uint8_t cmd;
    log_user_t tmp_log;
    log_user_t tmp_log_Cart;

    static char console_buffer[1024];
    static char user_data[1024];
    try {

        sense();

        manip_module.updateFromRobolli(_ts_bc_data);

        bzero((void*)console_buffer, sizeof(console_buffer));
        user_input(console_buffer, sizeof(console_buffer));

        user_loop();

        if(manip_module.updateToRobolli(_pos,_home))
            move(MV_POS|MV_VEL|MV_TOR|MV_STF);

        sprintf(user_data, "%ld\n", get_time_ns());
        xddp_test->write((void*)user_data, strlen(user_data));
    } catch ( boards_error &e ){
        DPRINTF("FATAL ERROR in %s ... %s\n", __FUNCTION__, e.what());
        // handle error .... exit thread
        // exi_ts_bc_datat {rt,nrt}_periodic_thread function
        _run_loop = 0;

    } catch ( boards_warn &e ) {
        DPRINTF("WARNING in %s ... %s\n", __FUNCTION__, e.what());
        // handle warning
    }


    // ***** SAVE USER LOG

    // log user data
    tmp_log_Cart.ts = get_time_ns();//(get_time_ns() - g_tStart);
    tmp_log_Cart.CartXd_R = manip_module.mVars.Xd_R(0);
    tmp_log_Cart.CartYd_R = manip_module.mVars.Xd_R(1);
    tmp_log_Cart.CartZd_R = manip_module.mVars.Xd_R(2);
    tmp_log_Cart.CartXd_L = manip_module.mVars.Xd_L(0);
    tmp_log_Cart.CartYd_L = manip_module.mVars.Xd_L(1);
    tmp_log_Cart.CartZd_L = manip_module.mVars.Xd_L(2);

    //tmp_log_Cart.CartX_R = dXd_R(0);
    //tmp_log_Cart.CartY_R = dXd_R(1);
    //tmp_log_Cart.CartZ_R = dXd_R(2);

    //tmp_log_Cart.CartX_L = dXd_L(0);
    //tmp_log_Cart.CartY_L = dXd_L(1);
    //tmp_log_Cart.CartZ_L = dXd_L(2);

    const mat fkin_po_right = manip_module.mVars.getFkinPoR();
    const mat fkin_po_left = manip_module.mVars.getFkinPoL();

    tmp_log_Cart.CartX_R = manip_module.mVars.fkin_po_right(0);
    tmp_log_Cart.CartY_R = manip_module.mVars.fkin_po_right(1);
    tmp_log_Cart.CartZ_R = manip_module.mVars.fkin_po_right(2);

    tmp_log_Cart.CartX_L = manip_module.mVars.fkin_po_left(0);
    tmp_log_Cart.CartY_L = manip_module.mVars.fkin_po_left(1);
    tmp_log_Cart.CartZ_L = manip_module.mVars.fkin_po_left(2);


    tmp_log_Cart.rel_x = manip_module.mVars.obj_r_T(0);
    tmp_log_Cart.rel_y = manip_module.mVars.obj_r_T(1);
    tmp_log_Cart.rel_z = manip_module.mVars.obj_r_T(2);


    log_user_buff_Cart.push_back(tmp_log_Cart);


    // log user data
    for (int i=0;i<16; i++)
    {
            tmp_log.ts = get_time_ns();//(get_time_ns() - g_tStart);
            tmp_log.des_pos = (_pos[two_arms_nohands_ctrl[i]-1])/100000.0*180.0/M_PI;
            tmp_log.qm = manip_module.mVars.q_l(i)*180.0/M_PI;

            log_user_buff[i].push_back(tmp_log);
    }


}

int Boards_ctrl_basic::user_loop(void) {
    manip_module.controlLaw();
    return 0;
}

int Boards_ctrl_basic::user_input(void *buffer, ssize_t buff_size) {

    static int toggle = 1;
    McBoard * b;
    uint8_t cmd;

    // fakes ids ... used for test group functions
    uint8_t bIds[2] = { 88, 99};
    int pos[2];
    short vel[2];

    int nbytes = Boards_ctrl_ext::user_input(buffer, buff_size);

    if ( nbytes <= 0 ) {
        return nbytes;
    }

    // parse input
    std::vector<int> parsed_int;
    cmd = parse_console(buffer, nbytes, &parsed_int);
    if ( parsed_int.size() ) {
        for (auto it = parsed_int.begin(); it != parsed_int.end(); it++) {
            DPRINTF("%d ", *it);
        }
        DPRINTF("\n");
    }

    switch ( cmd ) {
        case 'S':
            DPRINTF("Start control ...two_arms\n");
            start_stop_set_control(two_arms_ctrl,true);
            break;

        case 'h':
            DPRINTF("Set home pos\n");
            homing(homePos, homeVel);
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

        default:
            DPRINTF("Stop control ...\n");
            start_stop_control(false);
            clear_mcs_faults();
            break;
    }

    return nbytes;
}


uint8_t Boards_ctrl_basic::parse_console(void * buffer, ssize_t buff_size, void * parsed) {

    char tmp[1024];
    uint8_t ret;

    // parse input
    bzero((void*)tmp, sizeof(tmp));
    memcpy((void*)tmp, buffer, buff_size);
    printf("%d : %s\n", buff_size, tmp);

    typedef boost::tokenizer< boost::char_separator<char> > Tokenizer;
    boost::char_separator<char> sep(";");
    std::string line(tmp);

    Tokenizer info(line, sep); // tokenize the line of data

    int cnt = 0;
    for (Tokenizer::iterator it = info.begin(); it != info.end(); ++it) {

        switch (cnt) {
            case 0:
                if (it->length() > 0) {
                    //std::cout << it->c_str() << std::endl;
                    ret = it->c_str()[0];
                }
                break;
            default:
                ((std::vector<int>*)parsed)->push_back(atoi(it->c_str()));
                break;
        }
        cnt ++;
    }

    return ret;


}

