#include <robolli_legacy.h>
#include <vector>

robolli_legacy::robolli_legacy() {

}


robolli_legacy::~robolli_legacy() {

}

void robolli_legacy::init(/* interface goes here? */) {

}

/**
 * use actual joints position and set as 'home position' and 
 * move towards it 
 * 
 */
void robolli_legacy::homing() {
    int         dummy, bId = 0;
    std::vector<float> pos(MAX_MC_BOARDS);
    std::vector<float> vel(MAX_MC_BOARDS);
/* original code here */
//    get_bc_data(_ts_bc_data);

//    for (auto it = _mcs.begin(); it != _mcs.end(); it++) {
//        bId = it->first;
//        pos[bId-1] = mRAD2DEG(_ts_bc_data[bId-1].raw_bc_data.mc_bc_data.Position);
//        vel[bId-1] = 25;
//        it->second->get_PID(POSITION_GAINS, _stiff[bId-1], dummy, _damp[bId-1]);
//        //std::cout << pos[bId-1] << std::endl;
//    }
        
//    homing(pos, vel);
}

/**
 * set 'home position' and move towards it
 *  
 * @param pos_deg 
 * @param vel_deg_s 
 */ 
void robolli_legacy::homing(const std::vector<float> &pos_deg, const std::vector<float> &vel_deg_s) {

    int bId = 0;
//    if ( pos_deg.size() > MAX_MC_BOARDS) {
//        throw(std::runtime_error(std::string("wrong size pos_deg")));
//    }
//    if ( vel_deg_s.size() > MAX_MC_BOARDS) {
//        throw(std::runtime_error(std::string("wrong size vel_deg_s")));
//    }

//    for (auto it = _mcs.begin(); it != _mcs.end(); it++) {
//        bId = it->first;
//        _pos[bId-1]  = DEG2mRAD(pos_deg[bId-1]);
//        _home[bId-1] = DEG2mRAD(pos_deg[bId-1]);
//        _vel[bId-1]  = DEG2RAD(vel_deg_s[bId-1])*1000;
//    }

//    move();
}

unsigned long int robolli_legacy::get_time_ns() {
    //return yarp::os::Time::now()*1E9;
    return 0.0;
}
