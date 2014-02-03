
#ifndef INTERFACE_H
#define INTERFACE_H



#define WALKMAN_DRC_VALVE_COMMAND_NONE ""
#define WALKMAN_DRC_VALVE_COMMAND_REACH "reach"
#define WALKMAN_DRC_VALVE_COMMAND_APPROACH "approach"
#define WALKMAN_DRC_VALVE_COMMAND_GRASP "grasp"
#define WALKMAN_DRC_VALVE_COMMAND_TURN "turn"
#define WALKMAN_DRC_VALVE_COMMAND_RELEASE "release"
#define WALKMAN_DRC_VALVE_COMMAND_MOVE_AWAY "move_away"
#define WALKMAN_DRC_VALVE_STATUS_WAITING "waiting"
#define WALKMAN_DRC_VALVE_STATUS_READY "ready"
#define WALKMAN_DRC_VALVE_STATUS_REACHING "reaching"
#define WALKMAN_DRC_VALVE_STATUS_REACHED "reached"
#define WALKMAN_DRC_VALVE_STATUS_GRASPING "grasping"
#define WALKMAN_DRC_VALVE_STATUS_GRASPED "grasped"
#define WALKMAN_DRC_VALVE_STATUS_APPROACHING "approaching"
#define WALKMAN_DRC_VALVE_STATUS_APPROACHED "approached"
#define WALKMAN_DRC_VALVE_STATUS_TURNING "turning"
#define WALKMAN_DRC_VALVE_STATUS_TURNED "turned"
#define WALKMAN_DRC_VALVE_STATUS_UNGRASPING "ungrasping"
#define WALKMAN_DRC_VALVE_STATUS_UNGRASPED "ungrasped"
#define WALKMAN_DRC_VALVE_STATUS_MOVING_AWAY "moving_away"
#define WALKMAN_DRC_VALVE_STATUS_MOVED_AWAY "moved_away"



namespace walkman {
namespace drc {
namespace valve {

enum status {
    READY,
    MOVING,
    STOPPED
};

enum command {
    NONE,
    FORWARD,
    BACKWARDS,
    RIGHT,
    LEFT,
    ROTATE_RIGHT,
    ROTATE_LEFT
};

class interface
{
    protected:
        command command_i;
        int amount_i;
        int seq_num_i;
        bool start_i;
        bool stop_i;
    public:
        virtual void getCommand(command& cmd, int& seq_num)=0;

        virtual bool getStart() { return start_i;}
        virtual bool getStop() { return stop_i; }

        virtual void setStatus(status status_o, int seq_num_o) = 0;
};

}
}
}

#endif