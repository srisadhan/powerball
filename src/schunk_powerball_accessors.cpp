
#include "powerball/schunk_powerball.h"


Vector<6,float> SchunkPowerball::set_pos(Vector<6, float> pos) {
    for (int k=NODE_1; k<=NODE_6; k++) {
        canopen.RPDOwrite(1, k, CONTROLWORD_ENABLE_MOVEMENT, rad2cnt(pos[ node2motor(k)-1 ]) );
    }
}

void SchunkPowerball::set_pos(int node_num, float pos) {
    canopen.RPDOwrite(1, node_num, CONTROLWORD_ENABLE_MOVEMENT, rad2cnt(pos) );
}

Vector<6,float> SchunkPowerball::set_vel(Vector<6,float> vel) {
    for (int k=NODE_1; k<=NODE_6; k++) {
        canopen.RPDOwrite(2, k, CONTROLWORD_ENABLE_MOVEMENT_SPEED, rad2cnt(vel[ node2motor(k)-1 ]) );
    }
}

void SchunkPowerball::set_vel(int node_num, float vel) {
    canopen.RPDOwrite(2, node_num, CONTROLWORD_ENABLE_MOVEMENT_SPEED, rad2cnt(vel) );
}

Vector<6,float> SchunkPowerball::get_pos() { return _pb_state_read.pos; }

float SchunkPowerball::get_pos(int node_num) {
    if (node_num < NODE_1 || node_num > NODE_6) {
        std::cout << "get_pos: node_num out of range. Requested node_num was: " << node_num << std::endl;
        return 0;
    }
    else {
        return _pb_state_read.pos[node2motor(node_num)-1];
    }
}

Vector<6,float> SchunkPowerball::get_vel() { return _pb_state_read.vel; }

float SchunkPowerball::get_vel(int node_num) {
    if (node_num < NODE_1 || node_num > NODE_6) {
        std::cout << "get_qdot: node_num out of range. Requested node_num was: " << node_num << std::endl;
        return 0;
    }
    else {
        return _pb_state_read.vel[node2motor(node_num)-1];
    }
}


Vector<6,float> SchunkPowerball::get_tor() {
    return _pb_state_read.torque;
}


float SchunkPowerball::get_tor(int node_num) {
    if (node_num < NODE_1 || node_num > NODE_6) {
        std::cout << "get_cur: node_num out of range. Requested node_num was: " << node_num << std::endl;
        return 0;
    }
    else {
        return _pb_state_read.torque[node2motor(node_num)-1];
    }
}

TooN::Vector<6,int16_t> SchunkPowerball::get_status() {
    return _pb_state_read.status_word;
}

int16_t SchunkPowerball::get_status(int node_num) {
    if (node_num < NODE_1 || node_num > NODE_6) {
        std::cout << "get_fsm_state: node_num out of range. Requested node_num was: " << node_num << std::endl;
        return 0;
    }
    else {
        return _pb_state_read.status_word[node2motor(node_num)-1];
    }
}
