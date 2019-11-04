
#include "powerball/schunk_powerball.h"

#define STATE_FILTER    (79)

uint16_t SchunkPowerball::get_node_sdo_statusword(uint16_t node_num) {
    uint16_t status = canopen.SDOread(node_num, STATUSWORD);

    int16_t status_filter = status & STATE_FILTER;

    switch (status_filter) {
        case STATUS_NOT_READY_TO_SWITCH_ON:
            return STATUS_NOT_READY_TO_SWITCH_ON;
            break;

        case STATUS_SWITCH_ON_DISABLED:
            return STATUS_SWITCH_ON_DISABLED;
            break;

        case STATUS_READY_TO_SWITCH_ON:
            return STATUS_READY_TO_SWITCH_ON;
            break;

        case STATUS_SWITCHED_ON :
            return STATUS_SWITCHED_ON;
            break;

        case STATUS_OPERATION_ENABLED :
            if ( (status & 32) == 32 )
                return STATUS_OPERATION_ENABLED;
            else
                return STATUS_QUICK_STOP_ACTIVE;
                break;

        case STATUS_FAULT :
            return STATUS_FAULT;
            break;

        default:
            return status;
    }
}


bool SchunkPowerball::set_node_sdo_controlword(uint8_t node_num, int16_t state) {
    int16_t actual_state = get_node_sdo_statusword(node_num);
    int32_t pos_actual = _pb_state_read.pos[node2motor(node_num)-1];

    do {
        actual_state = get_node_sdo_statusword(node_num);

        if (actual_state == state) {
            std::cout << "State: " << state << " was succesfully set to motor # " << (int)node2motor(node_num) << std::endl;
            return true;
        }

        if (actual_state == STATUS_FAULT) {
            printf("error = MOTOR FAULT! Reset Motor node: %d\n",node_num);
            canopen.RPDOwrite(1, node_num, CONTROLWORD_FAULT_RESET_0, 0);
            canopen.RPDOwrite(1, node_num, CONTROLWORD_FAULT_RESET_1, 0);
        }
        else if (actual_state == STATUS_SWITCH_ON_DISABLED) {
            canopen.RPDOwrite(1, node_num, CONTROLWORD_SHUTDOWN, 0);
        }
        else if (actual_state == STATUS_READY_TO_SWITCH_ON) {
            canopen.RPDOwrite(1, node_num, CONTROLWORD_SWITCH_ON, 0);
        }
        else if (actual_state == STATUS_SWITCHED_ON) {
            canopen.RPDOwrite(1, node_num, CONTROLWORD_ENABLE_OPERATION, 0);
        }
        else if (actual_state == STATUS_OPERATION_ENABLED) {
        }

    }
    while (actual_state != state );

}

bool SchunkPowerball::set_sdo_controlword(uint8_t node_num, int16_t state) {
    if (node_num == NODE_ALL) {
        std::cout << std::endl  << "Setting state " << state << " to all nodes ......." << std::endl;
        bool success = true;
        for (int k=NODE_1; k<= NODE_6; k++) {
            success &= set_node_sdo_controlword(k,state);
        }
        return success;
    }
    else if (node_num >= NODE_1 && node_num <= NODE_6) {
        std::cout << "Setting state: " << state << " to node number " << node_num << std::endl;
        return set_node_sdo_controlword(node_num, state);
    }
    else {
        std::cout << "set_state: node number is out of range\n" << std::endl;
        return false;
    }
}


void SchunkPowerball::unbrake(int node_num) {
    canopen.SDOwrite_8byte(node_num, BRAKE, 0);
}

void SchunkPowerball::brake(int node_num) {
    canopen.SDOwrite_8byte(node_num, BRAKE, 1);
}


