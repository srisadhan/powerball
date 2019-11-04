#include "powerball/schunk_canopen.h"


void SchunkCANOpen::sendSync() {
    NTCAN_RESULT res;
    CMSG c_msg;
    int32_t len;

    c_msg.id = 0x80;
    c_msg.len = 0x0;
    len = 1;
    res = canWrite(hnd, &c_msg, &len, NULL);
    if(res != NTCAN_SUCCESS)
        FormatCanError(res);
}


void SchunkCANOpen::sendRTR(uint8_t CANid) {
    NTCAN_RESULT res;
    CMSG c_msg;
    int32_t len;

    c_msg.id = 0x180+CANid;
    c_msg.len = 0x08 | NTCAN_RTR;
    len = 1;
    res = canWrite(hnd, &c_msg, &len, NULL);
    if(res != NTCAN_SUCCESS)
        FormatCanError(res);
}


void SchunkCANOpen::sendNMT(uint8_t CANid, uint8_t command) {
    NTCAN_RESULT res;
    CMSG c_msg;
    int32_t len;

    c_msg.id  = 0x0;
    c_msg.len = 0x2;
    c_msg.data[0] = command;
    c_msg.data[1] = CANid;
    len = 1;
    res = canWrite(hnd, &c_msg, &len, NULL);
    if(res != NTCAN_SUCCESS) FormatCanError(res);
}


void SchunkCANOpen::SDOwrite_8byte(uint8_t CANid, SDOkey sdo, int32_t value) {
    NTCAN_RESULT res;
    CMSG c_msg;
    int32_t len;

    c_msg.id  = 0x600+CANid;
    c_msg.len = 0x8;    // 8 bytes in the data field

    c_msg.data[0] = 0x23;
    c_msg.data[1] = sdo.index & 0xFF;
    c_msg.data[2] = (sdo.index >> 8) & 0xFF;
    c_msg.data[3] = sdo.subindex;

    c_msg.data[4] = value & 0xFF;
    c_msg.data[5] = (value >> 8) & 0xFF;
    c_msg.data[6] = (value >> 16) & 0xFF;
    c_msg.data[7] = (value >> 24) & 0xFF;
    len = 1;
    res = canWrite(hnd, &c_msg, &len, NULL);
    if(res != NTCAN_SUCCESS)
        FormatCanError(res);

    res = canRead(hnd, &c_msg, &len, NULL);
    if(res != NTCAN_SUCCESS)
        FormatCanError(res);
    else if(c_msg.data[0] != 0x60)
        printf("SDO write error!");

}


void SchunkCANOpen::SDOwrite_1byte(uint8_t CANid, SDOkey sdo, int8_t value) {
    NTCAN_RESULT res;
    CMSG c_msg;
    int32_t len;

    c_msg.id  = 0x600+CANid;
    c_msg.len = 0x8;    // 8 bytes in the data field
    c_msg.data[0] = 0x2F;
    c_msg.data[1] = sdo.index & 0xFF;
    c_msg.data[2] = (sdo.index >> 8) & 0xFF;
    c_msg.data[3] = sdo.subindex;
    c_msg.data[4] = value;
    c_msg.data[5] = 0x0;
    c_msg.data[6] = 0x0;
    c_msg.data[7] = 0x0;
    len = 1;
    res = canWrite(hnd, &c_msg, &len, NULL);
    if(res != NTCAN_SUCCESS)
        FormatCanError(res);

    res = canRead(hnd, &c_msg, &len, NULL);
    if(res != NTCAN_SUCCESS)
        FormatCanError(res);
    else if(c_msg.data[0] != 0x60)
        printf("SDO write error!");

}

int32_t SchunkCANOpen::SDOread(uint8_t CANid, SDOkey sdo) {
    NTCAN_RESULT res;
    CMSG c_msg;
    int32_t len;
    int32_t value = 0;
    char a[4];

    c_msg.id  = 0x600+CANid;
    c_msg.len = 0x8;
    c_msg.data[0] = 0x40;
    c_msg.data[1] = sdo.index & 0xFF;
    c_msg.data[2] = (sdo.index >> 8) & 0xFF;
    c_msg.data[3] = sdo.subindex;
    c_msg.data[4] = 0x0;
    c_msg.data[5] = 0x0;
    c_msg.data[6] = 0x0;
    c_msg.data[7] = 0x0;
    len = 1;
    res = canWrite(hnd, &c_msg, &len, NULL);
    if(res != NTCAN_SUCCESS)
        FormatCanError(res);

    res = canRead(hnd, &c_msg, &len, NULL);
    if(res != NTCAN_SUCCESS)
        FormatCanError(res);
    else {
        a[0] = c_msg.data[4];
        a[1] = c_msg.data[5];
        a[2] = c_msg.data[6];
        a[3] = c_msg.data[7];
        value = *(int *)a;
    }
    return value;
        //printf("canRead received %d message(s) with %d bytes ", len, c_msg.len);
}


void SchunkCANOpen::RPDOwrite(int rpdo_num, uint8_t CANid, int32_t obj_1, int32_t obj_2) {
    NTCAN_RESULT res;
    CMSG c_msg;
    int32_t len;

    if (rpdo_num < 1 && rpdo_num > 2) {
        std::cout << "rpdo_num out of range. input was: " << rpdo_num << std::endl;
        return;
    }

    switch (rpdo_num) {
        case(1): {            
            c_msg.id  = RPDO1_msg+CANid;
            c_msg.len = 0x6;

            // control word
            c_msg.data[0] = obj_1 & 0xFF;
            c_msg.data[1] = (obj_1 >> 8) & 0xFF;

            // target position
            c_msg.data[2] = obj_2 & 0xFF;
            c_msg.data[3] = (obj_2 >> 8) & 0xFF;
            c_msg.data[4] = (obj_2 >> 16) & 0xFF;
            c_msg.data[5] = (obj_2 >> 24) & 0xFF;

            len = 1;
        } break;

        case(2): {
            c_msg.id  = RPDO2_msg+CANid;
            c_msg.len = 0x4;

            // control word
            c_msg.data[0] = obj_1 & 0xFF;
            c_msg.data[1] = (obj_1 >> 8) & 0xFF;

            // target velocity
            c_msg.data[2] = obj_2 & 0xFF;
            c_msg.data[3] = (obj_2 >> 8) & 0xFF;

            len = 1;
        } break;

    } // switch

    // send the packet
    res = canWrite(hnd, &c_msg, &len, NULL);
    if(res != NTCAN_SUCCESS) FormatCanError(res);

}

Powerball_State SchunkCANOpen::TPDOread() {
    NTCAN_RESULT res;
    CMSG c_msg;
    int32_t len;
    char a1[2], a2[2], a3[4], a4[4];

    // Send sync message to the robot. It will reply with all the configured TPDO messages.
    sendSync();

    // read one packet at a time
    len = 1;

    int node_num = 0;

    for (int n = 0; n<12; n++) {

        res = canRead(hnd, &c_msg, &len, NULL);
        if(res != NTCAN_SUCCESS)
            FormatCanError(res);
        else {

            if ( (c_msg.id >= TPDO1_msg+NODE_1) &&
                 (c_msg.id <= TPDO1_msg+NODE_6) ) {

                node_num = c_msg.id - TPDO1_msg;

                // status word (16bit)
                a1[0] = c_msg.data[0];
                a1[1] = c_msg.data[1];

                // actual torque (16bit)
                a2[0] = c_msg.data[2];
                a2[1] = c_msg.data[3];

                // actual position (32bit)
                a3[0] = c_msg.data[4];
                a3[1] = c_msg.data[5];
                a3[2] = c_msg.data[6];
                a3[3] = c_msg.data[7];

                int32_t pos = *(int32_t*)a3;
                int16_t tor = *(int16_t*)a2;
                int mot_num = node2motor(node_num);

                _pb_state.status_word[mot_num-1] = *(int16_t*)a1;
                _pb_state.torque[mot_num-1] = rt2t( tor, mot_num );
                _pb_state.pos[mot_num-1] = cnt2rad(pos);
            } // if

            else if ( (c_msg.id >= TPDO2_msg+NODE_1) &&
                      (c_msg.id <= TPDO2_msg+NODE_6) ) {

                node_num = c_msg.id - TPDO2_msg;

                // velocity (32bit)
                a4[0] = c_msg.data[0];
                a4[1] = c_msg.data[1];
                a4[2] = c_msg.data[2];
                a4[3] = c_msg.data[3];
                _pb_state.vel[node2motor(node_num)-1] = cnt2rad(*(int32_t*)a4);
            } // else
        } // else (read was succesfull)
    }
    return _pb_state;

}


