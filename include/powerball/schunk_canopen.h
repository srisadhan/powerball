#ifndef SCHUNK_CANOPEN_H
#define SCHUNK_CANOPEN_H

#include <ntcan.h>
#include <iostream>
#include <unistd.h>
#include <cstring>
#include "schunk_canopen_defines.h"
#include "schunk_powerball_defines.h"
#include "TooN/TooN.h"

using namespace::TooN;

class SchunkCANOpen {
    public:
        SchunkCANOpen();
        ~SchunkCANOpen();

        void connect(int addr = CAN_ADDR);
        void init();

        //--- IO fcns
        void sendSync();
        void sendRTR(uint8_t CANid);
        void SDOwrite_8byte(uint8_t CANid, SDOkey sdo, int32_t value);
        void SDOwrite_1byte(uint8_t CANid, SDOkey sdo, int8_t value);
        void sendNMT(uint8_t CANid, uint8_t command);
        int32_t SDOread(uint8_t CANid, SDOkey sdo);
        void RPDOwrite(int rpdo_num, uint8_t CANid, int32_t obj_1, int32_t obj_2);
        //void TPDOread(uint8_t CANid, uint16_t &StatusWord, int16_t &ActualCurrent, int32_t &ActualPosition, int32_t &ActualVel);
        Powerball_State TPDOread();
        //---

    private:

        NTCAN_HANDLE hnd;

        Powerball_State _pb_state;

        void Init_CANopen(uint8_t firstCANid, uint8_t lastCANid);
        void FormatCanError(long error);

        void close_connection();

};


#endif
