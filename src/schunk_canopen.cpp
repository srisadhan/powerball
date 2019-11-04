
#include "powerball/schunk_canopen.h"


SchunkCANOpen::SchunkCANOpen() {
    memset(&_pb_state, 0, sizeof(Powerball_State));
}


SchunkCANOpen::~SchunkCANOpen() {
    close_connection();
}


void SchunkCANOpen::init() {
    connect();
    Init_CANopen(NODE_1, NODE_6);
}


void SchunkCANOpen::close_connection() {
    std::cout << "Closing canOpen device ..." << std::endl;

    NTCAN_RESULT res = canClose(hnd);
    if(res != NTCAN_SUCCESS) FormatCanError(res);
    else {
        std::cout << "The device was closed succesfully" << std::endl;
    }

}

void SchunkCANOpen::connect(int addr) {

    std::cout << "Connecting canOpen device..." << std::endl;

    CAN_IF_STATUS cstat;
    NTCAN_RESULT res;

    res = canOpen(addr, 0, NTCAN_NO_QUEUE, NTCAN_NO_QUEUE, CANOPEN_TIMEOUT_TX_MS, CANOPEN_TIMEOUT_RX_MS, &hnd);
    if(res != NTCAN_SUCCESS) {
        std::cout << "canOpen error - " << std::flush;
        FormatCanError(res);
        exit(0);
    }

    // Empty the buffer before execution
    res = canIoctl(hnd, NTCAN_IOCTL_FLUSH_RX_FIFO, NULL);
    if(res != NTCAN_SUCCESS) {
        std::cout << "canIoctl error - " << std::flush;
        FormatCanError(res);
    }

    res = canStatus(hnd, &cstat);
    if(res != NTCAN_SUCCESS) {
        std::cout << "canStatus error - " << std::flush;
        FormatCanError(res);
    }
    else {
        printf("Net 0 is %d \n", *cstat.boardid);
    }

    res = canSetBaudrate(hnd, NTCAN_BAUD_500);
    //res = canSetBaudrate(hnd, NTCAN_BAUD_1000);
    if(res != NTCAN_SUCCESS) {
        FormatCanError(res);
    }
}

void SchunkCANOpen::Init_CANopen(uint8_t firstCANid, uint8_t lastCANid) {
    NTCAN_RESULT res;

    for (int CANid = firstCANid; CANid<=lastCANid ; CANid++) {

        // response of the SDO
        res = canIdAdd(hnd, 0x580+CANid);
        if(res != NTCAN_SUCCESS)
            FormatCanError(res);
        else {
            printf("canId 0x58%d added successfully \n",CANid);
        }

        // TPDO #1
        res = canIdAdd(hnd, TPDO1_msg+CANid);
        if(res != NTCAN_SUCCESS)
            FormatCanError(res);
        else {
            printf("canId 0x40%d added successful \n",CANid );
        }

        // TPDO #2
        res = canIdAdd(hnd, TPDO2_msg+CANid);
        if(res != NTCAN_SUCCESS)
            FormatCanError(res);
        else {
            printf("canId 0x41%d added successfully \n",CANid);
        }

        sendNMT(CANid, NMT_START_REMOTE_NODE);
    }
}


void SchunkCANOpen::FormatCanError(long error) {
    char error_msg[255];
    canFormatError(error, NTCAN_ERROR_FORMAT_LONG, error_msg, sizeof(error_msg) - 1);
    printf("failed with error %lx : %s\n ", error, error_msg);
}
