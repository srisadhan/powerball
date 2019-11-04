#ifndef _SCHUNK_CANOPEN_DEFINES_H
#define _SCHUNK_CANOPEN_DEFINES_H

const int CAN_ADDR = 60;

const uint8_t NODE_ALL = 0;
const uint8_t NODE_1 = 3;
const uint8_t NODE_2 = 4;
const uint8_t NODE_3 = 5;
const uint8_t NODE_4 = 6;
const uint8_t NODE_5 = 7;
const uint8_t NODE_6 = 8;

const int CANOPEN_TIMEOUT_TX_MS = 0;
const int CANOPEN_TIMEOUT_RX_MS = 0;

struct SDOkey {
        uint16_t index;
        uint8_t subindex;

        inline SDOkey(CMSG Msg):
            index((Msg.data[2] << 8) + Msg.data[1]),
            subindex(Msg.data[3]) {}

        inline SDOkey(uint16_t i, uint8_t s):
            index(i),
            subindex(s) {}
};


/***************************************************************/
//		define SDO protocol constants and functions
/***************************************************************/

const SDOkey STATUSWORD(0x6041, 0x0);
const SDOkey ERRORWORD(0x1001, 0x0);
const SDOkey DRIVERTEMPERATURE(0x22A2, 0x0);
const SDOkey MANUFACTURER(0x1002, 0x0);
const SDOkey MANUFACTURERDEVICENAME(0x1008, 0x0);
const SDOkey MANUFACTURERHWVERSION(0x1009, 0x0);
const SDOkey MANUFACTURERSOFTWAREVERSION(0x100A, 0x0);
const SDOkey PCBTEMPERATURE(0x200D, 0x1);

const SDOkey IDENTITYVENDORID(0x1018, 0x01);
const SDOkey IDENTITYPRODUCTCODE(0x1018, 0x02);
const SDOkey IDENTITYREVNUMBER(0x1018, 0x03);

const SDOkey ACTUALPOSITION(0x6064, 0x0);
const SDOkey TARGETPOSITION(0x607A, 0x0);
const SDOkey POSLIMITMIN(0x607D, 0x01);
const SDOkey POSLIMITMAX(0x607D, 0x02);
const SDOkey MAXVEL(0x6046, 0x02);

const SDOkey IP_TIME_UNITS(0x60C2, 0x1);
const SDOkey IP_TIME_INDEX(0x60C2, 0x2);
const SDOkey SYNC_TIMEOUT_FACTOR(0x200E, 0x0);


/*************************
 * Specific for schunk hardware
 ************************/
const SDOkey SCHUNKLINE(0x200b, 0x1);
const SDOkey SCHUNKDETAIL(0x200b, 0x3);

const SDOkey EXTENDED_STATUS(0x2050, 0x0);

/****************************************
 */

const SDOkey CONTROLWORD(0x6040, 0x0);
const SDOkey MODES_OF_OPERATION(0x6060, 0x0);
const SDOkey MODES_OF_OPERATION_DISPLAY(0x6061, 0x0);
const SDOkey ERROR_CODE(0x603F, 0x0);
const SDOkey ABORT_CONNECTION(0x6007, 0x0);
const SDOkey QUICK_STOP(0x605A, 0x0);
const SDOkey SHUTDOWN(0x605B, 0x0);
const SDOkey DISABLE_CODE(0x605C, 0x0);
const SDOkey HALT(0x605D, 0x0);
const SDOkey FAULT(0x605E, 0x0);
const SDOkey MODES(0x6060, 0x0);
const SDOkey BRAKE(0x60FE, 0x1);
const SDOkey VELOCITY_DEMAND(0x6043,0x0);
const SDOkey VELOCITY_ACCELERATION_DELTA_SPEED(0x6048,0x1);
const SDOkey VELOCITY_ACCELERATION_DELTA_TIME(0x6048,0x2);
const SDOkey VELOCITY_DECELERATION_DELTA_SPEED(0x6049,0x1);
const SDOkey VELOCITY_DECELERATION_DELTA_TIME(0x6049,0x2);
const SDOkey POSITION_DEMAND(0x6062,0x0);


//--- Constants for the PDO mapping

// Mapping and communication paramters

// RPDO #1 (1 object)
const SDOkey RPDO1_COMM(0x1400, 0x01);
const SDOkey RPDO1_TRANS(0x1400, 0x02);
const SDOkey RPDO1_MAPPING(0x1600, 0x0);
const SDOkey RPDO1_MAP_OBJ_1(0x1600, 0x01);
const SDOkey RPDO1_MAP_OBJ_2(0x1600, 0x02);

// RPDO #2 (2 objects)
const SDOkey RPDO2_COMM(0x1401, 0x01);
const SDOkey RPDO2_TRANS(0x1401, 0x02);
const SDOkey RPDO2_MAPPING(0x1601, 0x0);
const SDOkey RPDO2_MAP_OBJ_1(0x1601, 0x01);
const SDOkey RPDO2_MAP_OBJ_2(0x1601, 0x02);

// TPDO #1 (2 objects)
const SDOkey TPDO1_COMM(0x1800, 0x01);
const SDOkey TPDO1_TRANS(0x1800, 0x02);
const SDOkey TPDO1_MAPPING(0x1A00, 0x0);
const SDOkey TPDO1_MAP_OBJ_1(0x1A00, 0x01);
const SDOkey TPDO1_MAP_OBJ_2(0x1A00, 0x02);
const SDOkey TPDO1_MAP_OBJ_3(0x1A00, 0x03);

// TPDO #2 (1 objects)
const SDOkey TPDO2_COMM(0x1801, 0x01);
const SDOkey TPDO2_TRANS(0x1801, 0x02);
const SDOkey TPDO2_MAPPING(0x1A01, 0x0);
const SDOkey TPDO2_MAP_OBJ_1(0x1A01, 0x01);

// RPDO COB addresses (pc --> robot)
const int RPDO1_msg = 0x200;
const int RPDO2_msg = 0x300;

// TPDO COB addresses (robot --> pc)
const int TPDO1_msg = 0x180;
const int TPDO2_msg = 0x190;

//---


//-- Control word status
const uint16_t CONTROLWORD_SHUTDOWN = 0x0E;
const uint16_t CONTROLWORD_QUICKSTOP = 2;
const uint16_t CONTROLWORD_SWITCH_ON = 7;
const uint16_t CONTROLWORD_ENABLE_OPERATION = 15;
const uint16_t CONTROLWORD_ENABLE_MOVEMENT = 31;
const uint16_t CONTROLWORD_START_HOMING = 16;
const uint16_t CONTROLWORD_ENABLE_IP_MODE = 16;
const uint16_t CONTROLWORD_DISABLE_INTERPOLATED = 7;
const uint16_t CONTROLWORD_DISABLE_OPERATION = 7;
const uint16_t CONTROLWORD_DISABLE_VOLTAGE = 0x7D;
const uint16_t CONTROLWORD_FAULT_RESET_0 = 0x00;
const uint16_t CONTROLWORD_FAULT_RESET_1 = 0x80;
const uint16_t CONTROLWORD_HALT = 0x100;
const uint16_t CONTROLWORD_ENABLE_MOVEMENT_SPEED = 127;
//---

//--- Filtered status words (status && 79)
const uint16_t STATUS_NOT_READY_TO_SWITCH_ON = 0;
const uint16_t STATUS_SWITCH_ON_DISABLED = 64;
const uint16_t STATUS_READY_TO_SWITCH_ON = 1;
const uint16_t STATUS_SWITCHED_ON = 3;
const uint16_t STATUS_OPERATION_ENABLED = 7;
const uint16_t STATUS_FAULT = 8;
const uint16_t STATUS_QUICK_STOP_ACTIVE = 17;
//---

const uint8_t MODES_OF_OPERATION_HOMING_MODE = 0x6;
const uint8_t MODES_OF_OPERATION_PROFILE_POSITION_MODE = 0x1;
const uint8_t MODES_OF_OPERATION_VELOCITY_MODE = 0x2;
const uint8_t MODES_OF_OPERATION_PROFILE_VELOCITY_MODE = 0x3;
const uint8_t MODES_OF_OPERATION_TORQUE_PROFILE_MODE = 0x4;
const uint8_t MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE = 0x7;

/***************************************************************/
//	define NMT constants, variables and functions
/***************************************************************/

const uint8_t NMT_START_REMOTE_NODE     = 0x01;
const uint8_t NMT_STOP_REMOTE_NODE      = 0x02;
const uint8_t NMT_ENTER_PRE_OPERATIONAL = 0x80;
const uint8_t NMT_RESET_NODE            = 0x81;
const uint8_t NMT_RESET_COMMUNICATION   = 0x82;


#endif
