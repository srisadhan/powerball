#ifndef SCHUNK_POWERBALL_DEFINES_H
#define SCHUNK_POWERBALL_DEFINES_H

#include "TooN/TooN.h"

// Powerball state container
typedef struct Powerball_State {
    TooN::Vector<6,float> pos;
    TooN::Vector<6,float> vel;
    TooN::Vector<6,float> torque;
    TooN::Vector<6,int16_t> status_word;
} Powerball_State;

// encoder pulses per degree
const int ENCODER_PPD = 1000;

// motor rated torques
const int RATED_TORQUE_1 = 0x1B58;  // 7000mNm
const int RATED_TORQUE_2 = 0x1B58;  // 7000mNm
const int RATED_TORQUE_3 = 0x1B58;  // 7000mNm
const int RATED_TORQUE_4 = 0x1B58;  // 7000mNm
const int RATED_TORQUE_5 = 0x0581;  // 1409mNm
const int RATED_TORQUE_6 = 0x0581;  // 1409mNm

// conversion between node number (3 to 8) and motor index (1 to 6)
static uint8_t node2motor(uint8_t node_num) { return node_num-2; }
static uint8_t motor2node(uint8_t motor_num) { return motor_num+2; }

// convert between encoder units and radians
static float cnt2rad(int32_t cnt) { return ((float)cnt/(float)ENCODER_PPD)*M_PI/180.0f; }
static int32_t rad2cnt(float rad) { return (rad*180.0f/M_PI)*ENCODER_PPD; }

// conversion between adc units and amps
static int16_t amp2cnt(float amp) { return (int16_t)amp; }
static float cnt2amp(int16_t cnt) { return cnt; }

// conversion between mTn (milli rated torque) to torque
static int16_t t2rt(float t, int mot_num) {
    if (mot_num >= 1 && mot_num <= 4 ) {
        return t*1000*(1000/RATED_TORQUE_1);
    }
    else if (mot_num >= 5 && mot_num <= 6) {
        return t*1000*(1000/RATED_TORQUE_5);
    }
    else {
        std::cout << "t2rt: mot_num out of range. It was: " << mot_num << std::endl;
    }
}

static float rt2t(int16_t rt, int mot_num) {
    if (mot_num >= 1 && mot_num <= 4 ) {
        return ((float)rt)/1000.f*((float)RATED_TORQUE_1/1000.0f);
    }
    else if (mot_num >= 5 && mot_num <= 6) {
        return ((float)-rt)/1000.f*((float)RATED_TORQUE_5/1000.0f);
    }
    else {
        std::cout << "rt2t: mot_num out of range. It was: " << mot_num << std::endl;
        return 0.0f;
    }
}


#endif // SCHUNK_POWERBALL_DEFINES_H
