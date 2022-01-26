#include <stdlib.h>
#include <TooN/TooN.h>

#include "rq_sensor_state.h"
#include "rq_sensor_com.h"

extern TooN::Vector<6, float> FT;

float calculate_travel_time(TooN::Vector<6, float> Q_start, TooN::Vector<6, float> Q_end, float max_Qdot);

void TCP_receive(bool *stopFlag, bool *FT_calibrated);

static INT_8 sensor_state_machine(unsigned int max_retries_, std::string &ftdi_id);

int wait_for_connection(unsigned int max_retries_, std::string &ftdi_id);

void Robotiq_ft_receive(float dt, bool Tare, bool *stopFlag, bool *FT_calibrated);
