#ifndef SCHUNK_POWERBALL_H
#define SCHUNK_POWERBALL_H

#include "powerball/schunk_canopen.h"
#include "powerball/schunk_powerball_defines.h"
#include <TooN/TooN.h>

using namespace::TooN;


class SchunkPowerball {
    public:

        SchunkPowerball();
        ~SchunkPowerball();

        //--- Node functions
        void init_comm();
        void set_control_mode(int8_t mode);
        uint32_t get_control_mode(); // added by Sri
        bool set_sdo_controlword(uint8_t node_num, int16_t state);  // By sdo
        //---

        //--- Get measures
        Vector<6,float> get_pos();
        Vector<6,float> get_vel();
        Vector<6,float> get_tor();
        float get_pos(int node_num);
        float get_vel(int node_num);
        float get_tor(int node_num);
        //---

        //--- Set setpoints
        Vector<6,float> set_pos(Vector<6,float> pos);
        Vector<6,float> set_vel(Vector<6,float> vel);
        Vector<6,float> set_tor(Vector<6,float> tor);
        void set_pos(int node_num, float pos);
        void set_vel(int node_num, float vel);
        void set_tor(int node_num, float tor);
        //---

        TooN::Vector<6,int16_t> get_status();
        int16_t get_status(int node_num);

        void update();

        void unbrake(int node_num);
        void brake(int node_num);

        void goHome();

        // search for commutation index
        void commutation_search(uint8_t CANid);

        void print_operation_mode();
        void shutdown_motors();

        SchunkCANOpen canopen;

    private:

        Powerball_State _pb_state_read;

        bool set_node_sdo_controlword(uint8_t node_num, int16_t state);
        uint16_t get_node_sdo_statusword(uint16_t node_num);

        void configure_PDOs();
        void configure_PDO(uint8_t CANid);
        void set_node_control_mode(uint8_t CANid, int8_t mode);

};

#endif // SCHUNK_POWERBALL_H
