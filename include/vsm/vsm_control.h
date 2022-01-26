/*******************************************************************************
* Dynamixel control functions for gripper using DynamixelSDK
* Created on: 10/04/2021
* Author : Sri Sadhan
*******************************************************************************/

#include "dynamixel_sdk.h"
#include <utils.h>


class VSMControl{
    public:
        VSMControl();
        VSMControl(int baudrate, const char *device_name, float protocol_version, uint16_t addr_mx_torque_enable, uint16_t addr_mx_goal_position, uint16_t addr_mx_present_position);
        
        ~VSMControl();
        
        void dxl_comm_status(int dxl_comm_result, uint8_t dxl_error);
        bool dxl_enable();
        void dxl_disable();
        int get_connected_dxl_count(std::vector<int> *dxl_id_list);

        float convert_dxl2pos(uint32_t dxl_pos_t, double dxl_resolution, uint32_t vsm_min_pos, float passive_mag_width);
        float convert_dxl_to_VSMpos(uint32_t dxl_pos_t, double vsm_resolution, uint32_t vsm_min_pos, float passive_mag_width);
        void calculate_mag_sep(uint32_t dxl_pos_t, float handle_position, double vsm_resolution, uint32_t vsm_min_pos, float passive_mag_width, std::vector<float> *mag_sep); 

        uint16_t rf_get_pos(uint8_t dxl_id);
        void rf_set_pos(uint8_t dxl_id, uint16_t dxl_pos_t, uint16_t vsm_min_pos, uint16_t vsm_max_pos);
        
        uint32_t xm_get_pos(uint8_t dxl_id);
        void xm_set_pos(uint8_t dxl_id, uint32_t dxl_pos_t, uint32_t vsm_min_pos, uint32_t vsm_max_pos);

    private:
        int baudrate;
        int dxl_count;
        std::vector<int> dxl_id_list; 
        // const char *device_name;


        uint16_t addr_mx_torque_enable;
        uint16_t addr_mx_goal_position;
        uint16_t addr_mx_present_position;
        
        dynamixel::PortHandler *portHandler;
        dynamixel::PacketHandler *packetHandler;

        float pos; // magnet position 
        std::vector<float> sep;
};