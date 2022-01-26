#include "dynamixel_sdk.h"
#include "utils.h"
#include "vsm/vsm_control.h"
#include <vector>

VSMControl::VSMControl() {}

VSMControl::VSMControl(int baudrate, const char *device_name, float protocol_version, uint16_t addr_mx_torque_enable, uint16_t addr_mx_goal_position, uint16_t addr_mx_present_position)
{
    this->baudrate = baudrate;
    // this->device_name = device_name;
    this->addr_mx_torque_enable = addr_mx_torque_enable;
    this->addr_mx_goal_position = addr_mx_goal_position;
    this->addr_mx_present_position = addr_mx_present_position;

    // Initialize the PortHandler instance
    portHandler = dynamixel::PortHandler::getPortHandler(device_name);

    // Initialize PacketHandler instance
    packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);
}

VSMControl::~VSMControl()
{
  dxl_disable();
  portHandler->closePort();
}

// Get the number of connected dynamixels
int VSMControl::get_connected_dxl_count(std::vector<int> *dxl_id_list)
{
    int comm_success = 0;
    int dxl_count = 0;
    uint8_t dxl_error;
    uint16_t dxl_model_num;

    for (int id=1; id < 10; id++)
    {
      if (packetHandler->ping(portHandler, id, &dxl_model_num, &dxl_error)==COMM_SUCCESS)
      {
        dxl_count += 1;
        dxl_id_list->push_back(id);
      }
    }
    return dxl_count;
}

// Check the communication (packet transfer and receiving) status of the dynamixel
void VSMControl::dxl_comm_status(int dxl_comm_result, uint8_t dxl_error)
{
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf(RED "%s \n" DEFAULT, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf(RED "%s \n" DEFAULT, packetHandler->getRxPacketError(dxl_error));
    }
}

// Connect to dynamixel, set the baudrate and enable torque
bool VSMControl::dxl_enable()
{
    // Open port
    if (!portHandler->openPort())
    {
        printf(RED "Failed to open the port! \n" DEFAULT);
        return false;
    }

    // Set port baudrate
    if (!portHandler->setBaudRate(this->baudrate))
    {
        printf(RED "Failed to change the baudrate! \n" DEFAULT);
        return false;
    }

    dxl_count = get_connected_dxl_count(&dxl_id_list);
    // this->dxl_id_list = dxl_id_list;

    uint8_t torque_enable = 1;
    int dxl_comm_result;
    uint8_t dxl_error;
    for (int dxl_id : dxl_id_list)
    {
        // Enable Dynamixel Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, addr_mx_torque_enable, torque_enable, &dxl_error);
        dxl_comm_status(dxl_comm_result, dxl_error);
    }

    printf(GREEN "Successfully connected to dynamixel! \n" DEFAULT);
    return true;
}

// disable the dynamixel
void VSMControl::dxl_disable()
{
    uint8_t torque_disable = 0;
    uint8_t dxl_error = 0;
    int dxl_comm_result;

    for (int dxl_id : dxl_id_list)
    {
      // Disable Dynamixel Torque
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, addr_mx_torque_enable, torque_disable, &dxl_error);
      dxl_comm_status(dxl_comm_result, dxl_error);
    }
}

// convert the dynamixel encoder value to linear position
float VSMControl::convert_dxl2pos(uint32_t dxl_pos_t, double dxl_resolution, uint32_t vsm_min_pos, float passive_mag_width)
{
    return dxl_pos_t * dxl_resolution;
}

// convert dynamixel encoder value to position in vsm frame
float VSMControl::convert_dxl_to_VSMpos(uint32_t dxl_pos_t, double vsm_resolution, uint32_t vsm_min_pos, float passive_mag_width)
{   
    return passive_mag_width/2 + (dxl_pos_t - vsm_min_pos) * vsm_resolution;
}


void VSMControl::calculate_mag_sep(uint32_t dxl_pos_t, float handle_position, double vsm_resolution, uint32_t vsm_min_pos, float passive_mag_width, std::vector<float> *mag_sep) // , std::vector<float> *pos, std::vector<float> *sep)
{
    // 0.29 (width of passive magnet block) is the distance between outer magnets when the dynamixel is set to position 545
    // float pos2 =  passive_mag_width/2 + (dxl_pos_t - VSM_MIN_POS) * dxl_resolution;
    // float pos1 = -passive_mag_width/2 - (dxl_pos_t - VSM_MIN_POS) * dxl_resolution;

    float pos = convert_dxl_to_VSMpos(dxl_pos_t, vsm_resolution, vsm_min_pos, passive_mag_width);
    
    std::vector<float> sep{0, 0};
    sep[0] = handle_position + pos - (passive_mag_width/2);
    sep[1] = pos - handle_position - (passive_mag_width/2);
    
    *mag_sep = sep;
}

// get position for the rf series
uint16_t VSMControl::rf_get_pos(uint8_t dxl_id)
{
    uint8_t dxl_error;
    int dxl_comm_result;
    uint16_t dxl_pos_t;

    // Set dynamixel position
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_id, addr_mx_present_position, &dxl_pos_t, &dxl_error);
    return dxl_pos_t;
}

// set position for the rf series
void VSMControl::rf_set_pos(uint8_t dxl_id, uint16_t dxl_pos_t, uint16_t vsm_min_pos, uint16_t vsm_max_pos)
{
    uint8_t dxl_error;
    int dxl_comm_result;

    // check if the dxl_pos_t is within safe operating limits
    if (dxl_pos_t < vsm_min_pos)
    {
        dxl_pos_t = vsm_min_pos;
    }else if (dxl_pos_t > vsm_max_pos)
    {
        dxl_pos_t = vsm_max_pos;
    }

    // Set dynamixel position
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, addr_mx_goal_position, dxl_pos_t, &dxl_error);
    dxl_comm_status(dxl_comm_result, dxl_error);
}

// get position for the xm series
uint32_t VSMControl::xm_get_pos(uint8_t dxl_id)
{
    uint8_t dxl_error;
    int dxl_comm_result;
    uint32_t dxl_pos_t;

    // Set dynamixel position
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id, addr_mx_present_position, &dxl_pos_t, &dxl_error);
    return dxl_pos_t;
}

// set position for the xm series
void VSMControl::xm_set_pos(uint8_t dxl_id, uint32_t dxl_pos_t, uint32_t vsm_min_pos, uint32_t vsm_max_pos)
{
    uint8_t dxl_error;
    int dxl_comm_result;

    // check if the dxl_pos_t is within safe operating limits
    if (dxl_pos_t < vsm_min_pos)
    {
        dxl_pos_t = vsm_min_pos;
    }else if (dxl_pos_t > vsm_max_pos)
    {
        dxl_pos_t = vsm_max_pos;
    }

    // Set dynamixel position
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, addr_mx_goal_position, dxl_pos_t, &dxl_error);
    dxl_comm_status(dxl_comm_result, dxl_error);
}



// ------------------------------------------------------------------ //
// Impact adaptation strategies
// ------------------------------------------------------------------ //
// Set the active magnets to maximum posssible position to avoid impact transfer
// uint32_t impact_adaptation(uint32_t dxl_pos_t, uint32_t new_dxl_pos, float handle_position, float force_threshold)
// {
//     float res_force = abs(resForce(dxl_pos_t, handle_position));
//     if (res_force > force_threshold){
//             new_dxl_pos = vsm_max_pos - 25; // set the dynamixel to the fathest position minus 25 for the safety
//         }
//     return new_dxl_pos;
// }

// // detect if the force exceeds the threshold and change the disableTorque flag to true
// void detect_impact(uint32_t dxl_pos_t, float handle_position, float force_threshold, bool *disableTorque)
// {
//     float res_force = resForce(dxl_pos_t, handle_position);
//     printf("Handle position: %f, Resultant force: %f\n", linPot, res_force);

//     if (abs(res_force) > force_threshold){
//             *disableTorque = true;
//         }
// }

// // ------------------------------------------------------------------ //
// // Adaptation strategies
// // ------------------------------------------------------------------ //
// // The adaptation strategy will always try to move the magnet to the dxl_initial_pos once the external force is removed
// // ((1)) |F1 - F2|/K_virtual = x
// uint32_t spring_adaptation(uint32_t dxl_pos_t1, uint32_t dxl_pos_t, uint32_t new_dxl_pos, float handle_position, float K_virtual, float force_threshold)
// {
//     if (K_virtual < 1) K_virtual = 1; // prevent division by zero error

//     float res_force = abs(resForce(dxl_pos_t, handle_position));

//     if (res_force > force_threshold){
//         float estimated_pos = res_force / K_virtual ;
//         new_dxl_pos = dxl_pos_t1 +  estimated_pos / dxl_resolution;
//     }else{
//         new_dxl_pos = dxl_pos_t;
//     }

//     new_dxl_pos = set_safe_dxl_pos(new_dxl_pos);
//     return new_dxl_pos;
// }

// // Admittance adaptation strategy
// // ((2)) (|F1 - F2| - C_virtual * Vt)/M_virtual  * dt = Vt+1
// uint32_t admittance_adaptation(uint32_t dxl_pos_t1, uint32_t dxl_pos_t, float handle_position, float M_virtual, float C_virtual, float force_threshold)
// {
//     uint32_t new_dxl_pos;
//     float res_force;

//     res_force = abs(resForce(dxl_pos_t, handle_position));

//     if (res_force > force_threshold){
//         // float estimated_pos = (2 * res_force * dt * dt + 4*M_virtual * dxl_pos_t + (2*M_virtual - C_virtual*dt) * dxl_pos_t1) / (2*M_virtual + C_virtual * dt);
//         float estimated_pos = (2 * res_force * dt * dt) / (2*M_virtual + C_virtual * dt);

//         new_dxl_pos = dxl_pos_t + estimated_pos / dxl_resolution;
//     }else{
//         new_dxl_pos = dxl_pos_t;
//     }
//     printf("force:%f, current_pos:%d, new_pos:%d, handle_pos:%f\n", res_force, dxl_pos_t, new_dxl_pos, handle_position);
//     new_dxl_pos = set_safe_dxl_pos(new_dxl_pos);
//     return new_dxl_pos;
// }
