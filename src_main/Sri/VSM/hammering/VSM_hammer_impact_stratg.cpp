#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <chrono>
#include <typeinfo> 
#include <fstream>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library
#include <phidget21.h>                                      // Uses phidget 21 library
// boost headers
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     2

#define ESC_ASCII_VALUE                 0x1b
#define UPARROW_ASCII_VALUE             0x18
#define DOWNARROW_ASCII_VALUE           0x19

#if defined(__linux__)
  //the following are UBUNTU/LINUX ONLY terminal color codes.
    #define RESET       "\033[0m"
    #define RED         "\033[31m"              /* Red */
    #define GREEN       "\033[32m"              /* Green */
#endif

using namespace::std;

float dt = 0.005f; // sampling time

const double dxl_resolution = 300 * M_PI/180 * 25.46e-3/2 /1024;       // multiply this value with the dynamixel encode value to get displacement
double passive_mag_width = .029;                                       // width of the magnet block holding hammer

// dynamixel calibration parameters
uint16_t VSM_MIN_POS = 555;                                           // calibrated position of the VSM to set the active magnets to 0.002 m on each side
uint16_t VSM_MAX_POS = 1023;                                           // calibrated maximum position of the active magnets in the VSM

// Initialize the PortHandler instance
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

// Initialize PacketHandler instance
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

int dxl_comm_result = COMM_TX_FAIL;         // Communication result 
uint8_t dxl_error = 0;                      // Dynamixel error
uint16_t dxl_pos_t;                         // dxl position at time step t (present position)
uint16_t dxl_pos_t1;                        // dxl position at previous time step t-1
uint8_t torque_status = 0;
uint16_t dxl_init_position = 600;           // initialize the goal position with a safe value
double linPot = 0;                          // potentiometer reading in m from the equilibrium postion
double pos1;                                // position of magnet 1
double pos2;                                // position of magnet 2
double sep1;                                // separation between magnet 1 and hammer
double sep2;                                // separation between magnet 2 and hammer


double K_virtual = 20;                    // initialize the virtual stiffness value for adaptation 
double M_virtual = 2.0;
double C_virtual = 100;
double force_threshold = 15.0;

// enter key press terminates the code
void stop(bool* flag){
    char in;
    cin.get(in);
    *flag = true;
}

// Check the communication (packet transfer and receiving) status of the dynamixel
void dxl_comm_status(int dxl_comm_result, uint8_t dxl_error, dynamixel::PacketHandler *packetHandler)
{
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf(RED "%s \n" RESET, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf(RED "%s \n" RESET, packetHandler->getRxPacketError(dxl_error));
    }
}

// Connect to dynamixel, set the baudrate and enable torque
bool dxl_enable(int dxl_comm_result, uint8_t dxl_error, uint16_t dxl_pos_t, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
    // Open port
    if (!portHandler->openPort())
    {
        printf(RED "Failed to open the port! \n" RESET);
        return 0;
    }

    // Set port baudrate
    if (!portHandler->setBaudRate(BAUDRATE))
    {   
        printf(RED "Failed to change the baudrate! \n" RESET);
        return 0;
    }

    // Enable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    dxl_comm_status(dxl_comm_result, dxl_error, packetHandler);

    printf(GREEN "Successfully connected to dynamixel! \n" RESET);
    return 1;
}

// disable the dynamixel
void dxl_disable(int dxl_comm_result, uint8_t dxl_error, uint16_t dxl_pos_t, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
    // Disable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    dxl_comm_status(dxl_comm_result, dxl_error, packetHandler);
}

float convert_dxl2pos(uint16_t dxl_pos_t){
    return passive_mag_width/2 + (dxl_pos_t - VSM_MIN_POS) * dxl_resolution;
}

// verify if the new dynamixel position is in the range of operation
uint16_t set_safe_dxl_pos(uint16_t new_dxl_pos)
{
    if (new_dxl_pos < VSM_MIN_POS) 
    {  
        new_dxl_pos = VSM_MIN_POS;
    }else if (new_dxl_pos > VSM_MAX_POS) 
    {   
        new_dxl_pos = VSM_MAX_POS;
    }
    return new_dxl_pos;
}

// Store the value of the potentiometer into linPot
int CCONV SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value)
{
    double potCenter = 532; // get the center position of the handle during calibration
    
    linPot = (Value - potCenter)*60/1024/1e3;
    // printf("Pot value: %d, distance from center: %f \n", Value, linPot);
    return 0;
}
// error 
int CCONV ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
    printf("Error handled. %d - %s", ErrorCode, unknown);
    return 0;
}

// Connect to phidget kit and set the detect if the sensor value is changed
bool phidget_enable()
{   
    int result, numSensors;
    const char *err;
    CPhidgetInterfaceKitHandle ifKit = 0;
    CPhidgetInterfaceKit_create(&ifKit);
    CPhidget_set_OnError_Handler((CPhidgetHandle)ifKit, ErrorHandler, NULL);
    CPhidgetInterfaceKit_set_OnSensorChange_Handler (ifKit, SensorChangeHandler, NULL);
    CPhidget_open((CPhidgetHandle)ifKit, -1);
    if((result = CPhidget_waitForAttachment((CPhidgetHandle)ifKit, 10000)))
    {
        CPhidget_getErrorDescription(result, &err);
        printf("Problem connecting to phidget: %s \n", err);
        return 0;
    }

    CPhidgetInterfaceKit_setRatiometric(ifKit, 0);
    CPhidgetInterfaceKit_getSensorCount(ifKit, &numSensors);
    
    CPhidgetInterfaceKit_setSensorChangeTrigger(ifKit, 0, 0);  //0 represents higher sensitivity

    printf(GREEN "Successfully connected to Phidget with %d sensors \n" RESET, numSensors);
    return 1;
}    

// Magnet force model
double magForce(double s){
  double C1 = 28.41;
  double C2 = 206.35;

  return C1 * exp(-C2 * s);
}

// resultant force on the object
double resForce(uint16_t dxl_pos_t, double handle_position){

    // 0.29 (width of passive magnet block) is the distance between outer magnets when the dynamixel is set to position 545
    // double pos2 =  passive_mag_width/2 + (dxl_pos_t - VSM_MIN_POS) * dxl_resolution; 
    // double pos1 = -passive_mag_width/2 - (dxl_pos_t - VSM_MIN_POS) * dxl_resolution;
    pos1 = -convert_dxl2pos(dxl_pos_t);
    pos2 = convert_dxl2pos(dxl_pos_t);
    
    sep1 = handle_position - pos1 - (passive_mag_width/2);
    sep2 = pos2 - handle_position - (passive_mag_width/2);

    if (sep1 < 0) sep1 = 0.0;
    if (sep2 < 0) sep2 = 0.0;

    // printf("%f, %f, %f \n", pos1, pos2, handle_position);
    // printf("Sep1: %f, Sep2: %f \n", sep1, sep2);
    return magForce(sep1) - magForce(sep2);
}


// ------------------------------------------------------------------ //
// Impact adaptation strategies
// ------------------------------------------------------------------ //
// Set the active magnets to maximum posssible position to avoid impact transfer
uint16_t impact_adaptation(uint16_t dxl_pos_t, uint16_t new_dxl_pos, double handle_position, double force_threshold)
{
    double res_force = abs(resForce(dxl_pos_t, handle_position));
    if (res_force > force_threshold){ 
            new_dxl_pos = VSM_MAX_POS - 25; // set the dynamixel to the fathest position minus 25 for the safety
        }
    return new_dxl_pos;
}

// detect if the force exceeds the threshold and change the disableTorque flag to true
void detect_impact(uint16_t dxl_pos_t, double handle_position, double force_threshold, bool *disableTorque)
{   
    double res_force = resForce(dxl_pos_t, handle_position);
    printf("Handle position: %f, Resultant force: %f\n", linPot, res_force);

    if (abs(res_force) > force_threshold){ 
            *disableTorque = true;
        }
}

// ------------------------------------------------------------------ //
// Adaptation strategies
// ------------------------------------------------------------------ //
// The adaptation strategy will always try to move the magnet to the dxl_initial_pos once the external force is removed 
// ((1)) |F1 - F2|/K_virtual = x 
uint16_t spring_adaptation(uint16_t dxl_pos_t1, uint16_t dxl_pos_t, uint16_t new_dxl_pos, double handle_position, double K_virtual, double force_threshold)
{   
    if (K_virtual < 1) K_virtual = 1; // prevent division by zero error

    double res_force = abs(resForce(dxl_pos_t, handle_position));

    if (res_force > force_threshold){
        double estimated_pos = res_force / K_virtual ;
        new_dxl_pos = dxl_pos_t1 +  estimated_pos / dxl_resolution;
    }else{
        new_dxl_pos = dxl_pos_t;
    }
    
    new_dxl_pos = set_safe_dxl_pos(new_dxl_pos);
    return new_dxl_pos;
}

// Admittance adaptation strategy
// ((2)) (|F1 - F2| - C_virtual * Vt)/M_virtual  * dt = Vt+1
uint16_t admittance_adaptation(uint16_t dxl_pos_t1, uint16_t dxl_pos_t, double handle_position, double M_virtual, double C_virtual, double force_threshold)
{   
    uint16_t new_dxl_pos;
    double res_force;
    
    res_force = abs(resForce(dxl_pos_t, handle_position));
    
    if (res_force > force_threshold){
        // double estimated_pos = (2 * res_force * dt * dt + 4*M_virtual * dxl_pos_t + (2*M_virtual - C_virtual*dt) * dxl_pos_t1) / (2*M_virtual + C_virtual * dt);
        double estimated_pos = (2 * res_force * dt * dt) / (2*M_virtual + C_virtual * dt);
        
        new_dxl_pos = dxl_pos_t + estimated_pos / dxl_resolution;
    }else{
        new_dxl_pos = dxl_pos_t;
    }
    printf("force:%f, current_pos:%d, new_pos:%d, handle_pos:%f\n", res_force, dxl_pos_t, new_dxl_pos, handle_position);
    new_dxl_pos = set_safe_dxl_pos(new_dxl_pos);
    return new_dxl_pos;
}

int main(int argc, char**argv)
{  
    if (argc > 1){
        // convert the char arguments to int using stringstream
        stringstream strValue1, strValue2;

        // first argument as dynamixel_goal_position        
        strValue1 << argv[1];
        strValue1 >> dxl_init_position;

        if (argc > 2){
            // second argument as the force threshold value
            strValue2 << argv[2];
            strValue2 >> force_threshold;
        }
    }

    // connect to the available dynamixel
    if (!dxl_enable(dxl_comm_result, dxl_error, dxl_pos_t, portHandler, packetHandler)) return 0;
    //Initializing phidget
    if (!phidget_enable()) return 0; // exit the code if cannot connect to phidget

    // Set dynamixel position 
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_pos_t, &dxl_error);
    while(abs(dxl_pos_t - dxl_init_position) > DXL_MOVING_STATUS_THRESHOLD){ 
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_init_position, &dxl_error);
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_pos_t, &dxl_error);
        dxl_comm_status(dxl_comm_result, dxl_error, packetHandler);
    }
    dxl_pos_t1 = dxl_pos_t; // pos at t and t-1 are same at t=0

    // Stop thread
    bool stopFlag = false;
    boost::thread stop_thread(stop, &stopFlag);

    // separate thread running to detect impact
    bool disableTorque = false;

    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;
    std::chrono::time_point<std::chrono::system_clock> timeLoop;
    std::chrono::duration<float> elaps_loop;

    // Open recording file
    std::ofstream dataFile;
    dataFile.open("../data/VSM/hammer/slider_friction.csv");
    dataFile << "Time,handle_pos,mag_pos1,mag_pos2,sep1,sep2" << endl;

    uint16_t new_dxl_pos = dxl_init_position;
    
    while(!stopFlag)
    {        
        timeLoop = std::chrono::system_clock::now();

        // detect if force exceeds a threshold
        detect_impact(dxl_pos_t, linPot, force_threshold, &disableTorque);

        // Read the torque status from dynamixel
        packetHandler->read1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, &torque_status);
        // printf("Torque status: %d", torque_status);

        if (!disableTorque)
        {   
            // Type 1 adaptation - Always return to initial position
            // new_dxl_pos = spring_adaptation(dxl_init_position, dxl_pos_t, linPot, K_virtual, force_threshold);

            // Type 1 adaptation - Spring only 
            // new_dxl_pos = spring_adaptation(dxl_pos_t1, dxl_pos_t, new_dxl_pos, linPot, K_virtual, force_threshold);
            
            // Type 2 adaptation - Admittance control
            // new_dxl_pos = admittance_adaptation(dxl_pos_t1, dxl_pos_t, linPot, M_virtual, C_virtual, force_threshold);

            // printf("new pos: %d \n", new_dxl_pos);
            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, new_dxl_pos, &dxl_error);
            dxl_comm_status(dxl_comm_result, dxl_error, packetHandler);

        }else if (disableTorque && torque_status){
            
            // disable torque and disconnect dynamixel 
            dxl_disable(dxl_comm_result, dxl_error, dxl_pos_t, portHandler, packetHandler);
        }            

        // Read present position
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_pos_t, &dxl_error);
        dxl_comm_status(dxl_comm_result, dxl_error, packetHandler);

        dxl_pos_t1 = dxl_pos_t;

        // timestamp
        gettimeofday(&tv, NULL);
        curtime=tv.tv_sec;
        strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));
        dataFile << buffer << ":" << tv.tv_usec << ",";

        dataFile << linPot << "," << pos1 << "," << pos2<< "," << sep1<< "," << sep2<< "\n";

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ((dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }
        else {cout << "Communication Time Out!" << endl;}
    }
    
    // disable torque and disconnect dynamixel 
    dxl_disable(dxl_comm_result, dxl_error, dxl_pos_t, portHandler, packetHandler);

    // Close port
    portHandler->closePort();

    // stop threads
    stop_thread.interrupt();

    // close the file 
    // dataFile.close();

    return 1;
}