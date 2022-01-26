#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library
#include <phidget21.h>                                      // Uses phidget 21 library
// boost headers
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>
#include "utils.h"

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

#define ESC_ASCII_VALUE                 0x1b
#define UPARROW_ASCII_VALUE             0x18
#define DOWNARROW_ASCII_VALUE           0x19

using namespace::std;

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
        printf(RED "%s \n" DEFAULT, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf(RED "%s \n" DEFAULT, packetHandler->getRxPacketError(dxl_error));
    }
}

// Connect to dynamixel, set the baudrate and enable torque
bool dxl_enable(int dxl_comm_result, uint8_t dxl_error, uint16_t dxl_present_position, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
    // Open port
    if (!portHandler->openPort())
    {
        printf(RED "Failed to open the port! \n" DEFAULT);
        return 0;
    }

    // Set port baudrate
    if (!portHandler->setBaudRate(BAUDRATE))
    {   
        printf(RED "Failed to change the baudrate! \n" DEFAULT);
        return 0;
    }

    // Enable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    dxl_comm_status(dxl_comm_result, dxl_error, packetHandler);

    printf(GREEN "Successfully connected to dynamixel! \n" DEFAULT);
    return 1;
}

// disable the dynamixel
void dxl_disable(int dxl_comm_result, uint8_t dxl_error, uint16_t dxl_present_position, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
    // Disable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    dxl_comm_status(dxl_comm_result, dxl_error, packetHandler);

    // Close port
    portHandler->closePort();
}

double linPot;
// Store the value of the potentiometer into linPot
int CCONV SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value)
{
    double potCenter = 460; // get the center position of the handle during calibration
    
    linPot = (Value - potCenter)*60/1024/1e3;
    printf("Pot value: %d, distance from center: %f \n", Value, linPot);
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

    printf(GREEN "Successfully connected to Phidget with %d sensors \n" DEFAULT, numSensors);
    return 1;
}    

int main(int argc, char**argv)
{   
    uint16_t dxl_goal_position = 200;      // goal position

    
    if (argc>1){
        stringstream strValue;
        strValue << argv[1];
        strValue >> dxl_goal_position;
    }

    // Initialize the PortHandler instance
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    int dxl_comm_result = COMM_TX_FAIL;     // Communication result 
    uint8_t dxl_error = 0;                  // Dynamixel error
    uint16_t dxl_present_position = 0;      // present position

    // connect to the available dynamixel
    if (!dxl_enable(dxl_comm_result, dxl_error, dxl_present_position, portHandler, packetHandler)) return 0;
    //Initializing phidget
    if (!phidget_enable()) return 0; // exit the code if cannot connect to phidget

    // Set dynamixel position 
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position, &dxl_error);
    
    // Stop thread
    bool stopFlag = false;
    boost::thread stop_thread(stop, &stopFlag);

    while(!stopFlag)
    {        
        usleep(1000);
        // Read present position
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
        dxl_comm_status(dxl_comm_result, dxl_error, packetHandler);

        // printf("[ID:%03d] PresPos:%03d\n", DXL_ID, dxl_present_position);
        // if (abs(dxl_goal_position - dxl_present_position) > 10)
        // {
        //     dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position, &dxl_error);
        // }
    }
    
    // disable torque and disconnect dynamixel 
    dxl_disable(dxl_comm_result, dxl_error, dxl_present_position, portHandler, packetHandler);

    return 1;
}