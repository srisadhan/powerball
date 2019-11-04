
#include <fstream>
#include <chrono>
#include <istream>
#include <string>
#include <sstream>
#include <vector>
#include <iostream>
#include "TooN/TooN.h"
#include <math.h>
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <utils.h>
#include <USB2Dynamixel.h>
#include <commonOptions.h>
#include <phidget21.h>

using namespace::std;
using namespace::TooN;

namespace
{
commonOptions::Option<std::vector<std::string>> cnfDevice("usb2dyn.device", {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"}, "device for usb2BaccaratDealer");
commonOptions::Option<std::string> cfgConfigFile("file", "motorConfig.json", "file to work on");

commonOptions::Option<int> cfgSetupMotorID("id", -1, "setup motor to 1000000 baud and id (parameter)");
commonOptions::Switch swtScanAll("scanAll", "scan all motors");

commonOptions::Switch swtHelp("help", "show help", []() {
    commonOptions::print();
    exit(EXIT_SUCCESS);
});
}

//the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET   "\033[0m"
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */

static uint16_t readPosition(dynamixel::motorID motor, USB2Dynamixel &usb2Dynamixel)
{
    std::mutex mutex;
    uint16_t position;
    usb2Dynamixel.read(motor, dynamixel::Register::PRESENT_POSITION, 2, 0.01 * seconds,
                       [&](dynamixel::motorID, bool success, uint8_t, const uint8_t* receiveBuffer, uint8_t)
    {
        if (success)
        {
            position = *((uint16_t*)receiveBuffer);
        }
    }, &mutex);
    mutex.lock();
    return position;
}


static uint16_t readTorqueLimit(dynamixel::motorID motor, USB2Dynamixel &usb2Dynamixel)
{
    std::mutex mutex;
    uint16_t limit;
    usb2Dynamixel.read(motor, dynamixel::Register::TORQUE_LIMIT, 2, 0.01 * seconds,
                       [&](dynamixel::motorID, bool success, uint8_t, const uint8_t* receiveBuffer, uint8_t)
    {
        if (success)
        {
            limit = *((uint16_t*)receiveBuffer);
        }
    }, &mutex);
    mutex.lock();
    return limit;
}
static void setPosition(dynamixel::motorID motor, uint16_t targetPos, USB2Dynamixel &usb2Dynamixel) {
    uint8_t targetPosLow = (targetPos >> 0) & 0xff;
    uint8_t targetPosHigh = (targetPos >> 8) & 0xff;
    usb2Dynamixel.write(motor, dynamixel::Register::GOAL_POSITION, {targetPosLow, targetPosHigh});
}

static void setMaxTorque(dynamixel::motorID motor, uint16_t torqueLimit, USB2Dynamixel &usb2Dynamixel) {
    uint8_t torqueLimitLow = (torqueLimit >> 0) & 0xff;
    uint8_t torqueLimitHigh = (torqueLimit >> 8) & 0xff;
    usb2Dynamixel.write(motor, dynamixel::Register::TORQUE_LIMIT, {torqueLimitLow, torqueLimitHigh});
}

int x[8];

int CCONV SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value)
{
    x[Index] = Value;
    return 0;
}

int CCONV ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
    printf("Error handled. %d - %s", ErrorCode, unknown);
    return 0;
}



int main(int argc, char **argv) {
    int result, numSensors, i;
    const char *err;

    cout << GREEN << "start" << RESET << endl;
    //usleep(5000*1000);
    USB2Dynamixel usb2dyn(*cnfDevice, 50);
    int id1 = 1, id2 = 2;

    // homing for the gripper
    uint16_t actualpos1 = readPosition(id1, usb2dyn);
    uint16_t actualpos2 = readPosition(id2, usb2dyn);
    setMaxTorque(id1, 75, usb2dyn);
    setMaxTorque(id2, 75, usb2dyn);
    uint16_t limit1 = readTorqueLimit(id1,usb2dyn);
    uint16_t limit2 = readTorqueLimit(id2,usb2dyn);
    cout << limit1 << "   " << limit2 << endl;
    if ((limit1==75) && (limit2==75)){
        cout << "Gripper motors torque limits set to 75. Starting Homing ..." << endl;
    }
    else {
        cout << "Error setting torque limits to 75" << endl;
        return 0;
    }

    uint16_t goalpos1,goalpos2;
    goalpos1 = actualpos1;
    goalpos2 = actualpos2;

    while(abs(goalpos1-actualpos1)<10)
    {
        goalpos1++;
        setPosition(id1, goalpos1, usb2dyn);
        usleep(50*1000);
        actualpos1 = readPosition(id1, usb2dyn);
    }
    setPosition(id1, actualpos1-8, usb2dyn);

    while(abs(goalpos2-actualpos2)<10)
    {
        goalpos2++;
        setPosition(id2, goalpos2, usb2dyn);
        usleep(50*1000);
        actualpos2 = readPosition(id2, usb2dyn);
    }
    setPosition(id2, actualpos2-8, usb2dyn);
    usleep(1000*1000);

    uint16_t homepos1 = readPosition(id1, usb2dyn);
    uint16_t homepos2 = readPosition(id2, usb2dyn);
    setMaxTorque(id1, 300, usb2dyn);
    setMaxTorque(id2, 300, usb2dyn);

    uint16_t currentpos1 = homepos1;
    uint16_t currentpos2 = homepos2;
    float alpha=0;
    for(i=0;i<100;i++){
        alpha = (i+1)/100.0f;
        goalpos1 = (int) (alpha*(0+homepos1)+(1-alpha)*(currentpos1));
        goalpos2 = (int) (alpha*(-125+homepos2)+(1-alpha)*(currentpos2));
        setPosition(id1, goalpos1, usb2dyn);
        setPosition(id2, goalpos2, usb2dyn);
        usleep(25*1000);
    }
    currentpos1 = readPosition(id1, usb2dyn);
    currentpos2 = readPosition(id2, usb2dyn);



    //Declare an InterfaceKit handle
    CPhidgetInterfaceKitHandle ifKit = 0;
    //create the InterfaceKit object
    CPhidgetInterfaceKit_create(&ifKit);
    CPhidget_set_OnError_Handler((CPhidgetHandle)ifKit, ErrorHandler, NULL);
    //Registers a callback that will run if the sensor value changes by more than the OnSensorChange trig-ger.
    //Requires the handle for the IntefaceKit, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
    CPhidgetInterfaceKit_set_OnSensorChange_Handler (ifKit, SensorChangeHandler, NULL);


    //open the interfacekit for device connections
    CPhidget_open((CPhidgetHandle)ifKit, -1);
    if((result = CPhidget_waitForAttachment((CPhidgetHandle)ifKit, 10000)))
    {
        CPhidget_getErrorDescription(result, &err);
        printf("Problem waiting for attachment: %s\n", err);
        return 0;
    }
    CPhidgetInterfaceKit_setRatiometric(ifKit, 0);
    //get the number of sensors available
    CPhidgetInterfaceKit_getSensorCount(ifKit, &numSensors);

    //Change the sensitivity trigger of the sensors
    for(i = 0; i < numSensors; i++)
    {
        CPhidgetInterfaceKit_setSensorChangeTrigger(ifKit, i, 0);  //we'll just use 10 for fun
    }

    for(i=0;i<10;i++){
        printf("Sensor Value: %d\n", x[0]);
        usleep(100*1000);
    }


    printf("Press any key to go exit\n");
    getchar();

    printf("Closing...\n");
    CPhidget_close((CPhidgetHandle)ifKit);
    CPhidget_delete((CPhidgetHandle)ifKit);



    return 0;
}


