#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <ostream> // included for color output to the terminal

// schunk powerball headers
#include "powerball/schunk_powerball.h"
#include "vrep/v_repClass.h"
#include "powerball/schunk_kinematics.h"

// Toon headers
#include <TooN/TooN.h>
#include <TooN/LU.h>
#include <TooN/SVD.h>


// dynamixel headers
#include <utils.h>
#include <USB2Dynamixel.h>
#include <commonOptions.h>

// phidget headers
#include <phidget21.h>

//the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET       "\033[0m"
#define RED         "\033[31m"              /* Red */
#define GREEN       "\033[32m"              /* Green */
#define BOLDRED     "\033[1m\033[31m"       /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"       /* Bold Green */

// Global variables
const double resolution = 300*M_PI*25.46e-3/2/180/1024;
const double magWidth = 0.022; // width of the magnet holder

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


double linPot[2];

int CCONV SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value)
{
    vector<double> potOff = {160, 120};
    linPot[Index] = ((Value - potOff[Index]) * 100 / 1024) / 1e3;

    cout << linPot[0] << "," << linPot[1] << endl;
    return 0;
}

int CCONV ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
    printf("Error handled. %d - %s", ErrorCode, unknown);
    return 0;
}

void CloseGripper(int id1, int id2, USB2Dynamixel &usb2Dynamixel){
  
  // Start with this position - low stiffness
  setPosition(id1, 600, usb2Dynamixel);
  setPosition(id2, 0, usb2Dynamixel); 
  usleep(0.01*1e6);
}

void OpenGripper(int id1, int id2, USB2Dynamixel &usb2Dynamixel){
  
  // Start with this position - low stiffness
  setPosition(id1, 1023, usb2Dynamixel);
  setPosition(id2, 800, usb2Dynamixel); 
  usleep(0.01*1e6);
}

void ControlMag(int id, int target_pos, USB2Dynamixel &usb2Dynamixel){
    // Start with this position - low stiffness
    setPosition(id, target_pos, usb2Dynamixel);

    usleep(0.01*1e6);
}

double OuterMagSep(int pos_bit){
    double pos = (pos_bit - 250) * resolution + (.023 - 0.01615); // 23 mm offset at 250 bit encoder position minus 16.15 mm finger width (zero position of outer magnet or finger)
    return pos;
}

double InnerMagSep(int pos_bit){
    double pos = (pos_bit - 315) * resolution; // 5.85 mm offset at 315 bit encoder position (zero position of inner magnet or finger)
    return pos;
}

// Magnet force model
double magForce(double s){
  double a = 5.5e-6;
  double b = 6.85e-5;
  double c = 2.22e-7;
  return a/(pow(s,3)+b*s+c);
}

int main(){

    // Initializing dynamixel
    USB2Dynamixel usb2dyn(*cnfDevice, 4);
    int id1 = 1, id2 = 2;
    setMaxTorque(id1, 1023, usb2dyn); // outer magnets max 1023
    setMaxTorque(id2, 1023, usb2dyn); // inner magnets

    //Initializing phidget
    int result, numSensors;
    const char *err;
    CPhidgetInterfaceKitHandle ifKit = 0;
    CPhidgetInterfaceKit_create(&ifKit);
    CPhidget_set_OnError_Handler((CPhidgetHandle)ifKit, ErrorHandler, NULL);
    CPhidgetInterfaceKit_set_OnSensorChange_Handler(ifKit, SensorChangeHandler, NULL);
    CPhidget_open((CPhidgetHandle)ifKit, -1);
    if((result = CPhidget_waitForAttachment((CPhidgetHandle)ifKit, 10000)))
    {
        CPhidget_getErrorDescription(result, &err);
        printf("Problem connecting to phidget: %s\n", err);
        return 0;
    }
    CPhidgetInterfaceKit_setRatiometric(ifKit, 0);
    CPhidgetInterfaceKit_getSensorCount(ifKit, &numSensors);
    for(int sensor = 0; sensor < numSensors; sensor++)
    {
        CPhidgetInterfaceKit_setSensorChangeTrigger(ifKit, sensor, 0); // trigger is 0
    }
    
    while(1) {
        usleep(0.05 * 1e6);
    }
    OpenGripper(id1, id2, usb2dyn);

    usleep(2*1e6);

    cout << "---------" << readPosition(id1,usb2dyn) << "," << readPosition(id2,usb2dyn) << endl; 
    cout << "Out1: " << OuterMagSep(1023) << ", In1: " << InnerMagSep(800) << ", Finger1:"<< linPot[0] << "\n";
    cout << "Out2: " << OuterMagSep(1023) << ", In2: " << InnerMagSep(800) << ", Finger2:"<< linPot[1] << "\n";
    
    int target_pos = 1023;
       
    cout << "Force Finger 1:" << magForce(OuterMagSep(1023) - linPot[0]) << " ,"<< magForce(linPot[0] - InnerMagSep(800)) << endl;
    cout << "Force Finger 2:" << magForce(OuterMagSep(1023) - linPot[1])  << " ,"<< magForce(linPot[1] - InnerMagSep(800)) << endl;


    CloseGripper(id1, id2, usb2dyn);

    usleep(2*1e6);
    cout << "---------" << readPosition(id1,usb2dyn) << "," << readPosition(id2,usb2dyn) << endl; 
    cout << "Out1: " << OuterMagSep(600) << ", In1: " << InnerMagSep(0) << ", Finger1: "<< linPot[0] << "\n";
    cout << "Out2: " << OuterMagSep(600) << ", In2: " << InnerMagSep(0) << ", Finger2: "<< linPot[1] << "\n";
    
    
    cout << "Force Finger 1:" << magForce(OuterMagSep(600) - linPot[0]) << " ,"<< magForce(linPot[0] - InnerMagSep(0)) << endl;
    cout << "Force Finger 2:" << magForce(OuterMagSep(600) - linPot[1])  << " ,"<< magForce(linPot[1] - InnerMagSep(0)) << endl;
    
    return 0;
}