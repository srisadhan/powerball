#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <ostream> // included for color output to the terminal

// boost headers
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>

// schunk powerball headers
#include "powerball/schunk_powerball.h"
#include "vrep/v_repClass.h"
#include "powerball/schunk_kinematics.h"

// Toon headers
#include <TooN/TooN.h>
#include <TooN/LU.h>
#include <TooN/SVD.h>

// myo band headers
#include "myolinux/myoclient.h"
#include "myolinux/serial.h"

// dynamixel headers
#include <utils.h>
#include <USB2Dynamixel.h>
#include <commonOptions.h>

// phidget headers
#include <phidget21.h>

// plotting and linear algebra libraries
#include "matplotlibcpp.h"
#include "sigpack.h"
#include <armadillo>


using namespace::std;
using namespace::TooN;
using boost::asio::ip::tcp;


// global vars
Vector<6,float> FT;
const double resolution = 300*M_PI*25.46e-3/180/1024;
const double magWidth = 0.022; // width of the magnet holder
const double forceThres = 2*1.5; // force threshold on the object
const double dt = 0.005; // sampling time
double K_gain = 200; // gain for tuning the magnet motion

void stop(bool* flag){
    char in;
    cin.get(in);
    *flag = true;
}


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
    double potOff0 = 0.004; // 0.002 - offset in the potentiometer reading
    double potOff1 = 0.006; // potentiometer is placed at on offset on both the sides

    if(Index == 0){
      linPot[Index] = (60-Value*60/1023)/1e3 + potOff0;
    }else if (Index == 1){
      linPot[Index] = (Value*60/1023)/1e3 + potOff1;
    }
    // cout << linPot << endl;
    return 0;
}

int CCONV ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
    printf("Error handled. %d - %s", ErrorCode, unknown);
    return 0;
}

int GripperCalibration(int id1, int id2, USB2Dynamixel &usb2Dynamixel){
  // id1 is for outer magnets and id2 for inner magnets
  int pos1i = 1020;
  int pos2i = 500;// finger movement should be restricted beyond this point
  int pos1f = 640;
  int pos2f = 880;

  int pot1  = linPot[0];
  int pot2  = linPot[1];

  // Start with this position - low stiffness
  setPosition(id1, pos1i, usb2Dynamixel);
  setPosition(id2, pos2i, usb2Dynamixel);

  float dynParam = sqrt(pow(pos1f-pos1i,2)+pow(pos2f-pos2i,2));
  // Keep increasing the stiffness
  while( pos2f > pos2i && pos1f < pos1i && dynParam > 10){
    pos1i = pos1i - 10;
    pos2i = pos2i + 10;
    setPosition(id1, pos1i, usb2Dynamixel);
    setPosition(id2, pos2i, usb2Dynamixel);
    usleep(0.1*1e6);
    float dynParam = sqrt(pow(pos1f-pos1i,2)+pow(pos2f-pos2i,2));
    // cout << "Dyn1: " << pos1i << " , " << "Dyn2:" << pos2i << " , " << "Error: " << dynParam<<endl;
  }

  return 0;
}


// Magnet force model
double magForce(double s){
  double a = 4.8e-6;
  double b = 5.3e-5;
  double c = 2.3e-7;
  return a/(pow(s,3)+b*s+c);
}

// resultant force on the object
double resForce(USB2Dynamixel &usb2Dynamixel, double finger1, double finger2){

  double dyn1pos = 0.1425 - (1023 - readPosition(1,usb2Dynamixel))*resolution;
  return magForce(dyn1pos/2 - (finger1+ magWidth/2)) - magForce(dyn1pos/2 - (finger2+ magWidth/2)) ;
}

// Calculating dynamixel position asjustment for the desired magnet position using control algorithm
double interpMag(USB2Dynamixel &usb2Dynamixel, double pos, int initDyn){
  int Dyn = round((readPosition(1,usb2Dynamixel)*resolution + pos)/resolution);
  if (Dyn > 1023){
    Dyn = 1023;
  }
  else if (Dyn < initDyn){
    Dyn = initDyn - 10;
  }
  return Dyn;
}


// Magnet Position control based on the force threshold on the object
void magnetControl (USB2Dynamixel &usb2Dynamixel, double finger1, double finger2, int initDyn, bool* stopFlag){
  if(!*stopFlag){
    double objForce = abs(resForce(usb2Dynamixel,finger1,finger2));
    if(abs(objForce - forceThres) > 0.5){
      double x = K_gain*(objForce-forceThres)*pow(dt,2);
      cout << "Set this position to the dynamixel: " << interpMag(usb2Dynamixel, x, initDyn) <<endl;
      setPosition(1, interpMag(usb2Dynamixel, x, initDyn), usb2Dynamixel);
    }
  }else{
    cout << "Could not control the magnet position, exiting the control loop"<<endl;
  }
}


// ---------------------------- Main function ------------------------------
int main(int argc, char** argv){

  bool stiffVar = false; // flag for variable stiffness

  if (argc>1){
    if (strcmp(argv[1],"var") == 0){
      stiffVar = true;
    }
  }

  // Initializing dynamixel
  USB2Dynamixel usb2dyn(*cnfDevice, 50);
  int id1 = 1, id2 = 2;
  setMaxTorque(id1, 1023, usb2dyn); // outer magnets max 1023
  setMaxTorque(id2, 1023, usb2dyn); // inner magnets

  // int dyn1pos = 850; // for spaghetti
  int dyn1pos = 650; // for egg or orange

  setPosition(id1, dyn1pos, usb2dyn);
  setPosition(id2, 500, usb2dyn);
  usleep( 1*1000*1000 );

  //Initializing phidget
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
      printf("Problem connecting to phidget: %s\n", err);
      return 0;
  }
  CPhidgetInterfaceKit_setRatiometric(ifKit, 0);
  CPhidgetInterfaceKit_getSensorCount(ifKit, &numSensors);
  for(int i = 0; i < numSensors; i++)
  {
      CPhidgetInterfaceKit_setSensorChangeTrigger(ifKit, i, 0);  //we'll just use 10 for fun
  }

  cout << "dynamixel and phidget are joined!" << endl;

  // Stop thread
  bool stopFlag = false;
  boost::thread stop_thread(stop,&stopFlag);

  // GripperCalibration(id1, id2, usb2dyn);

  int count = 0;
  while(count < 1000 && !stopFlag){
    usleep(0.01*1e6);
    // boost::thread dyn_thread(magnetControl,usb2dyn,linPot[0],linPot[1], dyn1pos, &stopFlag);
    // dyn_thread.join();
    if (stiffVar){
      magnetControl(usb2dyn,linPot[0],linPot[1],dyn1pos, &stopFlag);
    }
    // cout << "Sensor 1: "<< linPot[0] << " , " << "Sensor 2: "<< linPot[1] << " , " << "Object Length: " << linPot[0]+linPot[1]-magWidth << endl;
    count += 1;
    // cout << "Force on the object: "<< resForce(usb2dyn,linPot[0],linPot[1])<< endl;
  }

  stop_thread.interrupt();
  CPhidget_close((CPhidgetHandle)ifKit);
  CPhidget_delete((CPhidgetHandle)ifKit);
  return 0;
}
