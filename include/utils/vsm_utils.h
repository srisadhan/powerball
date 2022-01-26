#include <stdlib.h>
#include <iostream>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library
#include <phidget21.h>                                      // Uses phidget 21 library

#include "utils/utils.h"


extern std::vector<float> linPot; // potentiometer reading in m from the equilibrium postion

// Store the value of the potentiometer into linPot
int CCONV SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value);


// error handler for phidget
int CCONV ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown);


// Connect to phidget kit and detect if the sensor value is changed
bool phidget_enable();


// Magnet force model
float magForce(float s);


// resultant force on the object
float resForce(std::vector<float> sep);

