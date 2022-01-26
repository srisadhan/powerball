#include "utils/vsm_utils.h"



using namespace::std;

std::vector<float> linPot{0,0};

// Store the value of the potentiometer into linPot
int CCONV SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value)
{
    vector<float> potCenter = {560, 554}; // get the center position of the handle during calibration
    
    if (Index < 2)
    {
        linPot[Index] = (Value - potCenter[Index])*100/1000/1e3; // VSM softpot strips are 10 cms long and their bit values are in the range [1, 1000]
        // printf("Index: %d, Pot value: %d, distance from center: %f \n", Index, Value, linPot[Index]);
    }
    return 0;
}

// error handler for phidget
int CCONV ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
    printf("Error handled. %d - %s", ErrorCode, unknown);
    return 0;
}

// Connect to phidget kit and detect if the sensor value is changed
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
    
    // change the sensor change trigger for the connected potentiometers
    for(int Index = 0; Index < 2; Index++)
    {
        CPhidgetInterfaceKit_setSensorChangeTrigger(ifKit, Index, 0);  //0 represents higher sensitivity
    }
    

    printf(GREEN "Successfully connected to Phidget with %d sensors \n" DEFAULT, numSensors);
    return 1;
}    

// Magnet force model
float magForce(float s)
{
      float C1 = 27.34; // small magnet model
      float C2 = 168.15;
    return C1 * exp(-C2 * s);
}

// resultant force on the object
float resForce(std::vector<float> sep)
{
    if (sep[0] < 0) sep[0] = 0.0;
    if (sep[1] < 0) sep[1] = 0.0;

    // printf("%f, %f, %f \n", pos1, pos2, handle_position);
    // printf("Sep1: %f, Sep2: %f \n", sep1, sep2);
    return magForce(sep[0]) - magForce(sep[1]);
}

