#include "powerball/schunk_powerball.h"
#include "powerball/schunk_kinematics.h"
#include <fstream>
#include <chrono>
#include <istream>
#include <string>
#include <sstream>
#include <vector>
#include <iostream>
#include "vrep/v_repClass.h"
#include <boost/array.hpp>
#include "TooN/TooN.h"
#include <boost/thread.hpp>
#include <utils.h>
#include <USB2Dynamixel.h>
#include <commonOptions.h>
#include <phidget21.h>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <chrono>


using namespace::std;
using namespace::TooN;
using boost::asio::ip::tcp;


// global vars
Vector<6,float> FT;

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

int x[8];

int CCONV SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value)
{
    x[Index] = Value;
    return 0;
}

int CCONV ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
    printf("Error handled. %d - %s", ErrorCode, unknown);
    return 0; }

void TCP_receive(bool *escFlag)
{

    boost::asio::io_service io_service;
    tcp::endpoint sender_endpoint = boost::asio::ip::tcp::endpoint(
                boost::asio::ip::address::from_string("192.168.1.30"),  boost::lexical_cast<int>("1000"));
    tcp::socket socket(io_service);
    socket.connect(sender_endpoint);

    boost::system::error_code ignored_error;
    int len=0;
    char recv_buf[128];

    // TARE the sensor
    //std::string msg="TARE(1)\n";
    //socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
    //len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
    //cout << "TCP recieved: " << recv_buf << endl;

    // continous receiving
    //msg="L1()\n";
    //socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
    //len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
    //cout << "TCP recieved: " << recv_buf << endl;

    // Read one data
    std::string msg="F()\n";
    socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
    cout << "TCP wrote " << msg << endl;
    len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
    cout << "TCP recieved: " << recv_buf << endl;

    // Force data
    while(!(*escFlag))
    {
        msg="F()\n";
        socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
        len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
        int timeStamp;
        sscanf(recv_buf,"F={%f,%f,%f,%f,%f,%f},%d",&FT[0],&FT[1],&FT[2],&FT[3],&FT[4],&FT[5],&timeStamp);
        usleep(0.001*1000*1000);
    }
    cout << "Ending TCP thread" << endl;
}

double interpolate( vector<double> &xData, vector<double> &yData, double x, bool extrapolate )
{
   int size = xData.size();

   int i = 0;                                                                  // find left end of interval for interpolation
   if ( x >= xData[size - 2] )                                                 // special case: beyond right end
   {
      i = size - 2;
   }
   else
   {
      while ( x > xData[i+1] ) i++;
   }
   double xL = xData[i], yL = yData[i], xR = xData[i+1], yR = yData[i+1];      // points on either side (unless beyond ends)
   if ( !extrapolate )                                                         // if beyond ends of array and not extrapolating
   {
      if ( x < xL ) yR = yL;
      if ( x > xR ) yL = yR;
   }

   double dydx = ( yR - yL ) / ( xR - xL );                                    // gradient

   return yL + dydx * ( x - xL );                                              // linear interpolation
}

// constants
const double P1 = -5.3429e-05;
const double resolution = 300*M_PI*25.46e-3/180/1023;
int u1fun(double u)
{
    return round(490+(u-13.5e-3)/resolution); //amir's setting - 13.5 mm is the minimum object width the gripper can hold
}
int u2fun(double u)
{
    int res = round(954+(u-134e-3)/resolution); //int res = round(954+(u-134e-3)/resolution); // amir's setting
    if (res>1023) {res=1023;}                   // 134 mm is the maximum object width the gripper can hold
    return res;
}


int main() {

    // Set sampling and timing options
    std::chrono::time_point<std::chrono::system_clock> timeLoop;
    std::chrono::duration<float> elaps_loop;

    int result, numSensors;
    const char *err;
    USB2Dynamixel usb2dyn(*cnfDevice, 50);
    int id1 = 1, id2 = 2;

    float scale_out = 30e-3;
    // homing for the gripper
    setMaxTorque(id1, 1023, usb2dyn); // outer magnets max 1023
    setMaxTorque(id2, 1023, usb2dyn); // inner magnets
    // int servo_in=u1fun(0e-3), servo_out=u2fun(127e-3-scale_out);
    // int servo_in=u1fun(20e-3), servo_out=u2fun(120e-3);
    int servo_in=u1fun(0e-3), servo_out=u2fun(127e-3);
    // cout << servo_in << endl;
    // cout << servo_out << endl;
    // usleep(4*1000*1000);
    setPosition(id1, servo_out, usb2dyn);
    setPosition(id2, servo_in, usb2dyn);
    usleep( 1*1000*1000 );

    // uint16_t pos1 = readPosition(id1, usb2dyn);
    // uint16_t pos2 = readPosition(id2, usb2dyn);
    // cout << pos1 << " , " << pos2 << endl;
    // return 0;

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
    for(int i = 0; i < numSensors; i++)
    {
        CPhidgetInterfaceKit_setSensorChangeTrigger(ifKit, i, 0);  //we'll just use 10 for fun
    }

    cout << "dynamixel and phidget are joined!" << endl;
    // load input file to a matrix
    Kin kin;
    std::vector< std::vector<double> > matrix;
    if (!(kin.inputFile("offline_a0.95.txt",&matrix)))
    {
        return 0;
    }
    TooN::Matrix<Dynamic,Dynamic,double> traj(kin.nrows, kin.ncols);
    cout << "size of matrix = " << kin.nrows << "*" << kin.ncols << endl;
    int  col=0; int row=0;
    for (std::vector< std::vector<double> >::const_iterator it = matrix.begin(); it != matrix.end(); ++ it)
    {
        col = 0;
        for (std::vector<double>::const_iterator itit = it->begin(); itit != it->end(); ++ itit)
        {
            traj(row,col) = *itit;
            col++;
        }
        row++;
    }

    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;
    float dt=0.010;


    // stop safety
    bool stop_flag = false;
    boost::thread stop_thread(stop,&stop_flag);


    // Interpolation data
    const int NPTS = 501;
    vector<double> xb, u1,u2;
    for ( int i = NPTS-1; i >= 0; i-- )
    {
        xb.push_back( double(traj(i,0)) );
        u1.push_back( double(traj(i,1)) );
        u2.push_back( double(traj(i,2)) );
    }



    // Powerball class
    SchunkPowerball pb;
    Vector<6,float> Q = Zeros, Qdot = Zeros;
    pb.update();
    Q = pb.get_pos();
    cout << "current Q: " <<Q[0]<<","<<Q[1]<<","<<Q[2]<<","<<Q[3]<<","<<Q[4]<<","<<Q[5]<< endl;

    // Initializing FT sensor
    bool escFlag=false;
    boost::thread FT_thread(TCP_receive,&escFlag);

    // Going to home pose
    //Vector<6,float> Qref=makeVector(0.8272,-0.5828,2.4172,0.0907,-1.0,-0.0031); //(0,-0.569414,1.80612,0,0.765379,-1.58488);
    //Vector<6,float> Qref=makeVector(0.69794,-0.778277,1.48343,0,0.139,-0.00136136); Amir's position
    Vector<6,float> Qref=makeVector(0,-0.523599,1.570709,0,1.0472,0);

    Vector<6,float> dQ = Qref-Q;
    float maxq=0;
    for (int n=0;n<6;n++)
    {
        if (abs(dQ[n])>maxq) {maxq=abs(dQ[n]);}
    }
    float Ttravel = maxq*7.0f;
    if (Ttravel<2.0){Ttravel=2.0;}
    cout << Ttravel <<endl;
    int itNum = Ttravel/dt;
    Vector<6,float> Qhold = Q;
    int n = 1;
    while((n<=itNum) && (!stop_flag))
    {
        Vector<6,float> Qt = (1-cos(float(n)/itNum*M_PI))/2 * dQ + Qhold;

        //Vector<6,float> Qt = Qhold;
        //Qt[0] = Qhold[0]+n*0.0005f;
        //cout << "command Q: " <<Qt[0]<<","<<Qt[1]<<","<<Qt[2]<<","<<Qt[3]<<","<<Qt[4]<<","<<Qt[5]<< endl;
        pb.set_pos(Qt);
        pb.update();
        Q = pb.get_pos();
        usleep( dt*1000*1000 );
        n++;
    }

    // Open recording file
    std::ofstream dataFile;
    dataFile.open("collision_reaction.csv");
    dataFile << "This is file header" << endl;

    // control loop
    n = 1;
    float T=6.0f;
    float A = -3*5.5f/180.0f*M_PI;
    int sensor0 = x[1];
    double xb0 = double(-x[0]-x[1])/2;
    int16_t u1_a = 0;
    int16_t u2_a = 0;
    while((n*dt<=T) && (!stop_flag))
    {
        timeLoop = std::chrono::system_clock::now();
        double sensor_BU = P1* (double(-x[0]-x[1])/2 - xb0);
        double sensor = P1* (double(-x[0]-x[1])/2 - xb0);                               // comment for stiff actuation
       // double sensor = 0;                                                            // comment for adjustable stiffness
        double u1_d = interpolate( xb, u1, sensor, false )+0e-3;
        double u2_d = interpolate( xb, u2, sensor, false );//-scale_out;
        //cout << sensor << " , " << u1_d << " , " << u2_d << endl;

        // if (n % 2 == 0){
        //     setPosition(id1, u2fun(u2_d), usb2dyn);
        //     setPosition(id2, u1fun(u1_d), usb2dyn);
        //     cout << sensor << " , " << u1_d << " , " << u2_d << endl;
        // }

        Vector<6,float> Qt = Zeros;
        Qt = Qref;
        float th = float(n)*dt/T*2.0f*M_PI;
        Qt[0] = (1-cos(th))/2.0f * A + Qref[0];


        pb.set_pos(Qt);

        pb.update();
        n++;

        dataFile<< sensor_BU << "," << u1_d << "," << u1_a <<","<< u2_d << "," << u2_a << ","  << FT[0]<<","<<FT[1]<<","<<FT[2]<<","<<FT[3]<<","<<FT[4]<<","<<FT[5]<< ","
                    <<x[0] <<","<< x[1] << endl;


        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }
    }


    escFlag = true;
    stop_thread.interrupt();   // kill the threads
     dataFile.close(); // close file
    CPhidget_close((CPhidgetHandle)ifKit);
    CPhidget_delete((CPhidgetHandle)ifKit);
    usleep(0.2*1000*1000);
    std::cout << "Exiting main..." << std::endl;
    return 0;

}
