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
int num_H;
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

Vector<6,float> traj_gen(Vector<6,float> Qref,Vector<6,float> Q,float Timefactor, float dt, bool stop_flag)
{
  SchunkPowerball pb;
  Vector<6,float> dQ = Qref-Q;
  float maxq=0;
  for (int n=0;n<6;n++)
  {
      if (abs(dQ[n])>maxq) {maxq=abs(dQ[n]);}
  }
  float Ttravel = maxq*Timefactor;
  if (Ttravel<2.0){Ttravel=2.0;}
  cout << Ttravel <<endl;
  int itNum = Ttravel/dt;
  Vector<6,float> Qhold = Q;
  int n = 1;
  while((n<=itNum) && (!stop_flag))
  {
      Vector<6,float> Qt = (1-cos(float(n)/itNum*M_PI))/2 * dQ + Qhold;
      pb.set_pos(Qt);
      pb.update();
      Q = pb.get_pos();
      usleep( dt*1000*1000 );
      n++;
  }
  return Q;
}

Matrix<Dynamic,6,double> traj_interp(Vector<6,float> Q, Vector<6,float> Qdot, Vector<6,float> Qref,float dt, float T)
{
  Kin kin;
  std::vector<std::vector<double>>interpolated_data;
  num_H = kin.HerInter(Q,Qdot,Qref,dt,T,&interpolated_data);

  int col,row = 0;
  Matrix<Dynamic,6,double> reach_point(num_H, 6);
  for (std::vector< std::vector<double> >::const_iterator it = interpolated_data.begin(); it != interpolated_data.end(); ++ it)
  {
     col = 0;
     for (std::vector<double>::const_iterator itit = it->begin(); itit != it->end(); ++ itit)
     {
         reach_point(row,col) = *itit;
         col++;
     }
     row++;
  }
  return reach_point;
}

Vector<6,float> track_points(SchunkPowerball pb, Matrix<Dynamic,6,double> reach_point, float dt){
  // SchunkPowerball pb;
  std::chrono::time_point<std::chrono::system_clock> timeLoop;
  std::chrono::duration<float> elaps_loop;

  int i = 0;
  while(i<num_H)
  {
      timeLoop = std::chrono::system_clock::now();
      pb.set_pos(reach_point[i]);
      pb.update();

      i++;

      elaps_loop = std::chrono::system_clock::now() - timeLoop;
      if ( (dt-elaps_loop.count()) > 0 ) {
          usleep( (dt-elaps_loop.count())*1000*1000 );
      }
  }
  Vector<6,float> Q = pb.get_pos();
  return Q;
}

int main(int argc, char** argv) {
    double gPos; // outer magnet position
    if (argc>1){
      if (strcmp(argv[1],"break") == 0){
        gPos = 75e-3;
      }
    }
    else{
      gPos = 90e-3;
    }
    // Set sampling and timing options
    std::chrono::time_point<std::chrono::system_clock> timeLoop;
    std::chrono::duration<float> elaps_loop;

    int result, numSensors;
    const char *err;
    USB2Dynamixel usb2dyn(*cnfDevice, 50);
    int id1 = 1, id2 = 2;


    // homing for the gripper
    setMaxTorque(id1, 1023, usb2dyn); // outer magnets max 1023
    setMaxTorque(id2, 1023, usb2dyn); // inner magnets
    int servo_in=u1fun(30e-3), servo_out=u2fun(134e-3);

    setPosition(id1, servo_out, usb2dyn);
    setPosition(id2, servo_in, usb2dyn);
    usleep( 1*1000*1000 );

    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;
    float dt=0.005;


    // stop safety
    bool stop_flag = false;
    boost::thread stop_thread(stop,&stop_flag);

    // Powerball class
    SchunkPowerball pb;
    Vector<6,float> Q = Zeros, Qdot = Zeros;
    pb.update();
    Q = pb.get_pos();
    cout << "current Q: " <<Q[0]<<","<<Q[1]<<","<<Q[2]<<","<<Q[3]<<","<<Q[4]<<","<<Q[5]<< endl;

    // Initializing FT sensor
    // bool escFlag=false;
    // boost::thread FT_thread(TCP_receive,&escFlag);

    // Matrix<Dynamic,6,double> reach_point;
    Vector<6,float> Qref1 = makeVector(-62.802, -46.708, 55.787, .001, 76.998, 30.178)*M_PI/180;
    Vector<6,float> Qref2 = makeVector(-61.8, -49.602, 60.594, .001, 70.801, 33.993)*M_PI/180;
    // Vector<6,float> Qref3 = makeVector(-62.803, -40.408, 48.187, 1, 91.197, 30.178)*M_PI/180;
    // Vector<6,float> Qref4 = makeVector(-106.556, -32.602, 54.578, 0, 90.1, -11.253)*M_PI/180;
    // Vector<6,float> Qref5 = makeVector(-98.103, -21.405, 60, 0, 99, -4.79)*M_PI/180;
    // Vector<6,float> Qref6 = makeVector(-139.4, -33.051, 42.404, 1, 103.870, -46.797)*M_PI/180;
    // Vector<6,float> Qref7 = makeVector(-139.4, -39.057, 55.805, 1, 86.66, -46.797)*M_PI/180;

    Vector<6,float> Qref3 = makeVector(-62.843, -25.211, 39.193, 0, 114.275, 30.159)*M_PI/180;
    Vector<6,float> Qref4 = makeVector(-95.151, -3.006, 60.388, 0, 113.890, -2.845)*M_PI/180;
    Vector<6,float> Qref5 = makeVector(-81.939, 1.997, 51.382, 0, 129.086, 8.660)*M_PI/180;
    Vector<6,float> Qref6 = makeVector(-127.939, -8.802, 37.182, 2.400, 134.886, -35.839)*M_PI/180;
    Vector<6,float> Qref7 = makeVector(-124.939, -37.402, 60.681, .1, 80.086, -35.839)*M_PI/180;
    // Vector<6,float> Qref8 = makeVector(-124.939, -43.901, 60.681, .1, 80.086, -35.839)*M_PI/180;
    Vector<6,float> Qref8 = makeVector(-124.413, -43.200, 64.404, .1, 75.309, -34.839)*M_PI/180;
    Vector<6,float> Qref9 = makeVector(-127.939, -8.802, 37.182, 2.400, 134.886, -35.839)*M_PI/180;
    Vector<6,float> Qref10= Zeros;

    // Vector<6,float> dQ = Q - Qref1;

    Matrix<Dynamic,6,float>  reach_point1 = traj_interp(Q, Qdot, Qref1,dt,10);
    // Q = track_points(pb, reach_point1, dt);
    int i = 0;
    while(i<num_H)
    {
        timeLoop = std::chrono::system_clock::now();
        pb.set_pos(reach_point1[i]);
        pb.update();

        i++;

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }
    }
    Q = pb.get_pos();

    Matrix<Dynamic,6,float> reach_point2 = traj_interp(Q, Qdot, Qref2,dt,3);
    // Q = track_points(pb, reach_point2, dt);
    i = 0;
    while(i<num_H)
    {
        timeLoop = std::chrono::system_clock::now();
        pb.set_pos(reach_point2[i]);
        pb.update();

        i++;

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }
    }

    servo_in=u1fun(30e-3), servo_out=u2fun(gPos); // egg grasp stiffness
    setPosition(id1, servo_out, usb2dyn);
    setPosition(id2, servo_in, usb2dyn);
    usleep(0.1*1000*1000);

    Q = pb.get_pos();
    Matrix<Dynamic,6,float> reach_point3 = traj_interp(Q, Qdot, Qref3,dt,4);
    // usleep(1*1000*1000);
    // Q = track_points(pb, reach_point3, dt);
    cout<<"num_H:"<<num_H<<endl;
    i = 0;
    while(i<num_H)
    {
        timeLoop = std::chrono::system_clock::now();
        pb.set_pos(reach_point3[i]);
        pb.update();

        i++;

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }
    }
    Q = pb.get_pos();

    Matrix<Dynamic,6,float> reach_point4 = traj_interp(Q, Qdot, Qref4,dt,1.5);
    // usleep(1*1000*1000);
    // Q = track_points(pb, reach_point4, dt);
    i = 0;
    while(i<num_H)
    {
        timeLoop = std::chrono::system_clock::now();
        pb.set_pos(reach_point4[i]);
        pb.update();

        i++;

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }
    }
    Q = pb.get_pos();

    usleep(3*1000*1000); // collision takes place before this

    // return 0;

    SchunkPowerball pb1;
    // pb.set_control_mode(MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE);
    // pb.set_sdo_controlword(NODE_ALL,STATUS_OPERATION_ENABLED);
    // pb.get_status(1);

    Matrix<Dynamic,6,float> reach_point5 = traj_interp(Q, Qdot, Qref5,dt,3);
    // usleep(1*1000*1000);
    // Q = track_points(pb1, reach_point5, dt);
    i = 0;
    while(i<num_H)
    {
        timeLoop = std::chrono::system_clock::now();
        pb1.set_pos(reach_point5[i]);
        pb1.update();

        i++;

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }
    }
    Q = pb1.get_pos();

    Matrix<Dynamic,6,float> reach_point6 = traj_interp(Q, Qdot, Qref6,dt,3);
    // usleep(1*1000*1000);
    // Q = track_points(pb1, reach_point6, dt);
    i = 0;
    while(i<num_H)
    {
        timeLoop = std::chrono::system_clock::now();
        pb1.set_pos(reach_point6[i]);
        pb1.update();

        i++;

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }
    }
    Q = pb1.get_pos();


    Matrix<Dynamic,6,float> reach_point7 = traj_interp(Q, Qdot, Qref7,dt,3);
    // usleep(1*1000*1000);
    // Q = track_points(pb1, reach_point7, dt);
    i = 0;
    while(i<num_H)
    {
        timeLoop = std::chrono::system_clock::now();
        pb1.set_pos(reach_point7[i]);
        pb1.update();

        i++;

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }
    }
    Q = pb1.get_pos();

    Matrix<Dynamic,6,float> reach_point8 = traj_interp(Q, Qdot, Qref8,dt,3);
    i = 0;
    while(i<num_H)
    {
        timeLoop = std::chrono::system_clock::now();
        pb1.set_pos(reach_point8[i]);
        pb1.update();

        i++;

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }
    }
    Q = pb1.get_pos();

    servo_in=u1fun(30e-3), servo_out=u2fun(134e-3);
    setPosition(id1, servo_out, usb2dyn);
    setPosition(id2, servo_in, usb2dyn);

    Matrix<Dynamic,6,float> reach_point9 = traj_interp(Q, Qdot, Qref9,dt,3);
    i = 0;
    while(i<num_H)
    {
        timeLoop = std::chrono::system_clock::now();
        pb1.set_pos(reach_point9[i]);
        pb1.update();

        i++;

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }
    }
    Q = pb1.get_pos();

    Matrix<Dynamic,6,float> reach_point10 = traj_interp(Q, Qdot, Qref10,dt,3);
    i = 0;
    while(i<num_H)
    {
        timeLoop = std::chrono::system_clock::now();
        pb1.set_pos(reach_point10[i]);
        pb1.update();

        i++;

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }
    }
    Q = pb1.get_pos();

    // Open recording file
    // std::ofstream dataFile;
    // dataFile.open("collision_reaction.csv");
    // dataFile << "This is file header" << endl;
    //
    // // control loop
    // int n = 1;
    // float T=6.0f;
    // float A = -5.5f/180.0f*M_PI;
    // int sensor0 = x[1];
    // double xb0 = double(-x[0]-x[1])/2;
    // int16_t u1_a = 0;
    // int16_t u2_a = 0;


    // escFlag = true;
    stop_thread.interrupt();   // kill the threads
     // dataFile.close(); // close file
    // CPhidget_close((CPhidgetHandle)ifKit);
    // CPhidget_delete((CPhidgetHandle)ifKit);
    usleep(0.2*1000*1000);
    std::cout << "Exiting main..." << std::endl;
    return 0;

}
