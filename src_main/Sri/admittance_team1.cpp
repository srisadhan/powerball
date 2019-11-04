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
#include <TooN/LU.h>
#include <TooN/SVD.h>

// myo band headers
#include "myolinux/myoclient.h"
#include "myolinux/serial.h"

// dynamixel headers
#include <utils.h>
#include <USB2Dynamixel.h>
#include <commonOptions.h>

// plotting and linear algebra libraries
#include "matplotlibcpp.h"
#include "sigpack.h"
#include <armadillo>

using namespace::std;
using boost::asio::ip::tcp;
using namespace::TooN;
using namespace myolinux;
namespace plt = matplotlibcpp;
// using namespace std::chrono;
namespace
{
  commonOptions::Option<std::vector<std::string>> cnfDevice("usb2dyn.device", {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"}, "device for usb2BaccaratDealer");
}
// code for color display in the terminal
namespace Color {
    enum Code {
        FG_RED      = 31,
        FG_GREEN    = 32,
        FG_BLUE     = 34,
        FG_DEFAULT  = 39,
        BG_RED      = 41,
        BG_GREEN    = 42,
        BG_BLUE     = 44,
        BG_DEFAULT  = 49
    };
    class Modifier {
        Code code;
    public:
        Modifier(Code pCode) : code(pCode) {}
        friend std::ostream&
        operator<<(std::ostream& os, const Modifier& mod) {
            return os << "\033[" << mod.code << "m";
        }
    };
}

Color::Modifier red(Color::FG_RED);
Color::Modifier green(Color::FG_GREEN);
Color::Modifier def(Color::FG_DEFAULT);

// Initializing the myo connection
myo::Client client(Serial{"/dev/ttyACM0", 115200});


//Global variables
float dt = 0.005f; // sampling time

Vector<8,float> EMG = Zeros;
Vector<4,float> ORI = Zeros;
Vector<3,float> ACC = Zeros;
Vector<3,float> GYR = Zeros;
Vector<6,float> FT  = Zeros;
Vector<6,float> Q   = Zeros;
Vector<6,float> Qe  = Zeros;
Vector<3,float> X   = Zeros;
Vector<3,float> X_init = Zeros;
Vector<6,float> Qdot = Zeros;
Vector<6,float> Qdot_a = Zeros;
Vector<6,float> joints_vrep = Zeros;
simxFloat newPos[3] ={-0.125f,0.439,0.275};
bool Start_record   = false;
// arma::mat emgVec(10000,8,arma::fill::zeros);
int counter = 0;
int windLen = 50;
string filename = "";

// Cartesian admittance parameters_ one time define
Vector<6,float> Md_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*0.1f; // for constant m (low multiplier (0.1f) - high mass, high multiplier (0.3f)- low mass)
//Vector<6,float> Md_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*(1.0f/15.0f); // for var m (min:3.5 max:)
Matrix<6,6,double> Md_inv = Md_diag.as_diagonal();

//Vector<6,float> Cd_diag = makeVector(1.2f,1.0f,1.0f,1.0f,1.0f,1.0f)*80; // for fine Low
// Vector<6,float> Cd_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*25; // for fine High
Vector<6,float> Cd_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*100; // for gross Low (multiplier (90) - low damping; multiplier (190) - high damping)

// Increase the Mass and damping in the z direction for the VSM

Matrix<6,6,double> Cd = Cd_diag.as_diagonal();
Vector<6,float> vel = Zeros;

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

// stops the program
void stop(bool* flag){
    char in;
    cin.get(in);
    *flag = true;
    Start_record = false;
}

// Vrep function
int vrep_draw(bool* flag){
    // connect to vrep
    int res = -1;
    if (! *flag){
      V_rep vrep;
      res = vrep.connect();
      if (res==-1)
      {
          cout << "V-REP Connection Error!" << endl;
          return 0;
      }

      while(!*flag)
      {
          vrep.setSphere(&newPos[0]);
          // vrep.setq(joints_vrep);
          usleep(40*1000);
      }
    }
}

// The computations for admittance control are carried on here
void computations(){
    Matrix<6,6,float> J = Zeros;
    Matrix<6,6,float> Rmat = Zeros;  // 6 by 6 matrix with R partitioned
    Matrix<3,3,float> R = Zeros;
    // Vector<3,float> X = Zeros;
    Vector<6,float> F_modified = Zeros;

    Kin kin;

    kin.Jacob(Q,&J);
    kin.FK_R(Q,&R);
    kin.FK_pos(Q,&X);

    // newPos[0] = -(X[1]-X_init[1])+0.06; // peg-hole experiment
    // newPos[1] = X[0]-X_init[0];
    // newPos[2] = 0.01;//X[2];

    // newPos[1] = (X[1]-X_init[1]); // for line traversal eperiment
    // newPos[0] = X[0]-X_init[0]+0.06;
    // newPos[2] = 0.01;//X[2];

    // Team 1 project setPosition
    // newPos[0] = -0.425;//-(X[1]-X_init[1]); // for line traversal eperiment
    // newPos[1] = -(X[1]-X_init[1]+0.8);
    // newPos[2] = X[2]-X_init[2]+0.6;

    // newPos[0] = -0.425;//; // for complex factory
    // newPos[1] = -1.5*(X[1]-X_init[1])-0.8;
    // newPos[2] = -1.5*(X[0]-X_init[0])+0.6;

    newPos[0] = -(X[1]-X_init[1]) - 0.125; // for maze traversal
    newPos[1] = X[0]-X_init[0]+0.465;
    newPos[2] = 0.01;//X[2];


    Rmat.slice<0,0,3,3>() = R.slice<0,0,3,3>();
    Rmat.slice<3,3,3,3>() = R.slice<0,0,3,3>();

    // Rotation about z axis is prevented - Modify according to the force sensor is attached to the end-effector
    // F_modified[0] = 0;//FT[0];//-FT[2];
    // F_modified[1] = FT[1];
    // F_modified[2] = FT[2];

    // Rotation about z axis is prevented - Modify according to the force sensor is attached to the end-effector
    F_modified[0] = FT[0];//FT[0];//-FT[2];
    F_modified[1] = FT[1];
    F_modified[2] = 0;

    float alpha = 1;
    vel = vel + dt*(Md_inv*(Rmat*(F_modified)) - Md_inv*alpha*Cd*vel);

    // solve inv(A)*b using LU
    SVD<6,6,float> luJ(J);
    Qdot = luJ.backsub(vel);

}

void TCP_receive(bool *errFlag)
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
    std::string msg="TARE(1)\n";
    socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
    len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
    cout << "TCP recieved: " << recv_buf << endl;

    // continous receiving
    msg="L1()\n";
    socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
    len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
    cout << "TCP recieved: " << recv_buf << endl;

    // Force data
    for (;;)
    {
        len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
        int timeStamp;
        sscanf(recv_buf,"F={%f,%f,%f,%f,%f,%f},%d",&FT[0],&FT[1],&FT[2],&FT[3],&FT[4],&FT[5],&timeStamp);
    }
}

// Myo armband files
myo::Client Myo_init()
{
    client.connect();// Autoconnect to the first Myo device
    if (!client.connected()) {
        cout<< red << "Unable to connect to Myo band"<<def<<endl;
    }else{
        cout<< green <<"Connection established with the Myo band .........."<<def<<endl;
    }

    client.setSleepMode(myo::SleepMode::NeverSleep);// Set sleep mode
    client.setMode(myo::EmgMode::SendEmg, myo::ImuMode::SendData, myo::ClassifierMode::Disabled);// Read EMG and IMU
    client.onEmg([](myo::EmgSample sample)
    {
        for (std::size_t i = 0; i < 8; i++) {
            EMG[i] = static_cast<int>(sample[i]);
        }
    });

    client.onImu([](myo::OrientationSample ori, myo::AccelerometerSample acc, myo::GyroscopeSample gyr)
    {
        for (size_t i = 0; i < 4 ; i++){
            ORI[i] = ori[i];
            if (i < 3){
                ACC[i] = acc[i];
                GYR[i] = gyr[i];
            }
        }
    });
    // auto name = client.deviceName();
    return client;
}

void Myo_receive(bool *errFlag)
{
    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;

    // Open recording file
    std::ofstream EMGFile, IMUFile;
    EMGFile.open("HRI_project/Team1/"+filename+"_EMG.csv");
    EMGFile << "Time,EMG1,EMG2,EMG3,EMG4,EMG5,EMG6,EMG7,EMG8"<< endl;

    IMUFile.open("HRI_project/Team1/"+filename+"_IMU.csv");
    IMUFile << "Time,ORI1,ORI2,ORI3,ORI4,ACC1,ACC2,ACC3,GYR1,GYR2,GYR3"<< endl;
    while(!*errFlag){
        try {
            client.listen();
            if(Start_record){
              gettimeofday(&tv, NULL);
              curtime=tv.tv_sec;
              strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));

              EMGFile << buffer << ":" << tv.tv_usec << ",";
              IMUFile << buffer << ":" << tv.tv_usec << ",";

              // for(int i = 0; i < 8;i++){
              //   emgVec(counter,i)= abs(EMG[i]);
              // }
              counter += 1;

              EMGFile<< EMG[0]<< "," << EMG[1]<< "," << EMG[2]<< "," << EMG[3]<< "," << EMG[4]<< "," << EMG[5]<< "," << EMG[6]<< "," << EMG[7]<< endl;
              IMUFile<< ORI[0]<< "," << ORI[1]<< "," << ORI[2]<< "," << ORI[3]<< "," << ACC[0]<< "," << ACC[1]<< "," << ACC[2]<< "," << GYR[0]<< "," << GYR[1]<< "," << GYR[2] <<endl;
            }
        }
        catch(myo::DisconnectedException &) {
            cout << "MYO Disconnected" << endl;
        }
    }
}

void EMG_calib(bool *errFlag)
{
    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;

    // Open recording file
    std::ofstream EMGFile, IMUFile;
    EMGFile.open("HRI_project/Team1/Calib.csv");
    EMGFile << "Time,EMG1,EMG2,EMG3,EMG4,EMG5,EMG6,EMG7,EMG8"<< endl;

    while(!*errFlag){
        try {
            client.listen();
            gettimeofday(&tv, NULL);
            curtime=tv.tv_sec;
            strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));

            EMGFile << buffer << ":" << tv.tv_usec << ",";
            EMGFile<< EMG[0]<<","<<EMG[1]<< "," << EMG[2]<< "," << EMG[3]<< "," << EMG[4]<< "," << EMG[5]<< "," << EMG[6]<< "," << EMG[7]<<endl;
        }
        catch(myo::DisconnectedException &) {
            cout << "MYO Disconnected" << endl;
        }
    }
}


// ----------------------------Main function------------------------------
int main(int argc, char** argv)
{
  bool vrepFlag = true;
  // arg to pass to the code is in the format "sudo ./admittance_3d_velMode vrep arg2 arg3 ..."
  if (argc>1){
    if (strcmp(argv[1],"vrep") == 0){
      vrepFlag = false;
    }
    if (argc > 2){
      filename =  argv[2];
    }
  }

  // timestamp vars
  char buffer[10];
  struct timeval tv;
  time_t curtime;
  std::chrono::time_point<std::chrono::system_clock> timeLoop;
  std::chrono::duration<float> elaps_loop;

  // Initialize Myo band
  myo::Client client = Myo_init(); // initializing the myo band here works (sometimes it works here and sometimes after initializing the admittance control thread).

  #if 0
  // Muscle activity calibration using the MYO armband
  bool Calib_errFlag = false;
  boost::thread Calib_thread(EMG_calib,&Calib_errFlag);
  cout<< red <<"Relax your muscles while maintaining the hand configuration.."<<def<<endl;
  usleep(5*1000*1000);
  cout<< green <<"Get ready to hold the handle firmly"<<def<<endl;
  usleep(1*1000*1000);
  cout<< red <<"Now practice high grasp------------"<< def<< endl;
  usleep(5*1000*1000);
  cout<<"Calibration completed, you can relax now"<<endl;
  Calib_errFlag = true;
  Calib_thread.interrupt();
  #endif

  // Myo thread for the actual experiment
  bool Myo_errFlag = false;
  boost::thread Myo_thread(Myo_receive,&Myo_errFlag);

  // connect to V-rep
  boost::thread vrep_thread(vrep_draw,&vrepFlag);


  // connect to robot
  SchunkPowerball pb;
  pb.update();
  Q = pb.get_pos();

  // stop by Enter key thread
  bool stop_flag = false;
  boost::thread stop_thread(stop,&stop_flag);


  // go to start pose
  // Qe = Data(0.3788f,-0.5109f,1.6861f,0.0126f,0.9335f,0.3827f);   // 2
  // Qe = Data(0.0262f,-0.7727f,1.9269f,-0.0049f,-1.1236f,0.0477f); // 1
  // Qe = Data(0.4141f,-0.699f,1.123f,0.0146f,1.3082f,0.4149f); // complex maze
  Qe = Data(0.0f,-0.2623f,1.4834f,0.0f,1.4363f,0.0f);
  // Qe = Data(0.4f,-0.465f,1.2024f,0.014f,1.5115f,0.4014f);

  Vector<6,float> dQ = Qe-Q;
  float maxq=0;
  for (int n=0;n<6;n++)
  {
      if (abs(dQ[n])>maxq) {maxq=abs(dQ[n]);}
  }
  float Ttravel = maxq*5;
  if (Ttravel<1.0){Ttravel=1.0;}
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

  // Get current pose of the robot
  Kin kin;
  kin.FK_pos(Q,&X_init);
  // set velocity mode active
  pb.set_control_mode(MODES_OF_OPERATION_VELOCITY_MODE);
  pb.update();

  // Open recording file
  std::ofstream dataFile;
  dataFile.open("HRI_project/Team1/"+filename+".csv");
  dataFile << "Time,FT1,FT2,X,Y,Z" << endl;

  // Initializing FT sensor
  bool errFlag=false;
  boost::thread FT_thread(TCP_receive,&errFlag);

  cout << "Admittance loop started!" << endl;
  cout<< green << "Start the experiment "<< def <<endl;


  while(!stop_flag)
  {

    timeLoop = std::chrono::system_clock::now();
    boost::thread threaded_computation(computations);

    // velocity saturation
    if (TooN::norm_inf(Qdot)>32.5*M_PI/180)
    {
        cout << "saturation!" << endl;
        stop_flag = true; // Emergency stop
    }
    else{pb.set_vel(Qdot);}

    pb.update();
    Q = pb.get_pos();
    Qdot_a = pb.get_vel();

    Start_record = true;
    // timestamp
    gettimeofday(&tv, NULL);
    curtime=tv.tv_sec;
    strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));
    dataFile << buffer << ":" << tv.tv_usec << ",";

    dataFile<<FT[0]<<","<<FT[1]<< "," <<newPos[0]<<","<< newPos[1]<<","<<newPos[2]<<endl;
    // cout<< newPos[0]<<","<< newPos[1]<<","<<newPos[2]<<endl;
    threaded_computation.join();

    elaps_loop = std::chrono::system_clock::now() - timeLoop;
    if ( (dt-elaps_loop.count()) > 0 ) {
        usleep( (dt-elaps_loop.count())*1000*1000 );
    }
    else {cout << "Communication Time Out!" << endl;}
  }

  dataFile.close(); // close file
  // stop the threads
  // Myo_thread.interrupt();
  FT_thread.interrupt();
  stop_thread.interrupt();
  vrep_thread.interrupt();

  pb.shutdown_motors();
  pb.update();
  usleep(200*1000);
  cout << "Exiting ..." << endl;

}
