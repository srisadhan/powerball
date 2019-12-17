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
#include "utils/utils.h"

// Toon headers
#include <TooN/LU.h>
#include <TooN/SVD.h>

// myo band headers
#include "myolinux/myoclient.h"
#include "myolinux/serial.h"


using namespace::std;
using boost::asio::ip::tcp;
using namespace::TooN;
using namespace myolinux;


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
Vector<6,float> joints_vrep = Zeros;
simxFloat newPos[3] ={0.0f,0.439,0.275};
bool Start_record   = false;

// Cartesian admittance parameters_ one time define
Vector<6,float> Md_diag = makeVector(.5,.5,.5,1.0f,1.0f,20.0f);
Matrix<6,6,double> Md_inv = Md_diag.as_diagonal();
Vector<6,float> Cd_diag = makeVector(7.0f,7.0f,7.0f,1.0f,1.0f,1.0f); // 7 for low and 25 for high damping
// Vector<6,float> Cd_diag = makeVector(25.0f,25.0f,25.0f,1.0f,1.0f,1.0f); // high damping
Matrix<6,6,double> Cd = Cd_diag.as_diagonal();
Vector<6,float> vel = Zeros;

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
          // vrep.setSphere(&newPos[0]);
          vrep.setq(joints_vrep);
          usleep(40*1000);
      }
    }
}

// The computations for admittance control are carried on here
void computations(){
    Matrix<6,6,float> J = Zeros;
    Matrix<6,6,float> Rmat = Zeros;  // 6 by 6 matrix with R partitioned
    Matrix<3,3,float> R = Zeros;
    Vector<3,float> X = Zeros;
    Vector<6,float> F_modified = Zeros;

    Kin kin;
    kin.Jacob(Q,&J);
    kin.FK_R(Q,&R);
    kin.FK_pos(Q,&X);

    newPos[0] = -X[1];
    newPos[1] = X[0];
    newPos[2] = X[2];

    Rmat.slice<0,0,3,3>() = R.slice<0,0,3,3>();
    Rmat.slice<3,3,3,3>() = R.slice<0,0,3,3>();

    // Rotation about z axis is prevented - Modify according to the force sensor is attached to the end-effector
    F_modified[0] = FT[0];
    F_modified[1] = FT[1];
    F_modified[2] = FT[2];
    // F_modified[3] = 5*FT[3];
    // F_modified[4] = 5*FT[4];

    //Vector<6,float> FT2 = makeVector(0,5,0,0,0,0);
    vel = vel + dt*(Md_inv*(Rmat*F_modified) - Md_inv*Cd*vel);

    // solve inv(A)*b using LU
    SVD<6,6,float> luJ(J);
    Vector<6,float> Qdot = luJ.backsub(vel);

    // saturation???
    Qe = dt*Qdot+Q;
    for (int i = 0;i<6;i++){
      joints_vrep[i] = Qe[i];
    }
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
        cout<< green <<"Establishing connection with the Myo band .........."<<def<<endl;
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
    EMGFile.open("Sri/EMG.csv");
    EMGFile << "Time,EMG1,EMG2,EMG3,EMG4,EMG5,EMG6,EMG7,EMG8"<< endl;

    IMUFile.open("Sri/  IMU.csv");
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

              EMGFile<< EMG[1]<< "," << EMG[2]<< "," << EMG[3]<< "," << EMG[4]<< "," << EMG[5]<< "," << EMG[6]<< "," << EMG[7]<< "," << EMG[8]<<endl;
              IMUFile<< ORI[1]<< "," << ORI[2]<< "," << ORI[3]<< "," << ORI[4]<< "," << ACC[1]<< "," << ACC[2]<< "," << ACC[3]<< "," << GYR[1]<< "," << GYR[2]<< "," << GYR[3] <<endl;
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
    EMGFile.open("Sri/Calib.csv");
    EMGFile << "Time,EMG1,EMG2,EMG3,EMG4,EMG5,EMG6,EMG7,EMG8"<< endl;

    while(!*errFlag){
        try {
            client.listen();
            gettimeofday(&tv, NULL);
            curtime=tv.tv_sec;
            strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));

            EMGFile << buffer << ":" << tv.tv_usec << ",";
            EMGFile<< EMG[1]<< "," << EMG[2]<< "," << EMG[3]<< "," << EMG[4]<< "," << EMG[5]<< "," << EMG[6]<< "," << EMG[7]<< "," << EMG[8]<<endl;
        }
        catch(myo::DisconnectedException &) {
            cout << "MYO Disconnected" << endl;
        }
    }
}

// Main function
int main(int argc, char** argv)
{
  bool vrepFlag = false;
  if (argc>1){
    if (strcmp(argv[1],"vrep") == 0){
      vrepFlag = true;
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
  Qe = Data(-0.6992f,-0.2623f,1.4834f,0.0f,1.4363f,-0.6112f);
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

  // Open recording file
  std::ofstream dataFile;
  dataFile.open("Sri/admittance.csv");
  dataFile<<"Time"<<","<<"Q1"<<","<<"Q2"<<","<<"Q3"<<","<<"Q4"<<","<<"Q5"<<","<<"Q6"<<"," << "Fx" <<","<< "Fy" << ","<< "Fz" <<endl;

  // Initializing FT sensor
  bool errFlag=false;
  boost::thread FT_thread(TCP_receive,&errFlag);

  cout << "Admittance loop started!" << endl;
  cout<< green << "Start the experiment "<< def <<endl;


  while(!stop_flag)
  {
    timeLoop = std::chrono::system_clock::now();
    pb.set_pos(Qe);
    boost::thread threaded_computation(computations);
    pb.update();
    Q = pb.get_pos();

    Start_record = true;
    // timestamp
    gettimeofday(&tv, NULL);
    curtime=tv.tv_sec;
    strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));
    dataFile << buffer << ":" << tv.tv_usec << ",";
    dataFile<<Q[0]<<","<<Q[1]<<","<<Q[2]<<","<<Q[3]<<","<<Q[4]<<","<<Q[5]<<"," << -FT[0] <<","<< -FT[1] << ","<< FT[2] <<endl;
    threaded_computation.join();

    elaps_loop = std::chrono::system_clock::now() - timeLoop;
    if ( (dt-elaps_loop.count()) > 0 ) {
        usleep( (dt-elaps_loop.count())*1000*1000 );
    }
    else {
        cout << "Communication Time Out!" << endl;
    }
  }

  dataFile.close(); // close file
  // stop the threads
  Myo_thread.interrupt();
  FT_thread.interrupt();
  stop_thread.interrupt();
  vrep_thread.interrupt();
  usleep(1000*500);
  cout << "Exiting ..." << endl;

}
