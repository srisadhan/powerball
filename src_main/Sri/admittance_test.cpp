#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
// #include <unistd.h>
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

// dynamixel headers
#include <utils.h>
#include <USB2Dynamixel.h>
#include <commonOptions.h>

// plotting and linear algebra libraries
// #include "matplotlibcpp.h"
#include "sigpack.h"
#include <armadillo>

using namespace::std;
using boost::asio::ip::tcp;
using namespace::TooN;
// namespace plt = matplotlibcpp;
// using namespace std::chrono;


Color::Modifier red(Color::FG_RED);
Color::Modifier green(Color::FG_GREEN);
Color::Modifier def(Color::FG_DEFAULT);


//Global variables
float dt = 0.005f; // sampling time

Vector<6,float> FT  = Zeros;
Vector<6,float> Q   = Zeros;
Vector<6,float> Qe  = Zeros;
Vector<3,float> X   = Zeros;
Vector<3,float> X_init = Zeros;
Vector<6,float> Qdot = Zeros;
Vector<6,float> Qdot_a = Zeros;
Vector<6,float> joints_vrep = Zeros;
simxFloat newPos[3] ={0.0f,0.439,0.275};
bool Start_record   = false;

// Cartesian admittance parameters_ one time define
Vector<6,float> Md_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*0.2f; // for constant m (low multiplier (0.1f) - high mass, high multiplier (0.3f)- low mass)
//Vector<6,float> Md_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*(1.0f/15.0f); // for var m (min:3.5 max:)
Matrix<6,6,double> Md_inv = Md_diag.as_diagonal();

//Vector<6,float> Cd_diag = makeVector(1.2f,1.0f,1.0f,1.0f,1.0f,1.0f)*80; // for fine Low
// Vector<6,float> Cd_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*25; // for fine High
Vector<6,float> Cd_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*90; // for gross Low (multiplier (90) - low damping; multiplier (190) - high damping)

// Increase the Mass and damping in the z direction for the VSM

Matrix<6,6,double> Cd = Cd_diag.as_diagonal();
Vector<6,float> vel = Zeros;


// stops the program
void stop(bool* flag){
    char in;
    cin.get(in);
    *flag = true;
    Start_record = false;
    printf("Stop flag triggered! \n");
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

    // newPos[0] = (X[1]-X_init[1]); // for maze traversal eperiment
    // newPos[1] = X[0]-X_init[0]+0.06;
    // newPos[2] = 0.01;//X[2];

    newPos[0] = -(X[1]-X_init[1])+0.06; // for maze traversal
    newPos[1] = X[0]-X_init[0]+0.439;
    newPos[2] = 0.01;//X[2];

    Rmat.slice<0,0,3,3>() = R.slice<0,0,3,3>();
    Rmat.slice<3,3,3,3>() = R.slice<0,0,3,3>();

    // Rotation about z axis is prevented - Modify according to the force sensor is attached to the end-effector
    F_modified[0] = FT[0];
    F_modified[1] = FT[1];
    F_modified[2] = FT[2];
    // F_modified[3] = 4*FT[3];
    // F_modified[4] = 4*FT[4];

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


// ----------------------------Main function------------------------------
int main(int argc, char** argv)
{
  bool vrepFlag = true;

  // arg to pass to the code is in the format "sudo ./admittance_3d_velMode vrep arg2 arg3 ..."
  if (argc>1){
    if (strcmp(argv[1],"vrep") == 0){
      vrepFlag = false;
    }
  }

  // timestamp vars
  char buffer[10];
  struct timeval tv;
  time_t curtime;
  std::chrono::time_point<std::chrono::system_clock> timeLoop;
  std::chrono::duration<float> elaps_loop;

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
  // Qe = Data(-0.6992f,-0.2623f,1.4834f,0.0f,1.4363f,-0.6112f); // peg in a hole experiment
//    ------------------------------------------------------------------------------------------
// Moving the robot to start position is stopping the robot when already close to start position
// FIXME: Fix this part
  Qe = Data(0.0f,-0.2623f,1.4834f,0.0f,1.4363f,0.0f); // line traversal
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
//    ------------------------------------------------------------------------------------------

  // Get current pose of the robot
  Kin kin;
  kin.FK_pos(Q,&X_init);
  // set velocity mode active
  pb.set_control_mode(MODES_OF_OPERATION_VELOCITY_MODE);
  pb.update();

  // Open recording file
  std::ofstream dataFile;
  dataFile.open("Sri/admittance.csv");
  dataFile << "Time,Q1,Q2,Q3,Q4,Q5,Q6,dQ1,dQ2,dQ3,dQ4,dQ5,dQ6,FT1,FT2,FT3,FT4,FT5,FT6,SimX,SimY,X,Y" << endl;

  // Initializing FT sensor
  bool errFlag=false;
  boost::thread FT_thread(TCP_receive,&errFlag);

  cout << "Admittance loop started!" << endl;
  cout<< green << "Start the experiment "<< def <<endl;

  Matrix<3,4,float> T_mat;
  while(!stop_flag)
  {

    timeLoop = std::chrono::system_clock::now();
    boost::thread threaded_computation(computations);

    // velocity saturation
    if (TooN::norm_inf(Qdot)>32.5*M_PI/180)
    {
        cout << "saturation!" << endl;
        stop_flag = true; // Emergency stop
        // FIXME: Don't stop while the velocity exceeds maximum velocity instead restrict it to that velocity 
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

    dataFile<<Q[0]<<","<<Q[1]<<","<<Q[2]<<","<<Q[3]<<","<<Q[4]<<","<<Q[5]<<","
                 <<Qdot_a[0]<<","<<Qdot_a[1]<<","<<Qdot_a[2]<<","<<Qdot_a[3]<<","<<Qdot_a[4]<<","<<Qdot_a[5]<<","
                  <<FT[0]<<","<<FT[1]<<","<<FT[2]<<","<<FT[3]<<","<<FT[4]<<","<<FT[5]<< "," <<newPos[0]<<","<< newPos[1]<<","<< X[0]<<","<< X[1]<<endl;

    threaded_computation.join();

    elaps_loop = std::chrono::system_clock::now() - timeLoop;
    if ( (dt-elaps_loop.count()) > 0 ) {
        usleep( (dt-elaps_loop.count())*1000*1000 );
    }
    else {cout << "Communication Time Out!" << endl;}

    kin.FK_pos(Q,&X_init);
    // cout << Q <<endl;
  }

  dataFile.close(); // close file
  // stop the threads
  FT_thread.interrupt();
  stop_thread.interrupt();
  vrep_thread.interrupt();

  pb.shutdown_motors();
  pb.update();
  usleep(200*1000);
  cout << "Exiting ..." << endl;

}
