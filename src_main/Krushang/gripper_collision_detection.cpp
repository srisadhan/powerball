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

// dynamixel headers
#include <utils.h>
#include <USB2Dynamixel.h>
#include <commonOptions.h>

// phidget headers
#include <phidget21.h>


// plotting and linear algebra libraries
#include "matplotlibcpp.h"
// #include "sigpack.h"
// #include <armadillo>


//the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET       "\033[0m"
#define RED         "\033[31m"              /* Red */
#define GREEN       "\033[32m"              /* Green */
#define BOLDRED     "\033[1m\033[31m"       /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"       /* Bold Green */

using namespace::std;
using namespace::TooN;
namespace plt = matplotlibcpp;
using boost::asio::ip::tcp;


// global vars
Vector<6,float> FT;
const double dt = 0.01; // sampling time
const double resolution = 300*M_PI*25.46e-3/2/180/1024;
int curr_pos_id1 = 1023;
int curr_pos_id2 = 800;
float integral_sum          = 0;
float residual              = 0;
float mass                  = 0.2260;                //  (100+2*45+2*18)/1000;   // mass of the gripper
float Fv                    = 0.45;                   // viscous coeeficient
Vector<3,float> xe          = makeVector(1,0,0);      // coordinate of X axis of end effector in end effector frame
Vector<3,float> gravity     = makeVector(0, 0,-9.81); // gravity vector in base frame
float KI                    = 40;                     // detection gain
float residual_threshold    = 0.25;                    // colllision detection threshold 
// global strings
std::string taskType = "";

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

void stop(bool* flag){
    char in;
    std::cin.get(in);
    *flag = true;
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
    std::cout << "TCP recieved: " << recv_buf << std::endl;

    // continous receiving
    msg="L1()\n";
    socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
    len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
    std::cout << "TCP recieved: " << recv_buf << std::endl;

    // Force data
    for (;;)
    {
        //msg="F()\n";
        //socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
        len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
        int timeStamp;
        sscanf(recv_buf,"F={%f,%f,%f,%f,%f,%f},%d",&FT[0],&FT[1],&FT[2],&FT[3],&FT[4],&FT[5],&timeStamp);

    }
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
    vector<double> potOff = {165, 120};
    linPot[Index] = ((Value - potOff[Index]) * 100 / 1024) / 1e3;

    // if (Index < 2)  cout << Index << ":" << Value << "\n";
    // if (Index < 2)  cout << Index << ":" << linPot[0] << "," << linPot[1] << "\n";
    return 0;
}

int CCONV ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
    printf("Error handled. %d - %s", ErrorCode, unknown);
    return 0;
}

void CloseGripper(int id1, int id2, USB2Dynamixel &usb2Dynamixel){

  // Start with this position - low stiffness
  setPosition(id1, 400, usb2Dynamixel);     // for fully close 350  or semi close 600
  setPosition(id2, 200, usb2Dynamixel);

  usleep(0.05*1e6);

  curr_pos_id1 = 600;
  curr_pos_id2 = 200;
}

void OpenGripper(int id1, int id2, USB2Dynamixel &usb2Dynamixel){

  // Start with this position - low stiffness
  setPosition(id1, 1023, usb2Dynamixel);
  setPosition(id2, 800, usb2Dynamixel);

  usleep(0.05*1e6);

  curr_pos_id1 = 1023;
  curr_pos_id2 = 800;
}

double OuterMagSep(int pos_bit){
    double pos = (pos_bit - 250) * resolution + (.025 - 0.01615); // 23 mm offset at 250 bit encoder position minus 16.15 mm finger width (zero position of outer magnet or finger)
    return pos;
}

double InnerMagSep(int pos_bit){
    double pos = (pos_bit - 515) * resolution; // 5.85 mm offset at 315 bit encoder position (zero position of inner magnet or finger)
    return pos;
}

// Magnet force model
double magForce(double s){
  double a = 5.5e-6;
  double b = 6.85e-5;
  double c = 2.22e-7;

  if (s<0){
      s=0;
  }
  return a/(pow(s,3)+b*s+c);
}

// Skew Symmetry Matrix
void skew_symmetry(Vector<3,float> w, Matrix<3,3,float> *S){
    float w1= w[0];
    float w2= w[1];
    float w3= w[2];

    *S = Data(0 , -w3, w2 ,
              w3 , 0 , -w1,
             -w2, w1, 0  );
}


// residual signal generator
float residual_signal(Vector<6,float> Q, Vector<6,float> Qdot,Vector<3,float> r_end_effector_pos,float x1_dot,float Fm ,float Previous_residual){

    Kin kin;
    Matrix<3,6,float> J_lin = Zeros;     // 3 x 6 linear part of jacobian Matrix
    Matrix<3,6,float> J_ang = Zeros;     // 3 x 6 angular part of jacobian Matrix
    Matrix<3,3,float> R     = Zeros;     // 3 x 3 forward rotation matrix

    // R[0][0] = round(R[0][0]*100000)/100000; 
    // R[0][1] = round(R[0][1]*100000)/100000;
    // R[0][2] = round(R[0][2]*100000)/100000;

    // R[1][0] = round(R[1][0]*100000)/100000;
    // R[1][1] = round(R[1][1]*100000)/100000;
    // R[1][2] = round(R[1][2]*100000)/100000;

    // R[2][0] = round(R[2][0]*100000)/100000;
    // R[2][1] = round(R[2][1]*100000)/100000;
    // R[2][2] = round(R[2][2]*100000)/100000;
    kin.JacobPos(Q, &J_lin);
    kin.JacobRot(Q, &J_ang);
    kin.FK_R(Q, &R);


    Matrix<3,3,float> R_trans = R.T();

    Matrix<3,3,float> Sw =Zeros;
    Vector<3,float> omega = R_trans * J_ang * Qdot;
    skew_symmetry(omega, &Sw);

    //calculating absolute translation velocity of grasped object in end-effector frame
    Vector<3,float> pe_dot = R_trans*J_lin*Qdot + Sw * r_end_effector_pos + makeVector(x1_dot,0,0);
    // cout << "pe_dot: "<< pe_dot << "\n";
    // calculating linear momentum and its derivative
    float p_linear_momentum = mass* (xe * pe_dot);
    float p_linear_mom_dot  = mass * (xe *(R_trans*gravity - Sw *pe_dot) ) + Fm + Fv * x1_dot;

    float integral_part = (p_linear_mom_dot+Previous_residual)*dt;
    integral_sum = integral_sum+ integral_part;

    residual = KI * (p_linear_momentum-integral_sum);

    return residual ;
}



int main(int argc, char **argv){

    char in;
    std::cout << "Please enter the type of task from (1, 2, 3) \n";
    std::cin >> taskType;
    std::cin.get(in);

    int vrep_bool = 3;

    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;


    if (argc < 2)
    {
        vrep_bool = 0;
        cout << GREEN << "Applying trajectory on PowerBall only" << RESET << endl;
    } else
    {
        if (strcmp(argv[1],"-v")==0)
        {
            vrep_bool = 1;
            cout << GREEN << "Applying trajectory on PowerBall and Vrep" << RESET << endl;
        } else if (strcmp(argv[1],"-vo")==0)
        {
            vrep_bool = 2;
            cout << GREEN << "Applying trajectory on Vrep only" << RESET << endl;
        }

    }

    // Initializing dynamixel
    USB2Dynamixel usb2dyn(*cnfDevice, 4);
    int id1 = 1, id2 = 2;
    setMaxTorque(id1, 1023, usb2dyn); // outer magnets max 1023
    setMaxTorque(id2, 1023, usb2dyn); // inner magnets

    OpenGripper(id1, id2, usb2dyn);
    // CloseGripper(id1, id2, usb2dyn);
    
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
    for(int sensor = 0; sensor < numSensors; sensor++)
    {
        CPhidgetInterfaceKit_setSensorChangeTrigger(ifKit, sensor, 0);
    }

    cout << "dynamixel and phidget are joined!" << endl;
    bool stopFlag = false;
    boost::thread stop_thread(stop,&stopFlag);
    
    // gripper test code 
    // while(!stopFlag){
        // std::cout<<OuterMagSep(curr_pos_id1)<<","<<linPot[0] << "," << InnerMagSep(curr_pos_id2) << "," << linPot[1] << endl;
        // std::cout<<OuterMagSep(curr_pos_id1)-linPot[0] << "," << linPot[0]-InnerMagSep(curr_pos_id2) << "," << OuterMagSep(curr_pos_id1)-linPot[1] << "," << linPot[1]-InnerMagSep(curr_pos_id2)  << endl;
    // }

    // Initializing FT sensor
    bool errFlag=false;
    boost::thread FT_thread(TCP_receive,&errFlag);


    // open a file to record data
    std::ofstream dataFile;
    dataFile.open("../data/KG/collision_detection/collision_" + taskType +".csv", ios::out);
    dataFile<< "Time,Q1,Q2,Q3,Q4,Q5,Q6,Qdot1,Qdot2,Qdot3,Qdot4,Qdot5,Qdot6,Torque1,Torque2,Torque3,Torque4,Torque5,Torque6,Fx,Fy,Fz,Mx,My,Mz,Sep11,Sep12,Sep21,Sep22,f11,f12,f21,f22,Outer_mag_Pos,Residual"<< endl;


    /* Set sampling and timing options*/
    std::chrono::time_point<std::chrono::system_clock> timeLoop;
    std::chrono::duration<float> elaps_loop;
    /*-----------------------------------------*/
    /*  Powerball class */
    SchunkPowerball pb;
    pb.update();
    pb.goHome();
    pb.update();

    // Check for V-REP connection
    int res = -1;
    V_rep vrep;
    if (vrep_bool!=0)
    {
        res = vrep.connect();
        if (res==-1)
        {
            cout << BOLDRED <<"V-REP Connection Error" << RESET << endl;
            return 0;
        }
    }

    /* VREP Class */
    TooN::Vector<6,float> joint_angle = Zeros;
    Vector<6,float> joint_angle_initializing = Zeros;

    Kin kin;

    /*Bring the end-effector of the robot to the starting point of trajectory*/
    Vector<6,float>Qs = pb.get_pos(); // makeVector(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    Vector<6,float>Qe = makeVector(0.0f, 0.0f, 40.0f, 0.0f, 0.0f, 0.0f)/180 * M_PI;

    /*------------------*/
    Vector<2,float> pos = Zeros;
    Vector<6,float> Q_present = pb.get_pos();
    Vector<6, float> Tor = pb.get_tor();
    Vector<6,float> Qdot = pb.get_vel();
    Vector<6,float> Qdot_i = Zeros;

    // interpolate the traj between current robot position and the starting point of the traj

    std::vector<std::vector<double>>interpolated_data1;

    int num_H1 = kin.HerInter(Qs,Qdot_i,Qe,dt,4.0,&interpolated_data1);


    int i=0;
    int col,row = 0;
    Matrix<Dynamic,6,double> reach_start(num_H1, 6);
    for (std::vector< std::vector<double> >::const_iterator it = interpolated_data1.begin(); it != interpolated_data1.end(); ++ it)
    {
       col = 0;
       for (std::vector<double>::const_iterator itit = it->begin(); itit != it->end(); ++ itit)
       {
           reach_start(row,col) = *itit;
           col++;
       }
       row++;
    }

    std::vector<float> residual_history(num_H1,0);
    float previous_gripper_finger1 = linPot[0];

    // Bring the robot to start position
    while(i<num_H1 & !stopFlag)
    {
        timeLoop = std::chrono::system_clock::now();
        if (vrep_bool!=2)
        {
          pb.set_pos(reach_start[i]);
          pb.update();
        }
        if (res!=-1)
        {
            if (vrep.isConnected())
            {
                for (int n=0;n<6;n++)
                {
                    joint_angle_initializing[n] = reach_start(i,n);
                }
                vrep.setq(joint_angle_initializing);
            } else
            {
                cout << BOLDRED <<"V-REP Disconnected!" << RESET << endl;
                return 0;
            }
        }
        if (i > 0) {
            Q_present = pb.get_pos();
            // Tor = pb.get_tor();
            Qdot = pb.get_vel();

            float Gripper_finger1 = linPot[0];
            float Gripper_finger2 = linPot[1];

            Vector<3,float> r_end_effector_pos = makeVector(Gripper_finger1-Gripper_finger2,0,0.17);
            float x1_dot          = (Gripper_finger1-previous_gripper_finger1)/dt;
            previous_gripper_finger1 = Gripper_finger1;

            //calculating contact force
            // float Fm1 = magForce(OuterMagSep(curr_pos_id1)-linPot[0]) + magForce(linPot[0]-InnerMagSep(curr_pos_id2));
            // float Fm2 = magForce(OuterMagSep(curr_pos_id1)-linPot[1]) + magForce(linPot[1]-InnerMagSep(curr_pos_id2));

            // float Fe  = Fm1-Fm2;

            // total magnetic force
            float Fm = (magForce(OuterMagSep(curr_pos_id1)-linPot[0]) - magForce(linPot[0]-InnerMagSep(curr_pos_id2)) ) -(magForce(OuterMagSep(curr_pos_id1)-linPot[1]) - magForce(linPot[1]-InnerMagSep(curr_pos_id2)) );
            
            //calculting residual signal
            residual_history[i] = residual_signal(Q_present,Qdot,r_end_effector_pos,x1_dot,Fm,residual_history[i-1]);
            float residual_dot = KI * (Fm-residual_history[i]);

            if (abs(residual_history[i]) >= residual_threshold){
                cout << RED <<"Collision detected at : " << i <<  RESET << "\n";
            }
        }
        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        cout << elaps_loop.count()  << "\n";
        
        // cout << OuterMagSep(curr_pos_id1) << "," << InnerMagSep(curr_pos_id2) << "," << linPot[0] << ","  << linPot[1] << "\n";

        // timestamp
        gettimeofday(&tv, NULL);
        curtime=tv.tv_sec;
        strftime(buffer, 10, "%S", localtime(&curtime));
        dataFile << buffer << ":" << tv.tv_usec << ",";

        dataFile<< Q_present[0]<< "," << Q_present[1]<< ","<< Q_present[2]<< ","<< Q_present[3]<< ","<< Q_present[4]<< ","<< Q_present[5]<< "," <<
        Qdot[0]<< "," << Qdot[1]<< ","<< Qdot[2]<< ","<< Qdot[3]<< ","<< Qdot[4]<< ","<< Qdot[5]<< "," <<
        Tor[0]<< "," << Tor[1]<< ","<< Tor[2]<< ","<< Tor[3]<< ","<< Tor[4]<< ","<< Tor[5]<< "," <<
        FT[0] << "," << FT[1] << "," << FT[2] << "," << FT[3] << "," << FT[4] << "," << FT[5] << "," <<
        OuterMagSep(curr_pos_id1)-linPot[0] << "," << linPot[0]-InnerMagSep(curr_pos_id2) << "," << OuterMagSep(curr_pos_id1)-linPot[1] << "," << linPot[1]-InnerMagSep(curr_pos_id2) << "," <<
        magForce(OuterMagSep(curr_pos_id1)-linPot[0]) << "," << magForce(linPot[0]-InnerMagSep(curr_pos_id2)) << "," << magForce(OuterMagSep(curr_pos_id1)-linPot[1]) << "," << magForce(linPot[1]-InnerMagSep(curr_pos_id2)) << "," <<OuterMagSep(curr_pos_id1) << "," << // "\n";
        residual_history[i] << "\n" ;

        i++;

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ((dt-elaps_loop.count()) > 0) {
            usleep( (dt-elaps_loop.count())*1000*1000);
        }else
        {
            cout << "communication delay" << "\n";
        }
        
    }
    cout<<"Finished sending the robot to the start point of the trajectory"<<endl;

    dataFile.close();

    plt::figure();
    plt::plot(residual_history,"r--");
    plt::show();
    // plt::pause(0.001);

    CPhidget_close((CPhidgetHandle)ifKit);
    CPhidget_delete((CPhidgetHandle)ifKit);
    usleep(1000*1000);

    FT_thread.interrupt();
    stop_thread.interrupt();

    // shutdown the powerball motors
    pb.update();
    usleep(1000*1000);
    std::cout << "Exiting ..." << std::endl;

    return 0;

}
