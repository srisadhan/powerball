#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <ostream> // included for color output to the terminal
#include <typeinfo>

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
#include "utils/powerball_utils.h"
#include "utils/vsm_utils.h"
#include "vsm/vsm_control.h"

// Toon headers
#include <TooN/LU.h>
#include <TooN/SVD.h>

// myo band headers
#include "myolinux/myoclient.h"
#include "myolinux/serial.h"

// plotting and linear algebra libraries
#include "matplotlibcpp.h"
#include "sigpack.h"
#include <armadillo>

using namespace::std;
using boost::asio::ip::tcp;
using namespace::TooN;
using namespace myolinux;
namespace plt = matplotlibcpp;

// Initializing the myo connection
myo::Client client(Serial{"/dev/ttyACM0", 115200});

// Replace the control table parameters according to the dynamixel model
// These parameters are for the new dynamixel motors (XM430-W210R) used in the VSM
#define ADDR_MX_TORQUE_ENABLE           64                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           116
#define ADDR_MX_PRESENT_POSITION        132

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID1                         1                   // Dynamixel ID: 1
#define DXL_ID2                         2                   // Dynamixel ID: 2
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
#define DXL_MOVING_STATUS_THRESHOLD     20

#define ESC_ASCII_VALUE                 0x1b
#define UPARROW_ASCII_VALUE             0x18
#define DOWNARROW_ASCII_VALUE           0x19

//Global variables
float dt = 0.005f; // sampling time

const double dxl_resolution = 300 * M_PI/180 * 25.46e-3/2 /4096;       // multiply this value with the dynamixel encode value to get displacement
float passive_mag_width = .020;                                       // width of the magnet block holding hammer

// dynamixel calibration parameters
uint32_t VSM_MIN_POS = 50;                                           // calibrated position of the VSM to set the active magnets to 0.001 m on each side
uint32_t VSM_MAX_POS = 2048;                                          // calibrated maximum position of the active magnets in the VSM


// int dxl_comm_result = COMM_TX_FAIL;         // Communication result 
// uint8_t dxl_error = 0;                      // Dynamixel error
uint32_t dxl_pos_t;                         // dxl position at time step t (present position)
uint32_t dxl_pos_t1;                        // dxl position at previous time step t-1
uint8_t torque_status = 0;
int dxl_init_position = 600;           // initialize the goal position with a safe value

std::vector<uint32_t> dxl_pos(2, 0);        // encoder reading of each dynamixel 
// std::vector<float> linPot{0, 0};            // potentiometer reading in m from the equilibrium postion
std::vector<float> vsm_force(2, 0);         // resultant force on each degree of freedom (vsm_force[i] - ith DoF)
std::vector<std::vector<float>> mag_sep(2, std::vector<float> (2, 0));            // magnet separation matrix

std::string damping;
std::string stiffness;

Vector<8,float> EMG = Zeros;
Vector<4,float> ORI = Zeros;
Vector<3,float> ACC = Zeros;
Vector<3,float> GYR = Zeros;
Vector<6,float> Q   = Zeros;
Vector<6,float> Qs  = Zeros;
Vector<6,float> Qe  = Zeros;
Vector<6, float> Q_interm;
Vector<3,float> X   = Zeros;
Vector<3,float> X_init = Zeros;
Vector<6,float> Qdot = Zeros;
Vector<6,float> Qdot_a = Zeros;
Vector<6,float> joints_vrep = Zeros;
simxFloat newPos[3] ={0.0f,0.439,0.275};
bool Start_record   = false;
// arma::mat emgVec(10000,8,arma::fill::zeros);
int counter = 0;
int windLen = 50;

// Cartesian admittance parameters_ one time define
Vector<6,float> Md_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*0.05f; // for constant m (low multiplier (0.1f) - high mass, high multiplier (0.3f)- low mass)
//Vector<6,float> Md_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*(1.0f/15.0f); // for var m (min:3.5 max:)
Matrix<6,6,double> Md_inv = Md_diag.as_diagonal();

//Vector<6,float> Cd_diag = makeVector(1.2f,1.0f,1.0f,1.0f,1.0f,1.0f)*80; // for fine Low
// Vector<6,float> Cd_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*25; // for fine High
Vector<6,float> Cd_diag = makeVector(1.0f,1.0f,1.5f,1.0f,1.0f,1.0f)*130; // for gross Low (multiplier (90) - low damping; multiplier (190) - high damping)

// Increase the Mass and damping in the z direction for the VSM

Matrix<6,6,double> Cd;
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
          vrep.setSphere(&newPos[0]);
          // vrep.setq(joints_vrep);
          usleep(40*1000);
      }
    }
}

// The computations for admittance control are carried on here
void computations(){
    Matrix<6,6,float> J = Zeros;
    Matrix<6,6,float> Rb_e_mat = Zeros;  // 6 by 6 matrix with R partitioned
    Matrix<6,6,float> Re_f_mat = Zeros;  // 6 by 6 matrix with R partitioned
    Matrix<3,3,float> Rb_e = Zeros;  // rotation of the end-effector wrt the base frame
    Matrix<3,3,float> Re_f = Data(cos(-M_PI/4), -sin(-M_PI/4),  0,
                                  sin(-M_PI/4),  cos(-M_PI/4),  0,
                                            0,            0,  1); // rotation of the force sensor wrt end-effector
    // Vector<3,float> X = Zeros;
    Vector<6,float> F_modified = Zeros;

    Kin kin;

    kin.Jacob(Q,&J);
    kin.FK_R(Q,&Rb_e);
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

    // Replicate the rotation matrix to multiply with force and moment values
    Rb_e_mat.slice<0,0,3,3>() = Rb_e;
    Rb_e_mat.slice<3,3,3,3>() = Rb_e;

    Re_f_mat.slice<0,0,3,3>() = Re_f;
    Re_f_mat.slice<3,3,3,3>() = Re_f;

    // the location of moment application is 10 cm from the center of FT sensor 
    FT.slice<3,3>() = FT.slice<3,3>() * 0; 

    // transform the rotation matrix from FT-frame to base frame
    F_modified = Rb_e_mat * Re_f_mat * FT; // Use this for rotation along with translation in cartesian space
    // NOTE: Use the method described in 1. (or) 2. for only translation 
    // F_modified.slice<0,3>() = Rb_e * Re_f * FT.slice<0,3>(); // only for translation in cartesian space 

    // printf("Force data:{%f, %f, %f, %f, %f, %f}\n", F_modified[0],F_modified[1],F_modified[2],F_modified[3],F_modified[4],F_modified[5]);
    
    vel = vel + dt*(Md_inv*F_modified - Md_inv*Cd*vel);

    // There are two ways to prevent rotation in cartesian space
    // Note: Currently the damping is set to very high value to prevent rotation (reduce it if you want to rotate the end-effector)
    // 1. set the angular velocities zero after the velocity has been calculated.
    // 2. set the moments to zeros before calculating cartesian velocity
    vel.slice<3,3>() = Zeros;

    // solve inv(A)*b using LU
    SVD<6,6,float> luJ(J);
    Qdot = luJ.backsub(vel);

}



// Myo armband files
myo::Client Myo_init()
{
    client.connect();// Autoconnect to the first Myo device
    if (!client.connected()) {
        cout<< RED<< "Unable to connect to Myo band"<<DEFAULT <<endl;
    }else{
        cout<< GREEN <<"Connection established with the Myo band .........."<<DEFAULT <<endl;
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
    EMGFile.open("../data/VSM/peg-hole/EMG_stiff_" + std::to_string(dxl_init_position) + "_" + damping +".csv");
    EMGFile << "Time,EMG1,EMG2,EMG3,EMG4,EMG5,EMG6,EMG7,EMG8"<< endl;

    IMUFile.open("../data/VSM/peg-hole/IMU_stiff_" + std::to_string(dxl_init_position) + "_" + damping +".csv");
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
              // counter += 1;

              EMGFile<< EMG[0]<< "," << EMG[1]<< "," << EMG[2]<< "," << EMG[3]<< "," << EMG[4]<< "," << EMG[5]<< "," << EMG[6]<< "," << EMG[7]<< endl;
              IMUFile<< ORI[0]<< "," << ORI[1]<< "," << ORI[2]<< "," << ORI[3]<< "," << ACC[0]<< "," << ACC[1]<< "," << ACC[2]<< "," << GYR[0]<< "," << GYR[1]<< "," << GYR[2] <<endl;
            }
        }
        catch(myo::DisconnectedException &) {
            cout << "MYO Disconnected" << endl;
        }
    }
}

void EMG_calib(bool *calibFlag)
{
    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;

    // Open recording file
    std::ofstream EMGFile, IMUFile;
    EMGFile.open("../data/VSM/peg-hole/emg_calib/calib.csv");
    EMGFile << "Time,EMG1,EMG2,EMG3,EMG4,EMG5,EMG6,EMG7,EMG8"<< endl;

    while(*calibFlag){
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

// VSM collision detection 
void vsm_computations(VSMControl *vsm, std::vector<int> dxl_id_list, bool *stopFlag, bool *vsmFlag)
{   
    float external_force; 
    std::vector<float> sep;
    vsm->dxl_enable();

    for(;;)
    {   // vsmFlag is true when VSM is connected else false
        if ((!(*stopFlag)) && (*vsmFlag))
        {   
            for (int id = 0; id < dxl_id_list.size(); id++)
            {   
                dxl_pos[id] = vsm->xm_get_pos(dxl_id_list[id]);
                vsm->calculate_mag_sep(dxl_pos[id], linPot[id], dxl_resolution, VSM_MIN_POS, passive_mag_width, &sep);

                // calculate the external force acting on the tool
                external_force = resForce(sep);    

                // store the magnet separations in global matrix
                for (int j=0; j < 2; j++)
                {
                    mag_sep[id][j] = sep[j];
                }

                vsm_force[id] = external_force;
                
            }
        }
    }
}

// plot emg envelop
// void matplot_Enlp(bool *stopFlag){
//   std::vector<float> x(2,counter);
//   std::vector<float> ymin(2,0.0);         // min envelop among 8 EMG channels
//   std::vector<float> ymax(2,0.0);         // max envelop among 8 EMG channels
//   std::vector<float> ydiff(2,0.0);        // difference of max and min envelop
//   arma::rowvec temp1(8,arma::fill::zeros);
//   float temp = 0;
//   int srt = 0, stp = 0;
//
//   if(counter >= windLen){
//       srt     = counter-std::floor(windLen/2);
//       stp     = counter+std::floor(windLen/2);
//       temp1   = (arma::sum(emgVec.rows(srt,stp))/50)/windLen;
//       ymin[1] = arma::min(temp1);
//       ymax[1] = arma::max(temp1);
//       ydiff[1]= ymax[1] - ymin[1];
//
//       plt::subplot(3,1,1);
//       plt::plot(x,ymin,"r-");
//       plt::xlim(0,10000);plt::ylim(0,1);
//       plt::suptitle("Min of EMG channels");
//
//       plt::subplot(3,1,2);
//       plt::plot(x,ymax,"r-");
//       plt::xlim(0,10000);plt::ylim(0,1);
//       plt::suptitle("Max of EMG channels");
//
//       plt::subplot(3,1,3);
//       plt::plot(x,ydiff,"r-");
//       plt::xlim(0,10000);plt::ylim(0,1);
//       plt::suptitle("Diff of EMG envelop");
//
//       plt::pause(0.001);
//   }
// }


// ----------------------------Main function------------------------------
int main(int argc, char** argv)
{
    bool vsmFlag = true; // true - if you want to connect to dynamixel else false
    bool vrepFlag = true;
    bool Calib_Flag = false;
    bool FT_calibrated = false;
    stringstream strValue;
    

    // bool stopFlag = false;
    // boost::thread FT_thread(Robotiq_ft_receive, dt, true, &stopFlag, &FT_calibrated);
    // boost::thread stop_thread(stop,&stopFlag);

    // while (!(stopFlag))
    // {

    // }
    // FT_thread.interrupt();
    // stop_thread.interrupt();
    // return 0; 

    // arg to pass to the code is in the format "sudo ./admittance_3d_velMode -v/vo arg2 arg3 ..."
    if (argc>1){
        if (strcmp(argv[1], "-h") == 0) {
            printf(GREEN "Supported arguments in order \n ./admittance_3d_velMode -calib \n (or) \n ./admittance_3d_velMode -no_vsm ld..\n");
            printf("arg1: -calib : prepare just for EMG calibration of the subject\n");
            printf("arg1: -no_vsm : if you don't want to use VSM (or) \n      dynamixel initial position value in between [100, 2048] \n");
            printf("arg1: ld : for low damping admittance control (or) by default it is set to high damping\n" DEFAULT);

            return 0;
        }
        else if ((strcmp(argv[1], "-calib") == 0)) {
            printf("Calibration of EMG using isometric contractions");  
            Calib_Flag = true;
            vsmFlag = false;
        }
        else if (strcmp(argv[1], "-no_vsm") == 0) 
        {
            printf(GREEN "Initializing the admittance control without VSM \n" DEFAULT);
            vsmFlag = false;
            // strValue << argv[1];
            dxl_init_position = 0;

        }
        else
        {
            printf(GREEN "Initializing the admittance control with the VSM \n" DEFAULT);
            vsmFlag = true;
            strValue << argv[1];
            strValue >> dxl_init_position;
            if ((dxl_init_position < 100) || (dxl_init_position > 2048))
            {
                printf("Please provide the values in the range [100, 2048] \n");
                return 1;
            }
        }
        
        if (argc > 2){
            if (strcmp(argv[2], "ld") == 0)
            {
                Cd_diag = makeVector(80.0f,80.0f,200.0f,200.0f,200.0f,200.0f); // low damping
                printf(GREEN "Set the damping level to Low" DEFAULT);
                damping = "ld";
            }else if (strcmp(argv[2], "hd") == 0)
            {
                Cd_diag = makeVector(200.0f,200.0f,200.0f,200.0f,200.0f,200.0f); // high damping
                printf(GREEN "Set the damping level to High" DEFAULT);
                damping = "hd";    
            }
        }
    }
    Cd = Cd_diag.as_diagonal();
    
    VSMControl vsm(BAUDRATE, DEVICENAME, PROTOCOL_VERSION, ADDR_MX_TORQUE_ENABLE, ADDR_MX_GOAL_POSITION, ADDR_MX_PRESENT_POSITION);
    std::vector<int> dxl_id_list;
    if (vsmFlag)
    {
        int dxl_comm_result = COMM_TX_FAIL;                             // Communication result
        uint8_t dxl_error = 0;                                          // Dynamixel error
            
        // connect to the available dynamixel
        if (!vsm.dxl_enable()) return 0;
        printf("Number of dyn connected: %d \n", vsm.get_connected_dxl_count(&dxl_id_list));

        vsm.xm_set_pos(dxl_id_list[0], dxl_init_position, VSM_MIN_POS, VSM_MAX_POS);
        vsm.xm_set_pos(dxl_id_list[1], dxl_init_position, VSM_MIN_POS, VSM_MAX_POS);
        usleep(1e6);
        //Initializing phidget
        if (!phidget_enable()) return 0; // exit the code if cannot connect to phidget

        // set constant position to both the motors before starting the experiment
        // NOTE: Be careful with subtraction of uint values 
        while(abs((int)dxl_pos_t - (int)dxl_init_position) > DXL_MOVING_STATUS_THRESHOLD)
        { 
            vsm.xm_set_pos(dxl_id_list[0], dxl_init_position, VSM_MIN_POS, VSM_MAX_POS);
            dxl_pos_t = vsm.xm_get_pos(dxl_id_list[0]);
            // printf("dyn1 current position: %d and required position: %d\n", dxl_pos_t, dxl_init_position);

            vsm.xm_set_pos(dxl_id_list[1], dxl_init_position, VSM_MIN_POS, VSM_MAX_POS);
            dxl_pos_t = vsm.xm_get_pos(dxl_id_list[1]);
            // printf("dyn2 current position: %d and required position: %d\n", dxl_pos_t, dxl_init_position);
            // cout << abs(dxl_pos_t) << "," << abs(dxl_init_position) << "," << abs((int)dxl_pos_t - (int)dxl_init_position) << endl;
        }

        // Initializing the VSM - settings initial position to the dynamixel motors
        dxl_pos_t = vsm.xm_get_pos(dxl_id_list[0]);
        printf("dyn1 actual position: %d, desired position: %d\n", dxl_pos_t, dxl_init_position);
        dxl_pos_t = vsm.xm_get_pos(dxl_id_list[1]);
        printf("dyn2 actual position: %d, desired position: %d\n", dxl_pos_t, dxl_init_position);

        dxl_pos_t1 = dxl_pos_t; // pos at t and t-1 are same at t=0    
    }

    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;
    std::chrono::time_point<std::chrono::system_clock> timeLoop;
    std::chrono::duration<float> elaps_loop;

    // Initialize Myo band
    myo::Client client = Myo_init(); // initializing the myo band here works (sometimes it works here and sometimes after initializing the admittance control thread).

    // Muscle activity calibration using the MYO armband
    if (Calib_Flag) 
    {    
        printf("Relax your muscles and get ready to lift the weight and hold it in upright position for 10 seconds...\n" );
        usleep(1*1000*1000);

        boost::thread Calib_thread(EMG_calib,&Calib_Flag);

        usleep(10*1000*1000);
        Calib_Flag = false;
        printf(GREEN "Calibration completed, you can relax now\n" DEFAULT);
        
        Calib_thread.interrupt();
        return 0;
    }

    // Myo thread for the actual experiment
    bool Myo_errFlag = false;
    boost::thread Myo_thread(Myo_receive,&Myo_errFlag);

    // connect to V-rep
    boost::thread vrep_thread(vrep_draw,&vrepFlag);

    // stop the program using the keyboard's Enter key
    bool stopFlag = false;
    boost::thread stop_thread(stop,&stopFlag);

    // Initializing FT sensor
    // boost::thread FT_thread(TCP_receive,&stopFlag, &FT_calibrated); // for WEISS KMS40 sensor 
    boost::thread FT_thread(Robotiq_ft_receive, dt, true, &stopFlag, &FT_calibrated); // for Robotiq FT 300 sensor
    while(!FT_calibrated){}; // wait until the force-sensor is calibrated

    // start the collision detection using VSM
    boost::thread vsm_thread(vsm_computations, &vsm, dxl_id_list, &stopFlag, &vsmFlag);

    // connect to robot
    SchunkPowerball pb;
    Kin kin;

    pb.update();
   
    // go to start pose
    Qs = pb.get_pos();
    // Qe = Data(-0.6992f,-0.2623f,1.4834f,0.0f,1.4363f,-0.6112f); // peg in a hole experiment - old
    Qe = makeVector(-30.0f,-10.0f,95.0f,0.0f,74.0f,105.0f) * M_PI/180; //
    // move the robot to start position
    std::vector< std::vector<double> > matrix;
    
    float T_travel = calculate_travel_time(Qs, Qe, 5); // used 20 deg/s instead of 72 deg/s for safety
    printf("Please wait! Estimated time to travel to start position is : %f \n", T_travel);
    
    kin.HerInter(Qs, Qdot, Qe, dt, T_travel, &matrix);
    for (int i = 0; i < matrix.size(); i++)
    {   
        timeLoop = std::chrono::system_clock::now();
        
        for (int j=0; j < matrix[0].size(); j++)
        {
            Q_interm[j] = matrix[i][j];
        }

        pb.set_pos(Q_interm);
        pb.update();

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ((dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }
        else {cout << "Communication Time Out!" << endl;}
    }

    // clear the matrix for next interpolation 
    matrix.clear();

    pb.update();
    
    // Get current pose of the robot
    Q = pb.get_pos();
    kin.FK_pos(Q,&X_init);

    // set velocity mode active
    pb.set_sdo_controlword(NODE_ALL,STATUS_OPERATION_ENABLED);
    pb.set_control_mode(MODES_OF_OPERATION_VELOCITY_MODE);
    pb.update();

    // Open recording file
    std::ofstream dataFile;

    dataFile.open("../data/VSM/peg-hole/admittance_stiff_" + std::to_string(dxl_init_position) + "_" + damping +".csv");
    dataFile << "Time,Q1,Q2,Q3,Q4,Q5,Q6,Qdot1,Qdot2,Qdot3,Qdot4,Qdot5,Qdot6,x,y,z,xdot,ydot,zdot,FT1,FT2,FT3,FT4,FT5,FT6,handle_pos1,handle_pos2,mag11, mag12, mag21,mag22,f1,f2, dxl_pos1, dxl_pos2" << endl;
    
    // dataFile << "Time,Q1,Q2,Q3,Q4,Q5,Q6,dQ1,dQ2,dQ3,dQ4,dQ5,dQ6,FT1,FT2,FT3,FT4,FT5,FT6,SimX,SimY,X,Y" << endl;

    cout << "Admittance loop started!" << endl;
    cout<< GREEN << "Start the experiment "<< DEFAULT  <<endl;

    //   cout << pb.get_status() << endl;
    Matrix<3,4,float> T_mat;
    while(!stopFlag)
    {

        timeLoop = std::chrono::system_clock::now();
        boost::thread threaded_computation(computations);
        // velocity saturation
        if (TooN::norm_inf(Qdot)>50*M_PI/180)
        {
            cout << "saturation!" << endl;
            // stopFlag = true; // Emergency stop
            Qdot = makeVector(1.0, 1.0, 1.0, 1.0, 1.0, 1.0)*0;
            pb.set_vel(Qdot);
        }
        else
        {
            pb.set_vel(Qdot);
        }

        pb.update();
        Q = pb.get_pos();
        Qdot_a = pb.get_vel();

        Start_record = true;
        // timestamp
        gettimeofday(&tv, NULL);
        curtime=tv.tv_sec;
        strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));
        dataFile << buffer << ":" << tv.tv_usec << ",";

        // dataFile<<Q[0]<<","<<Q[1]<<","<<Q[2]<<","<<Q[3]<<","<<Q[4]<<","<<Q[5]<<","
        //             <<Qdot_a[0]<<","<<Qdot_a[1]<<","<<Qdot_a[2]<<","<<Qdot_a[3]<<","<<Qdot_a[4]<<","<<Qdot_a[5]<<","
        //             <<FT[0]<<","<<FT[1]<<","<<FT[2]<<","<<FT[3]<<","<<FT[4]<<","<<FT[5]<< "," <<newPos[0]<<","<< newPos[1]<<","<< X[0]<<","<< X[1]<<endl;
        
        dataFile << Q[0] << "," << Q[1] << "," << Q[2] << "," << Q[3] << "," << Q[4] << "," << Q[5] << "," 
                    << Qdot[0] << "," << Qdot[1] << "," << Qdot[2] << "," << Qdot[3] << "," << Qdot[4] << "," << Qdot[5] << "," 
                    << X[0] << "," << X[1] << ","<< X[2] << ","<< vel[0] << ","<< vel[1] << ","<< vel[2] << ","
                    << FT[0] << "," << FT[1] << "," << FT[2] << "," << FT[3] << "," << FT[4] << "," << FT[5] << "," 
                    << linPot[0] << "," << linPot[1] << "," <<  mag_sep[0][0] << "," << mag_sep[0][1] << "," << mag_sep[1][0] << "," << mag_sep[1][1] << "," 
                    <<  vsm_force[0] << "," << vsm_force[1] << "," << dxl_pos[0] << "," << dxl_pos[1] <<"\n";

        threaded_computation.join();

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }
        // else {cout << "Communication Time Out!" << endl;}

        kin.FK_pos(Q,&X_init);
        // cout << Q <<endl;
    }

    dataFile.close(); // close file
    // stop the threads
    Myo_thread.interrupt();
    FT_thread.interrupt();
    stop_thread.interrupt();
    vrep_thread.interrupt();
    vsm_thread.interrupt();
    //   pb.shutdown_motors();
    pb.update();
    //   usleep(200*1000);
    cout << "Exiting ..." << endl;

}
