#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>
#include <chrono>
#include "powerball/schunk_powerball.h"
#include "vrep/v_repClass.h"
#include "powerball/schunk_kinematics.h"
#include <TooN/LU.h>
#include <TooN/SVD.h>
#include "utils/utils.h"

#include "myolinux/myoclient.h" // myo band headers
#include "myolinux/serial.h"
#include <ostream> // included for color output to the terminal
#include <cinttypes>

using namespace::std;
using boost::asio::ip::tcp;
using namespace::TooN;
using namespace myolinux;

Color::Modifier red(Color::FG_RED);
Color::Modifier green(Color::FG_GREEN);
Color::Modifier def(Color::FG_DEFAULT);

// global vars - MYO
Vector<8,float> EMG;
// Vector<4,float> ORI;
// Vector<3,float> ACC;
// Vector<3,float> GYR;
bool Start_record = false;

// global vars
Vector<6,float> FT;
simxFloat newPos[3]={0.0f,0.439,0.275};
Vector<6,float> joints_vrep = Data(0,0,0,0,0,0);

// global vars computation
float dt = 0.004f;
Vector<6,float> Q = Zeros;
Vector<6,float> Qe = Zeros;
Vector<6,float> Qdot = Zeros;
Vector<6,float> Qdot_a = Zeros;
Vector<2,float> v_xy = Zeros;
Vector<3,float> X_init = Zeros;
Vector<3,float> X = Zeros;
string SubName;

// Cartesian admittance parameters_ one time define
Vector<6,float> Md_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*0.3f; // for constant m
//Vector<6,float> Md_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*(1.0f/15.0f); // for var m (min:3.5 max:)
Matrix<6,6,double> Md_inv = Md_diag.as_diagonal();

//Vector<6,float> Cd_diag = makeVector(1.2f,1.0f,1.0f,1.0f,1.0f,1.0f)*80; // for fine Low
// Vector<6,float> Cd_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*25; // for fine High
Vector<6,float> Cd_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*90; // for gross Low
//Vector<6,float> Cd_diag = makeVector(1.4f,1.0f,1.0f,1.0f,1.0f,1.0f)*30; // for gross High
// Vector<6,float> Cd_diag = makeVector(1.2f,1.0f,1.0f,1.0f,1.0f,1.0f)*30; // for comb High
//Vector<6,float> Cd_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*1.0f; // for velocity-based adapt

Matrix<6,6,double> Cd = Cd_diag.as_diagonal();
Vector<6,float> vel = Zeros;
Matrix<6,6,float> R_F_offset = Data(cos(M_PI/2),-sin(M_PI/2),0, 0,0,0,
                                    sin(M_PI/2),cos(M_PI/2),0,  0,0,0,
                                    0,0,1,                       0,0,0,
                                    0,0,0, cos(M_PI/2),-sin(M_PI/2),0,
                                    0,0,0, sin(M_PI/2),cos(M_PI/2),0,
                                    0,0,0,                      0,0,1);



// Initializing the myo connection
myo::Client client(Serial{"/dev/ttyACM0", 115200});

void stop(bool* flag){
    char in;
    cin.get(in);
    *flag = true;
}

int sgn(float val)
{
    if (val > 0)
        return 1;
    else
        return -1;
}

int vrep_draw(){
    // connect to vrep
    int res = -1;
    V_rep vrep;
    res = vrep.connect();
    if (res==-1)
    {
        cout << "V-REP Connection Error!" << endl;
        return 0;
    }

    for (;;)
    {
        vrep.setSphere(&newPos[0]);
        usleep(40*1000);
    }
}

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
    newPos[0] = -(X[1]-X_init[1])+0.06;
    newPos[1] = X[0]-X_init[0];
    newPos[2] = 0.01;//X[2];

    Rmat.slice<0,0,3,3>() = R.slice<0,0,3,3>();
    Rmat.slice<3,3,3,3>() = R.slice<0,0,3,3>();

    // Rotation about z axis is prevented - Modify according to the force sensor is attached to the end-effector
    F_modified[0] = -FT[0];
    F_modified[1] = -FT[1];
    F_modified[2] =  0;//FT[2];
    // F_modified[3] = -FT[3];
    // F_modified[4] = -FT[4];

    // prevent rotation and z motion
    // F_modified[0] = -FT[1];
    // F_modified[1] = FT[0];
    // F_modified[2] = FT[2];
    // // F_modified[0] = -FT[3]*10.0f;
    // F_modified[1] = -FT[4]*10.0f;


    // no adaptation
    //vel = vel + dt*(Md_inv*(Rmat*(R_F_offset*F_modified)) - Md_inv*Cd*vel);


    // adaptation 1 (geometrical)
    // float C1=0.075, C2=0.125, alpha=1.0f, xx=0.0;
    // if (X[1]<=C1)
    // {
    //     alpha=1.0f;
    // } else if ((X[1]>C1) && (X[1]<C2))
    // {
    //     xx = (X[1]-C1)*4/(C2-C1) - 2;
    //     alpha = tanh(xx)/0.964*1.5 + 2.5;
    // } else
    // {
    //     alpha=3.5f;
    // }

    float alpha = 1;
    vel = vel + dt*(Md_inv*(Rmat*(F_modified)) - Md_inv*alpha*Cd*vel);
    // vel = vel + dt*(Md_inv*(Rmat*(R_F_offset*F_modified)) - Md_inv*alpha*Cd*vel);

    /*
    // adaptation velocity-based (Fanny)
    Vector<6,float> v_vec = J*Qdot_a;
    v_xy[0] = v_vec[0];
    v_xy[1] = v_vec[1];
    float v_norm = TooN::norm_2(v_xy);
    float Dcoeff = 100.0f*exp(-5.0f*v_norm);
    vel = vel + dt*(Md_inv*(Rmat*(R_F_offset*F_modified)) - Md_inv*Cd*Dcoeff*vel);
    */

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
        //msg="F()\n";
        //socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
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
        cout<< "Unable to connect to Myo band"<<endl;
    }else{
        cout<< red<<"Please wait while establishing connection with the Myo band .........."<<def<<endl;
    }

    //client.vibrate(myo::Vibration::Medium);// Vibrate Myo band
    client.setSleepMode(myo::SleepMode::NeverSleep);// Set sleep mode
    client.setMode(myo::EmgMode::SendEmg, myo::ImuMode::SendData, myo::ClassifierMode::Disabled);// Read EMG and IMU
    client.onEmg([](myo::EmgSample sample)
    {
        for (std::size_t i = 0; i < 8; i++) {
            EMG[i] = static_cast<int>(sample[i]);
        }
    });

    // client.onImu([](myo::OrientationSample ori, myo::AccelerometerSample acc, myo::GyroscopeSample gyr)
    // {
    //     for (std::size_t i = 0; i < 4 ; i++){
    //         ORI[i] = ori[i];
    //         if (i < 3){
    //             ACC[i] = acc[i];
    //             GYR[i] = gyr[i];
    //         }
    //     }
    // });
    // auto name = client.deviceName();
    // std::cout << name << std::endl;
    return client;
}

void Myo_receive(bool *errFlag)//(myo::Client client)
{
    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;

    // Open recording file
    std::ofstream EMGFile;
    EMGFile.open("EMG"+SubName+".csv");
    EMGFile << "Time" <<","<<"EMG1" <<","<< "EMG2" <<","<< "EMG3" <<","<< "EMG4" <<","<< "EMG5" <<","<< "EMG6" <<","<< "EMG7" <<","<< "EMG8" <<"," << endl;

    for(;;){
        try {
            client.listen();
            if(Start_record){
              gettimeofday(&tv, NULL);
              curtime=tv.tv_sec;
              strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));
              EMGFile << buffer << ":" << tv.tv_usec << ",";
              EMGFile<< EMG[1]<< ","<< EMG[2]<< ","<< EMG[3]<< ","<< EMG[4]<< ","<< EMG[5]<< ","<< EMG[6]<< ","<< EMG[7]<< ","<< EMG[8]<<endl;
            }
        }
        catch(myo::DisconnectedException &) {
            std::cout << "Disconnected" << std::endl;
        }
    }
}


int main(int argc, char** argv)
{

    // subject information
    cout << "What is subject name? ";
    cin >> SubName;
    cout << "Please wait " << SubName << endl;
    char in;
    cin.get(in);

    // Set sampling and timing options
    std::chrono::time_point<std::chrono::system_clock> timeLoop;
    std::chrono::duration<float> elaps_loop;

    // Initialize Myo band
    myo::Client client = Myo_init(); // initializing the myo band here works (sometimes it works here and sometimes after initializing the admittance control thread).
    bool errFlag = false;
    boost::thread Myo_thread(Myo_receive,&errFlag);

    // connect to V-rep
    boost::thread vrep_thread(vrep_draw);

    // Open recording file
    std::ofstream dataFile;
    dataFile.open("admittance_EMG"+SubName+".csv");
    dataFile << "Time,Q1,Q2,Q3,Q4,Q5,Q6,dQ1,dQ2,dQ3,dQ4,dQ5,dQ6,FT1,FT2,FT3,FT4,FT5,FT6,SimX,SimY,X,Y" << endl;

    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;

    // connect to robot
    SchunkPowerball pb;
    pb.update();
    Q = pb.get_pos();

    // stop by Enter key thread
    bool stop_flag = false;
    boost::thread stop_thread(stop,&stop_flag);

    // go to start pose
    // Qe = Data(0.0f,-M_PI/6,M_PI/2,0.0f,M_PI/3,0.0f);
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

    // Get current pose of the robot
    Kin kin;
    kin.FK_pos(Q,&X_init);
    // set velocity mode active
    pb.set_control_mode(MODES_OF_OPERATION_VELOCITY_MODE);
    pb.update();

    // Initializing FT sensor
    // bool errFlag=false;
    boost::thread FT_thread(TCP_receive,&errFlag);

    // int cnt = 0;
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
            //Qdot[n] = 32.5*M_PI/180*sgn(Qdot[n]);
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

        dataFile<<Q[0]<<","<<Q[1]<<","<<Q[2]<<","<<Q[3]<<","<<Q[4]<<","<<Q[5]<<","
                     <<Qdot_a[0]<<","<<Qdot_a[1]<<","<<Qdot_a[2]<<","<<Qdot_a[3]<<","<<Qdot_a[4]<<","<<Qdot_a[5]<<","
                      <<FT[0]<<","<<FT[1]<<","<<FT[2]<<","<<FT[3]<<","<<FT[4]<<","<<FT[5]<< "," <<newPos[0]<<","<< newPos[1]<<","<< X[0]<<","<< X[1]<<endl;

        threaded_computation.join();

        // cnt++;
        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }
        else {
            cout << "Communication Time Out!" << endl;
        }

    }


    dataFile.close(); // close file
    // kill FT thread
    FT_thread.interrupt();
    stop_thread.interrupt();   // kill the thread
    Myo_thread.interrupt();
    vrep_thread.interrupt();
    // set pb off since it is in vel mode
    pb.shutdown_motors();
    pb.update();
    usleep(1000*500);
    cout << "Exiting ..." << endl;

}
