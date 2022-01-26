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

#include <sys/stat.h>
// #include <sys/types.h>


using boost::asio::ip::tcp;
using namespace::TooN;

Color::Modifier red(Color::FG_RED);
Color::Modifier green(Color::FG_GREEN);
Color::Modifier def(Color::FG_DEFAULT);


V_rep vrep;

// global vars
Vector<6,float> FT;
simxFloat newPos[3]={0.0f,0.439,0.275};

// global vars computation
float dt = 0.005f;
Vector<6,float> Q = Zeros;
Vector<6,float> Qe = Zeros;
Vector<6,float> Qdot = Zeros;
Vector<6,float> Qdot_a = Zeros;
Vector<2,float> v_xy = Zeros;


// global strings
std::string taskType = "";
bool Start_record   = false;

// Cartesian admittance parameters one time define
Vector<6,float> Md_diag = makeVector(0.08f,0.08f,0.08f,10.0f,10.0f,10.0f)*1.0f; // for constant m
Matrix<6,6,double> Md_inv = Md_diag.as_diagonal();

Vector<6,float> Cd_diag = makeVector(80.0f,80.0f,80.0f,10.0f,10.0f,10.0f); // for High damping
// Vector<6,float> Cd_diag = makeVector(20.0f,20.0f,20.0f,6.0f,6.0f,6.0f); // for Low damping
Matrix<6,6,double> Cd = Cd_diag.as_diagonal();

Vector<6,float> vel = Zeros;
Matrix<6,6,float> R_F_offset = Data(cos(M_PI/2),-sin(M_PI/2),0, 0,0,0,
                                    sin(M_PI/2),cos(M_PI/2),0,  0,0,0,
                                    0,0,1,                       0,0,0,
                                    0,0,0, cos(M_PI/2),-sin(M_PI/2),0,
                                    0,0,0, sin(M_PI/2),cos(M_PI/2),0,
                                    0,0,0,                      0,0,1);



void stop(bool* flag){
    char in;
    std::cin.get(in);
    *flag = true;
    Start_record = false;
}

int sgn(float val)
{
    if (val > 0)
        return 1;
    else
        return -1;
}

int vrep_draw(const char* scene){
    // connect to vrep
    int res = -1;
    
    res = vrep.connect();
    if (res==-1)
    {
        std::cout << "V-REP Connection Error!" << std::endl;
        return 0;
    }
    vrep.loadScene(scene);
    vrep.startVrepSimulation();

    for (;;)
    {
        vrep.setSphere(&newPos[0]);
        // usleep(40*1000);
    }
}

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

    // prevent rotation and z motion
    F_modified[0] = -FT[3]*10.0f;// = FT[1];
    F_modified[1] = -FT[4]*10.0f;// = -FT[0];


    // no adaptation
    vel = vel + dt*(Md_inv*(Rmat*(R_F_offset*F_modified)) - Md_inv*Cd*vel);


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
    //vel = vel + dt*(Md_inv*(Rmat*(R_F_offset*F_modified)) - Md_inv*alpha*Cd*vel);

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


// Main file
int main(int argc, char** argv)
{   
    struct stat info;
    char in;

    // subject information
    // std::cout << green << "What is subject name? " << def;
    // std::cin >> SubName;
    // std::cin.get(in);

    std::cout << green << "What is task type? " << def;
    std::cin >> taskType;
    std::cout << red << "Please wait, Setting the task type to: " << taskType << def << std::endl;
    std::cin.get(in);
    
    
    // set the damping based on the task config (First letter: damping)
    if (std::tolower(taskType.c_str()[0]) == 'l'){
        Cd_diag = makeVector(10.0f,10.0f,10.0f,10.0f,10.0f,10.0f)*16.0f; // for High damping
        std::cout<< "Im in high damping condition"<<std::endl;}
    else if (std::tolower(taskType.c_str()[0]) == 'h')
        Cd_diag = makeVector(20.0f,20.0f,20.0f,6.0f,6.0f,6.0f) * 2; // for Low damping
    else 
        return 0;
    
    Cd = Cd_diag.as_diagonal();

    // set vrep scene based on the task config(second letter: Task type)
    std::string vrep_scene;
    if (std::tolower(taskType.c_str()[1]) == 'f')
        vrep_scene = "/home/srisadha/powerball/vrep_scenes/admittance_low_high_fine.ttt";
    else if (std::tolower(taskType.c_str()[1]) == 'g')
        vrep_scene = "/home/srisadha/powerball/vrep_scenes/admittance_low_gross.ttt";        
    else if (std::tolower(taskType.c_str()[1]) == 'c')
        vrep_scene = "/home/srisadha/powerball/vrep_scenes/admittance_comb.ttt";
    else
        return 0;


    // Set sampling and timing options
    std::chrono::time_point<std::chrono::system_clock> timeLoop;
    std::chrono::duration<float> elaps_loop;
    
    // stop the V-rep simulation if already running
    if (vrep.isConnected() == true)
        vrep.stopVrepSimulation(); 
        vrep.disconnect();
        
    usleep(2*1000*1000);    

    // connect to V-rep and start the simulation with new scene
    boost::thread vrep_thread(vrep_draw, vrep_scene.c_str());    
    usleep(1000*1000); // wait for the vrep to start simulation

    // Open recording file
    std::ofstream dataFile;
    dataFile.open("HRI_project/2021/admittance/admittance_" + taskType + ".csv");
    dataFile << "TimeStamp , Q1, Q2, Q3, Q4, Q5, Q6, dQ1, dQ2, dQ3, dQ4, dQ5, dQ6, Fx, Fy, Fz, Mx, My, Mz, Joint1_torque, Joint2_torque, Joint3_torque, Joint4_torque, Joint5_torque, Joint6_torque " << std::endl;

    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;

    // connect to robot
    SchunkPowerball pb;
    pb.update();
    Q = pb.get_pos();
    Vector<6, float> Tor = pb.get_tor();

    // stop by Enter key thread
    bool stop_flag = false;
    boost::thread stop_thread(stop,&stop_flag);

    // go to start pose
    // Qe = Data(0.0f,-M_PI/6,M_PI/2,0.0f,M_PI/3,0.0f);
    Qe = Data(0.0f,-33.35*M_PI/180, 68*M_PI/180, 0.0f, 77.46*M_PI/180, 0.0f);
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

    // set velocity mode active
    pb.set_control_mode(MODES_OF_OPERATION_VELOCITY_MODE);
    pb.update();

    // Initializing FT sensor
    bool errFlag=false;
    boost::thread FT_thread(TCP_receive,&errFlag);

    int cnt = 0;
    std::cout << "Admittance loop started!" << std::endl;
    
    // start recording the EMG data
    Start_record = true;

    while((cnt<60/dt) && (!stop_flag))
    {
        timeLoop = std::chrono::system_clock::now();

        boost::thread threaded_computation(computations);
        
        // velocity saturation
        if (TooN::norm_inf(Qdot)>32.5*M_PI/180)
        {
            std::cout << "saturation!" << std::endl;
            for (int joint = 0; joint < 6; joint++){
                Qdot[joint] = 0*M_PI/180*sgn(Qdot[joint]);    
            }
        }
        else
        {
            pb.set_vel(Qdot);
        }

        pb.update();
        Q = pb.get_pos();
        Qdot_a = pb.get_vel();
        Tor = pb.get_tor();

        // timestamp
        gettimeofday(&tv, NULL);
        curtime=tv.tv_sec;
        strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));
        dataFile << buffer << ":" << tv.tv_usec << ",";

        dataFile<<Q[0]<<","<<Q[1]<<","<<Q[2]<<","<<Q[3]<<","<<Q[4]<<","<<Q[5]<<","
                     <<Qdot_a[0]<<","<<Qdot_a[1]<<","<<Qdot_a[2]<<","<<Qdot_a[3]<<","<<Qdot_a[4]<<","<<Qdot_a[5]<<","
                      <<FT[0]<<","<<FT[1]<<","<<FT[2]<<","<<FT[3]<<","<<FT[4]<<","<<FT[5]<< ","
                      <<Tor[0]<<","<<Tor[1]<<","<<Tor[2]<<","<<Tor[3]<<","<<Tor[4]<<","<<Tor[5]<<std::endl;

        threaded_computation.join();
        
        cnt++;
        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }
        else {
            std::cout << "Communication Time Out!" << std::endl;
        }

    }


    dataFile.close(); // close file
    // kill FT thread
    FT_thread.interrupt();
    stop_thread.interrupt();   // kill the thread
    vrep_thread.interrupt();

    vrep.stopVrepSimulation();
    usleep(500*1000);
    // set pb off since it is in vel mode
    pb.shutdown_motors();
    pb.update();
    usleep(1000*1000);
    std::cout << "Exiting ..." << std::endl;

}
