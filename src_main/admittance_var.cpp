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

using namespace::std;
using boost::asio::ip::tcp;
using namespace::TooN;

// global vars
Vector<6,float> FT;
simxFloat newPos[3]={0.0f,0.439,0.275};

// global vars computation
float dt = 0.003f;
Vector<6,float> Q = Zeros;
Vector<6,float> Qe = Zeros;
Vector<6,float> Qdot = Zeros;

// Cartesian admittance parameters_ one time define
Vector<6,float> Md_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*0.3f;
Matrix<6,6,double> Md_inv = Md_diag.as_diagonal();
Vector<6,float> Cd_diag = makeVector(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f)*1.0; // 30 for high and 80 for low admittance
Matrix<6,6,double> Cd = Cd_diag.as_diagonal();
Vector<6,float> vel = Zeros;

// temp vars
Vector<6,float> Qdot_a = Zeros;
Vector<2,float> v_xy = Zeros;


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
    F_modified[0] = FT[3]*10;
    F_modified[1] = FT[4]*10;

    // adaptation 1 (Fanny)
    Vector<6,float> v_vec = J*Qdot_a;
    v_xy[0] = v_vec[0];
    v_xy[1] = v_vec[1];
    float v_norm = TooN::norm_2(v_xy);
    float Dcoeff = 100.0f*exp(-5.0f*v_norm);
    Dcoeff = 30.0f;


    // coordination transformation
    vel = vel + dt*(Md_inv*(Rmat*(F_modified)) - Md_inv*Cd*Dcoeff*vel);
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

int main(int argc, char** argv)
{
    string SubName;
    // subject information
    cout << "What is subject name? ";
    cin >> SubName;
    cout << "Please wait " << SubName << endl;
    char in;
    cin.get(in);

    // Set sampling and timing options
    std::chrono::time_point<std::chrono::system_clock> timeLoop;
    std::chrono::duration<float> elaps_loop;

    // connect to V-rep
    boost::thread vrep_thread(vrep_draw);

    // Open recording file
    std::ofstream dataFile;
    dataFile.open("admittance_AdaptComb_"+SubName+".csv");
    dataFile << "TimeStamp , Q1, Q2, Q3, Q4, Q5, Q6, dQ1, dQ2, dQ3, dQ4, dQ5, dQ6, FT1, FT2, FT3, FT4, FT5, FT6" << endl;
    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;

    // connect to robot
    SchunkPowerball pb;
    pb.update();
    Q = pb.get_pos();

    // go to Start Pose trajectory
    Kin kin;
    std::vector< std::vector<double> > buf_mat;
    Qe = Data(0.0f,-M_PI/6,M_PI/2,0.0f,M_PI/3,0.0f);
    int numQ = kin.HerInter(Q,Qdot,Qe,dt,5.0,&buf_mat);
    Matrix<Dynamic,Dynamic,double> traj(numQ, 6);
    int col=0;
    int row=0;
    for (std::vector< std::vector<double> >::const_iterator it = buf_mat.begin(); it != buf_mat.end(); ++ it)
    {
        col = 0;
        for (std::vector<double>::const_iterator itit = it->begin(); itit != it->end(); ++ itit)
        {
            traj(row,col) = *itit;
            col++;
        }
        row++;
    }

    cout << "start homing ..." << endl;
    for (int n=0; n<numQ ; n++)
    {
        timeLoop = std::chrono::system_clock::now();

        pb.set_pos(traj[n]);
        pb.update();
        Q = pb.get_pos();

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }
    }

    // set velocity mode active
    pb.set_control_mode(MODES_OF_OPERATION_VELOCITY_MODE);
    pb.update();

    // Initializing FT sensor
    bool errFlag=false;
    boost::thread FT_thread(TCP_receive,&errFlag);


    // stop by Enter key thread
    bool stop_flag = false;
    boost::thread stop_thread(stop,&stop_flag);

    int cnt = 0;
    cout << "Admittance loop started!" << endl;
    //while((cnt<90/dt) && (!stop_flag))
    while(!stop_flag)
    {
        timeLoop = std::chrono::system_clock::now();

        // velocity saturation

        if (TooN::norm_inf(Qdot)>32.5*M_PI/180)
        {
            cout << "saturation!" << endl;
            //Qdot[n] = 32.5*M_PI/180*sgn(Qdot[n]);
        }
        else
        {
            pb.set_vel(Qdot);
        }

        boost::thread threaded_computation(computations);
        pb.update();
        Q = pb.get_pos();
        Qdot_a = pb.get_vel();

        // timestamp
        gettimeofday(&tv, NULL);
        curtime=tv.tv_sec;
        strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));
        dataFile << buffer << ":" << tv.tv_usec << ",";

        dataFile<<Q[0]<<","<<Q[1]<<","<<Q[2]<<","<<Q[3]<<","<<Q[4]<<","<<Q[5]<<","
                     <<Qdot_a[0]<<","<<Qdot_a[1]<<","<<Qdot_a[2]<<","<<Qdot_a[3]<<","<<Qdot_a[4]<<","<<Qdot_a[5]<<","
                      <<FT[0]<<","<<FT[1]<<","<<FT[2]<<","<<FT[3]<<","<<FT[4]<<","<<FT[5]<< endl;

        threaded_computation.join();

        cnt++;
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
    vrep_thread.interrupt();

    // set pb off since it is in vel mode
    pb.shutdown_motors();
    pb.update();

    usleep(1000*500);
    cout << "Exiting ..." << endl;

}
