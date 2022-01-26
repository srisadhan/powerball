#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <chrono>
#include <typeinfo> 
#include <fstream>
#include <vector>
#include <typeinfo>

// boost headers
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>

// Powerball headers
#include "powerball/schunk_powerball.h"
#include "powerball/schunk_kinematics.h"

// V-Rep headers
#include "vrep/v_repClass.h"

#include <TooN/LU.h>
#include <TooN/SVD.h>
#include "utils/utils.h"
#include "utils/powerball_utils.h"

using namespace::std;

float dt = 0.005f; // sampling time


// enter key press terminates the code
void stop(bool* flag){
    char in;
    cin.get(in);
    *flag = true;
}

int main(int argc, char**argv)
{   
    float max_Qdot = 10;
    bool robot_Flag = true; // true if robot should be moved

    if (argc > 1){
        if (strcmp(argv[1], "-h") == 0){
            printf(GREEN "./straight_line_collision arg1 arg2 arg3 \n" );
            printf("arg1: -vo : only if want to display trajectory on vrep \n   -v: sets trajectory both on v-rep and robot \n");
            printf("arg2: max velocity of the robot joint in deg/s (<70) \n" DEFAULT);
            return 0;
        }else{
            // convert the char arguments to int using stringstream
            stringstream strValue;

            if (strcmp(argv[1], "-vo") == 0) {
                robot_Flag = false;
            }
            if (argc > 2){
                // second argument as the force threshold value
                strValue << argv[2];
                strValue >> max_Qdot; 
            }
        }
    }

    // Check for V-REP connection
    int res = -1;
    V_rep vrep;
    res = vrep.connect();
    if (res==-1)
    {
        std::cout << RED <<"V-REP Connection Error" << DEFAULT << std::endl;
        return 0;
    }

    // Stop thread
    bool stopFlag = false;
    boost::thread stop_thread(stop, &stopFlag);

    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;
    std::chrono::time_point<std::chrono::system_clock> timeLoop;
    std::chrono::duration<float> elaps_loop;

    Vector<6,float> Qs;
    Vector<6,float> Qe;

    
    Qs = makeVector(0.0f,-20.0f,70.0f,0.0f,-1.0f,0.0f) * M_PI/180; // this pos is good when VSM is not present
    Qe = makeVector(0.0f,-86.0f,-70.0f,0.0f,71.0f,0.0f) * M_PI/180; //


    Vector<6,float> Q;
    Vector<6, float> Q_interm;
    Vector<6,float> Qd = Zeros;
    Vector<6,float> Qdot = Zeros;
    Vector<6,float> F;
    Vector<6,float> Torque;
    Vector<3,float> pos;
    Vector<3,float> vel;
    Matrix<6, 6, float> J;

    SchunkPowerball pb;
    Kin kin;

    Q = pb.get_pos();
    // move the robot to start position
    std::vector< std::vector<double> > matrix;
    
    float T_travel = calculate_travel_time(Q, Qs, 10); // used 20 deg/s instead of 72 deg/s for safety
    printf("Please wait! Estimated time to travel to start position is : %f \n", T_travel);
    
    kin.HerInter(Q, Qd, Qs, dt, T_travel, &matrix);
    std::cout<< matrix.size() << std::endl;

    for (int i = 0; i < matrix.size(); i++)
    {   
        if (!stopFlag){
            timeLoop = std::chrono::system_clock::now();
            
            for (int j=0; j < matrix[0].size(); j++)
            {
                Q_interm[j] = matrix[i][j];
            }

            if (vrep.isConnected())
            {
                vrep.setq(Q_interm);
            }else
            {
                std::cout << "Error connecting to V-Rep" << std::endl;
            }
            
            if (robot_Flag){
                pb.set_pos(Q_interm);
                pb.update();
            }
        
            elaps_loop = std::chrono::system_clock::now() - timeLoop;
            if ((dt-elaps_loop.count()) > 0 ) {
                usleep( (dt-elaps_loop.count())*1000*1000 );
            }
            else {std::cout << "Communication Time Out!" << std::endl;}
        }
    }

    // clear the matrix for next interpolation 
    matrix.clear();

    pb.update();

    Q = pb.get_pos();

    T_travel = calculate_travel_time(Qs, Qe, max_Qdot); // used 20 deg/s instead of 72 deg/s for safety
    std::cout << max_Qdot << "," << T_travel << "\n";
    kin.HerInter(Qs, Qd, Qe, dt, T_travel, &matrix);

    
    for(int i = 0; i < matrix.size(); i++)
    {   
        if (stopFlag) 
        {   
            printf(RED "Terminate robot program! stopping the robot \n" DEFAULT);
            pb.update();
            break;
        }
        else{
            timeLoop = std::chrono::system_clock::now();
            for (int j=0; j < matrix[0].size(); j++)
            {
                Q_interm[j] = matrix[i][j];
            }

            if (vrep.isConnected())
            {
                vrep.setq(Q_interm);
            }else
            {
                std::cout << "Error connecting to V-Rep" << std::endl;
            }

            if (robot_Flag){
                pb.set_pos(Q_interm);
                pb.update();
            }

            Q = pb.get_pos();

            elaps_loop = std::chrono::system_clock::now() - timeLoop;
            if ((dt-elaps_loop.count()) > 0 ) {
                usleep( (dt-elaps_loop.count())*1000*1000 );
            }
            else {cout << "Communication Time Out!" << endl;}
        }
    }
    
    // shutdown the powerball motors
    pb.update();
    
    // stop threads
    stop_thread.interrupt();

    // pb.shutdown_motors();   

    return 0;
}