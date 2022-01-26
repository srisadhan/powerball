#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <chrono>
#include <typeinfo> 
#include <fstream>
#include <vector>
#include <typeinfo>

// #include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library
// #include <phidget21.h>                                      // Uses phidget 21 library
#include "vsm/vsm_control.h"

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
#include "utils/vsm_utils.h"

// Control table address for new dynamixel - XM430-W210R
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

using namespace::std;
using boost::asio::ip::tcp;

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

// Vector<6, float> FT = Zeros;
Vector<6, float> Tor = Zeros;

float K_virtual = 20;                       // initialize the virtual stiffness value for adaptation 
float M_virtual = 2.0;
float C_virtual = 100;
float force_threshold = 15.0;

// enter key press terminates the code
void stop(bool* flag){
    char in;
    cin.get(in);
    *flag = true;
}

// VSM collision detection 
void vsm_collision_detection(VSMControl *vsm, std::vector<int> dxl_id_list, float force_threshold, bool *stopFlag, bool *vsmFlag)
{   
    float external_force; 
    std::vector<float> sep;

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
                
                // Raise a stop flag if collision is detected
                if (external_force > force_threshold)
                {   
                    printf(RED "Collision detected using VSM! \n" DEFAULT);
                    *stopFlag = true;
                }
            }
        }
    }
}


// Calculate end-effector force from the robot joint torques
// FIXME: fix this part of the code, check for the correctness
Vector<6, float> robot_force_estimation(Vector<6,float> Q, Matrix <6, 6, float> J, Vector<6, float> Torque)
{
    Vector<6, float> end_force;
        
    // solve inv(J.T())*Torque using LU
    SVD<6,6,float> luJ(J.T());
    end_force = luJ.backsub(Torque);
    return end_force;
}


// ------------------------------------------------------------------ //
// Impact adaptation strategies
// ------------------------------------------------------------------ //
// Set the active magnets to maximum posssible position to avoid impact transfer
// uint32_t impact_adaptation(uint32_t dxl_pos_t, uint32_t new_dxl_pos, float handle_position, float force_threshold)
// {
//     float res_force = abs(resForce(dxl_pos_t, handle_position));
//     if (res_force > force_threshold){ 
//             new_dxl_pos = VSM_MAX_POS - 25; // set the dynamixel to the fathest position minus 25 for the safety
//         }
//     return new_dxl_pos;
// }

// // detect if the force exceeds the threshold and change the disableTorque flag to true
// void detect_impact(uint32_t dxl_pos_t, float handle_position, float force_threshold, bool *disableTorque)
// {   
//     float res_force = resForce(dxl_pos_t, handle_position);
//     printf("Handle position: %f, Resultant force: %f\n", linPot, res_force);

//     if (abs(res_force) > force_threshold){ 
//             *disableTorque = true;
//         }
// }

// // ------------------------------------------------------------------ //
// // Adaptation strategies
// // ------------------------------------------------------------------ //
// // The adaptation strategy will always try to move the magnet to the dxl_initial_pos once the external force is removed 
// // ((1)) |F1 - F2|/K_virtual = x 
// uint32_t spring_adaptation(uint32_t dxl_pos_t1, uint32_t dxl_pos_t, uint32_t new_dxl_pos, float handle_position, float K_virtual, float force_threshold)
// {   
//     if (K_virtual < 1) K_virtual = 1; // prevent division by zero error

//     float res_force = abs(resForce(dxl_pos_t, handle_position));

//     if (res_force > force_threshold){
//         float estimated_pos = res_force / K_virtual ;
//         new_dxl_pos = dxl_pos_t1 +  estimated_pos / dxl_resolution;
//     }else{
//         new_dxl_pos = dxl_pos_t;
//     }
    
//     new_dxl_pos = set_safe_dxl_pos(new_dxl_pos);
//     return new_dxl_pos;
// }

// // Admittance adaptation strategy
// // ((2)) (|F1 - F2| - C_virtual * Vt)/M_virtual  * dt = Vt+1
// uint32_t admittance_adaptation(uint32_t dxl_pos_t1, uint32_t dxl_pos_t, float handle_position, float M_virtual, float C_virtual, float force_threshold)
// {   
//     uint32_t new_dxl_pos;
//     float res_force;
    
//     res_force = abs(resForce(dxl_pos_t, handle_position));
    
//     if (res_force > force_threshold){
//         // float estimated_pos = (2 * res_force * dt * dt + 4*M_virtual * dxl_pos_t + (2*M_virtual - C_virtual*dt) * dxl_pos_t1) / (2*M_virtual + C_virtual * dt);
//         float estimated_pos = (2 * res_force * dt * dt) / (2*M_virtual + C_virtual * dt);
        
//         new_dxl_pos = dxl_pos_t + estimated_pos / dxl_resolution;
//     }else{
//         new_dxl_pos = dxl_pos_t;
//     }
//     printf("force:%f, current_pos:%d, new_pos:%d, handle_pos:%f\n", res_force, dxl_pos_t, new_dxl_pos, handle_position);
//     new_dxl_pos = set_safe_dxl_pos(new_dxl_pos);
//     return new_dxl_pos;
// }


int main(int argc, char**argv)
{   
    float max_Qdot = 10;
    bool vsmFlag = true; // flag to use the VSM or not
    bool FT_calibrated = false; 

    // Stop thread
    bool stopFlag = false;
    boost::thread stop_thread(stop, &stopFlag);

    // #NOTE: This location gives the force-sensor enough time to tare and get ready for recording
    // Initializing Force-Torque sensor
    boost::thread FT_thread(TCP_receive, &stopFlag, &FT_calibrated); // for Weiss kms 40 sensor
    // boost::thread FT_thread(Robotiq_ft_receive, dt, true, &stopFlag, &FT_calibrated); // for robotiq FT 300 sensor
    while(!FT_calibrated){}; // wait until the force-sensor is calibrated


    // flag robot's motion when impact is detected
    bool disableTorque = false;

    if (argc > 1){
        if (strcmp(argv[1], "-h") == 0){
            printf(GREEN "./straight_line_collision arg1 arg2 arg3 \n" );
            printf("arg1: -no_vsm - if you don't want to use VSM (or) \n      dynamixel initial position value in between [100, 2048] \n");
            printf("arg2: force threshold for the adaptation strategy \n");
            printf("arg3: max velocity of the robot joint in deg/s (<70) \n" DEFAULT);
            return 0;
        }else{
            // convert the char arguments to int using stringstream
            stringstream strValue1, strValue2, strValue3;

            if (strcmp(argv[1], "-no_vsm") == 0) {
                vsmFlag = false;
            } else{
                // first argument as dynamixel_goal_position        
                strValue1 << argv[1];
                strValue1 >> dxl_init_position;

                if ((dxl_init_position < 100) || (dxl_init_position > 2048))
                {
                    printf("Please provide the values in the range [100, 2048] \n");
                    return 1;
                }
            }

            if (argc > 2){
                // second argument as the force threshold value
                strValue2 << argv[2];
                strValue2 >> force_threshold;

                if (argc > 3)
                {
                    // second argument as the force threshold value
                    strValue3 << argv[3];
                    strValue3 >> max_Qdot;                    
                }
            }
        }
    }

    VSMControl vsm(BAUDRATE, DEVICENAME, PROTOCOL_VERSION, ADDR_MX_TORQUE_ENABLE, ADDR_MX_GOAL_POSITION, ADDR_MX_PRESENT_POSITION);
    std::vector<int> dxl_id_list;
    if (vsmFlag)
    {
        int dxl_comm_result = COMM_TX_FAIL;                             // Communication result
        uint8_t dxl_error = 0;                                          // Dynamixel error
            
        // connect to the available dynamixel
        if (!vsm.dxl_enable()) return 0;
        printf("Number of dyn connected: %d \n", vsm.get_connected_dxl_count(&dxl_id_list));

        vsm.xm_set_pos(dxl_id_list[0], (VSM_MIN_POS+300), VSM_MIN_POS, VSM_MAX_POS);
        vsm.xm_set_pos(dxl_id_list[1], (VSM_MIN_POS+300), VSM_MIN_POS, VSM_MAX_POS);
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

    Vector<6,float> Qs;
    Vector<6,float> Qe;

    if (vsmFlag){
        //Only VSM
        // makeVector(20.0f,-22.0f,88.0f,0.0f,68.0f,12.0f) * M_PI/180; //
        // makeVector(-14.0f,-22.0f,88.0f,0.0f,68.0f,-18.0f) * M_PI/180; //

        // with force sensor and VSM and collision detection "ON"
        Qs = makeVector(20.0f,-22.0f,78.0f,0.0f,78.0f,22.0f) * M_PI/180; //
        Qe = makeVector(-50.0f,-22.0f,78.0f,0.0f,78.0f,-50.0f) * M_PI/180; //
    }else {
        Qs = makeVector(20.0f,-25.0f,95.0f,0.0f,58.0f,22.0f) * M_PI/180; // this pos is good when VSM is not present
        Qe = makeVector(-50.0f,-25.0f,95.0f,0.0f,58.0f,-50.0f) * M_PI/180; //
    }

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
    
    // uint32_t mode = pb.get_control_mode();
    // printf("Powerball current operation mode is: %x \n", mode);
    // if (mode != MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE)
    // {   
    //     // change the operation mode to position
    //     pb.set_sdo_controlword(NODE_ALL,STATUS_OPERATION_ENABLED);
    //     pb.set_control_mode(MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE);
    // }
    
    // for (int i=3; i < 9; i++) pb.unbrake(i);
    // pb.update();

    Q = pb.get_pos();
    // move the robot to start position
    std::vector< std::vector<double> > matrix;
    
    float T_travel = calculate_travel_time(Q, Qs, 10); // used 20 deg/s instead of 72 deg/s for safety
    printf("Please wait! Estimated time to travel to start position is : %f \n", T_travel);
    
    kin.HerInter(Q, Qd, Qs, dt, T_travel, &matrix);
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

    // Open recording file
    std::ofstream dataFile;
    if(vsmFlag)
    {
        dataFile.open("../data/VSM/collision/collision_dxl_" + std::to_string(dxl_init_position) + "_velocity_" + std::to_string(int(max_Qdot)) + ".csv");
    }else
    {
        dataFile.open("../data/VSM/collision/collision_dxl_noVSM_velocity_" + std::to_string(int(max_Qdot)) + ".csv");
    }
    
    
    dataFile << "Time,Q1,Q2,Q3,Q4,Q5,Q6,Qdot1,Qdot2,Qdot3,Qdot4,Qdot5,Qdot6,x,y,z,xdot,ydot,zdot,Torque1,Torque2,Torque3,Torque4,Torque5,Torque6,F1,F2,F3,F4,F5,F6,FT1,FT2,FT3,FT4,FT5,FT6,handle_pos1,handle_pos2,mag11, mag12, mag21,mag22,f1,f2, dxl_pos1, dxl_pos2" << endl;
    
    uint32_t new_dxl_pos = dxl_pos_t;
    Q = pb.get_pos();

    T_travel = calculate_travel_time(Q, Qe, max_Qdot); // used 20 deg/s instead of 72 deg/s for safety
    cout << max_Qdot << "," << T_travel << "\n";
    kin.HerInter(Q, Qd, Qe, dt, T_travel, &matrix);
    
    // start the collision detection using VSM
    boost::thread vsm_thread(vsm_collision_detection, &vsm, dxl_id_list, force_threshold, &stopFlag, &vsmFlag);
    
    for(int i = 0; i < matrix.size(); i++)
    {   
        if (stopFlag) 
        {   
            printf(RED "Collision detected! stopping the robot \n" DEFAULT);
            pb.update();
            break;
        }
        else{
            timeLoop = std::chrono::system_clock::now();
            for (int j=0; j < matrix[0].size(); j++)
            {
                Q_interm[j] = matrix[i][j];
            }

            pb.set_pos(Q_interm);

            pb.update();
            Q = pb.get_pos();
            kin.FK_pos(Q, &pos);

            Torque = pb.get_tor();
            kin.Jacob(Q,  &J);
            Qdot = pb.get_vel();
            vel = J.slice<0,0,3,6>() * Qdot;
            F = robot_force_estimation(Q, J,Torque);
            // vsm_thread.join();
            // printf("Estimated torque: %d, %d, %d, %d, %d, %d \n", int(F[0]), int(F[1]), int(F[2]), int(F[3]), int(F[4]), int(F[5]));
        //     // detect if force exceeds a threshold
        //     detect_impact(dxl_pos_t, linPot, force_threshold, &disableTorque);

        //     // Read the torque status from dynamixel
        //     packetHandler->read1ByteTxRx(portHandler, DXL_ID1, ADDR_MX_TORQUE_ENABLE, &torque_status);
        //     // printf("Torque status: %d", torque_status);

        //     if (!disableTorque)
        //     {   
        //         // Type 1 adaptation - Always return to initial position
        //         // new_dxl_pos = spring_adaptation(dxl_init_position, dxl_pos_t, linPot, K_virtual, force_threshold);

        //         // Type 1 adaptation - Spring only 
        //         // new_dxl_pos = spring_adaptation(dxl_pos_t1, dxl_pos_t, new_dxl_pos, linPot, K_virtual, force_threshold);
                
        //         // Type 2 adaptation - Admittance control
        //         // new_dxl_pos = admittance_adaptation(dxl_pos_t1, dxl_pos_t, linPot, M_virtual, C_virtual, force_threshold);

        //         // printf("new pos: %d \n", new_dxl_pos);
        //         dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID1, ADDR_MX_GOAL_POSITION, new_dxl_pos, &dxl_error);
        //         dxl_comm_status(dxl_comm_result, dxl_error, packetHandler);

        //     }else if (disableTorque && torque_status){
                
        //         // disable torque and disconnect dynamixel 
        //         dxl_disable(dxl_comm_result, dxl_error, dxl_pos_t, portHandler, packetHandler);
        //     }            

        //     // Read present position
        //     dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID1, ADDR_MX_PRESENT_POSITION, &dxl_pos_t, &dxl_error);
        //     dxl_comm_status(dxl_comm_result, dxl_error, packetHandler);

        //     dxl_pos_t1 = dxl_pos_t;

            // timestamp
            gettimeofday(&tv, NULL);
            curtime=tv.tv_sec;
            strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));
            dataFile << buffer << ":" << tv.tv_usec << ",";

            dataFile << Q[0] << "," << Q[1] << "," << Q[2] << "," << Q[3] << "," << Q[4] << "," << Q[5] << "," 
                    << Qdot[0] << "," << Qdot[1] << "," << Qdot[2] << "," << Qdot[3] << "," << Qdot[4] << "," << Qdot[5] << "," 
                    << pos[0] << "," << pos[1] << ","<< pos[2] << ","<< vel[0] << ","<< vel[1] << ","<< vel[2] << ","
                    << Torque[0] << "," << Torque[1] << "," << Torque[2] << "," << Torque[3] << "," << Torque[4] << "," << Torque[5] << "," 
                    << F[0] << "," << F[1] << "," << F[2] << "," << F[3] << "," << F[4] << "," << F[5] << "," 
                    << FT[0] << "," << FT[1] << "," << FT[2] << "," << FT[3] << "," << FT[4] << "," << FT[5] << "," 
                    << linPot[0] << "," << linPot[1] << "," <<  mag_sep[0][0] << "," << mag_sep[0][1] << "," << mag_sep[1][0] << "," << mag_sep[1][1] << "," 
                    <<  vsm_force[0] << "," << vsm_force[1] << "," << dxl_pos[0] << "," << dxl_pos[1] <<"\n";

            elaps_loop = std::chrono::system_clock::now() - timeLoop;
            if ((dt-elaps_loop.count()) > 0 ) {
                usleep( (dt-elaps_loop.count())*1000*1000 );
            }
            else {cout << "Communication Time Out!" << endl;}

            if ((!vsmFlag) & (abs(FT[1]) > force_threshold))
            {
                stopFlag = true;
                printf("Collision detected using Force-torque sensor");
            }
        }
    }
    
    // Keep collecting the position and force information for 2 sec after the hammer hits the nail
    float counter = 0.0;
    while(counter < 2){
        timeLoop = std::chrono::system_clock::now();
        counter += dt;
        // timestamp
        gettimeofday(&tv, NULL);
        curtime=tv.tv_sec;
        strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));
        dataFile << buffer << ":" << tv.tv_usec << ",";

        dataFile << Q[0] << "," << Q[1] << "," << Q[2] << "," << Q[3] << "," << Q[4] << "," << Q[5] << "," 
                << Qdot[0] << "," << Qdot[1] << "," << Qdot[2] << "," << Qdot[3] << "," << Qdot[4] << "," << Qdot[5] << "," 
                << pos[0] << "," << pos[1] << ","<< pos[2] << ","<< vel[0] << ","<< vel[1] << ","<< vel[2] << ","
                << Torque[0] << "," << Torque[1] << "," << Torque[2] << "," << Torque[3] << "," << Torque[4] << "," << Torque[5] << "," 
                << F[0] << "," << F[1] << "," << F[2] << "," << F[3] << "," << F[4] << "," << F[5] << "," 
                << FT[0] << "," << FT[1] << "," << FT[2] << "," << FT[3] << "," << FT[4] << "," << FT[5] << "," 
                << linPot[0] << "," << linPot[1] << "," <<  mag_sep[0][0] << "," << mag_sep[0][1] << "," << mag_sep[1][0] << "," << mag_sep[1][1] << "," 
                <<  vsm_force[0] << "," << vsm_force[1] << "," << dxl_pos[0] << "," << dxl_pos[1] <<"\n";

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }

    }
    // shutdown the powerball motors
    pb.update();
    
    // disable torque and disconnect dynamixel 
    // dxl_disable(dxl_comm_result, dxl_error, dxl_pos_t, portHandler, packetHandler);

    // stop threads
    stop_thread.interrupt();
    FT_thread.interrupt();
    vsm_thread.interrupt();

    // close the file 
    dataFile.close();
    // pb.shutdown_motors();   

    return 0;
}