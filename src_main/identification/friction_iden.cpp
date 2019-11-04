
#include "powerball/schunk_powerball.h"
#include <fstream>
#include <chrono>

using namespace::std;

int main(int argc, char **argv) {

    SchunkPowerball pb;
    pb.update();
    pb.set_control_mode(MODES_OF_OPERATION_VELOCITY_MODE);

    TooN::Vector<6, float> cmd = Zeros;

    // Set trajectory and measure containers
    int it = 36;
    TooN::Vector<Dynamic, float> qdot_d(it);
    TooN::Vector<Dynamic, float> q_lim(it) ;
    TooN::Vector<Dynamic,float> qdot_meas(it);
    TooN::Vector<Dynamic,float> tor_meas(it);

    qdot_d[0] = 0.5f;
    qdot_d[1] = -0.5f;
    qdot_d[2] = 1.0f;
    qdot_d[3] = -1.0f;
    qdot_d[4] = 2.0f;
    qdot_d[5] = -2.0f;
    qdot_d[6] = 3.0f;
    qdot_d[7] = -3.0f;
    qdot_d[8] = 4.0f;
    qdot_d[9] = -4.0f;
    qdot_d[10] = 5.0f;
    qdot_d[11] = -5.0f;
    qdot_d[12] = 6.0f;
    qdot_d[13] = -6.0f;
    qdot_d[14] = 7.0f;
    qdot_d[15] = -7.0f;
    qdot_d[16] = 8.0f;
    qdot_d[17] = -8.0f;
    qdot_d[18] = 9.0f;
    qdot_d[19] = -9.0f;
    qdot_d[20] = 10.0f;
    qdot_d[21] = -10.0f;
    qdot_d[22] = 12.0f;
    qdot_d[23] = -12.0f;
    qdot_d[24] = 14.0f;
    qdot_d[25] = -14.0f;
    qdot_d[26] = 16.0f;
    qdot_d[27] = -16.0f;
    qdot_d[28] = 18.0f;
    qdot_d[29] = -18.0f;
    qdot_d[30] = 20.0f;
    qdot_d[31] = -20.0f;
    qdot_d[32] = 25.0f;
    qdot_d[33] = -25.0f;
    qdot_d[34] = 32.0f;
    qdot_d[35] = -32.0f;
    qdot_d = qdot_d*M_PI/180;

    q_lim[0] = 5.0f;
    q_lim[1] = -5.1f;
    q_lim[2] = 10.0f;
    q_lim[3] = -10.2f;
    q_lim[4] = 20.0f;
    q_lim[5] = -20.5f;
    q_lim[6] = 30.0f;
    q_lim[7] = -30.8f;
    q_lim[8] = 40.0f;
    q_lim[9] = -41.0f;
    q_lim[10] = 50.0f;
    q_lim[11] = -51.5f;
    q_lim[12] = 60.0f;
    q_lim[13] = -62.0f;
    q_lim[14] = 70.0f;
    q_lim[15] = -73.0f;
    q_lim[16] = 80.0f;
    q_lim[17] = -84.0f;
    q_lim[18] = 90.0f;
    q_lim[19] = -95.0f;
    q_lim[20] = 100.0f;
    q_lim[21] = -102.0f;
    q_lim[22] = 104.0f;
    q_lim[23] = -106.0f;
    q_lim[24] = 108.0f;
    q_lim[25] = -110.0f;
    q_lim[26] = 112.0f;
    q_lim[27] = -114.0f;
    q_lim[28] = 116.0f;
    q_lim[29] = -119.0f;
    q_lim[30] = 121.0f;
    q_lim[31] = -123.0f;
    q_lim[32] = 125.0f;
    q_lim[33] = -128.0f;
    q_lim[34] = 131.0f;
    q_lim[35] = 135.0f;
    q_lim = q_lim*M_PI/180;

    /*
    qdot_d = makeVector(0.5f, -0.5f, 1.0f, -1.0f, 2.0f, -2.0f, 3.0f, -3.0f, 4.0f, -4.0f, 5.0f, -5.0f,
                        7.0f, -7.0f , 8.0f, -8.0f, 9.0f, -9.0f, 10.0f, -10.0f, 12.0f, -12.0f, 14.0f, -14.0f,
                        16.0f, -16.0f, 18.0f, -18.0f, 20.0f, -20.0f, 25.0f, -25.0f, 30.0f, -30.0f,
                        35.0f, -35.0f, 40.0f, -40.0f, 50.0f, -50.0f, 60.0f, -60.0f)*M_PI/180;

    q_lim = makeVector(5.0f, -5.1f, 10.0f, -10.2f, 20.0f, -20.5f, 30.0f, -30.8f, 40.0f, -41.0f, 50.0f, -51.5f,
                       60.0f, -62.0f, 70.0f, -73.0f, 80.0f, -84.0f, 90.0f, -95.0f, 100.0f, -105.0f, 120.0f, -125.0f,
                       130.0f, -135.0f, 140.0f, -141.0f, 142.0f, -143.0f, 144.0f, -145.5f, 147.0f, -148.5f, 150.0f,
                       -151.5f, 153.0f, -154.5f, 156.0f, -158.0f, 160.0f, -163.0f)*M_PI/180;
    */

    // Test axis
    int node_num = NODE_5;
    int motor_ind = node2motor(node_num)-1;


    // Set measure containers
    float actual_pos, actual_vel, actual_tor;


    int i;
    float Ts = 0.005f;


    for (int n=0; n<it ; n++) {
        std::cout << "index number = " << n << std::endl;
        cmd[motor_ind] = qdot_d[n];
        pb.set_vel(cmd);
        pb.update();
        std::cout << "set velocity as\n " << cmd << std::endl;
        actual_pos = pb.get_pos(node_num);
        actual_vel = pb.get_vel(node_num);
        actual_tor = pb.get_tor(node_num);

        std::cout << "actual= " << abs(actual_pos) << " lim= " << abs(q_lim[n]) << std::endl;

        i = 0;
        while( abs(actual_pos)<abs(q_lim[n]) ) {  //

            usleep(Ts*1000*1000);
            pb.update();
            actual_pos = pb.get_pos(node_num);
            actual_vel = pb.get_vel(node_num);
            actual_tor = pb.get_tor(node_num);
            i++;
            qdot_meas[n] =  ((i-1)*qdot_meas[n] + actual_vel)/i;
            tor_meas[n] =  ((i-1)*tor_meas[n] + actual_tor)/i;

        }

        cmd = makeVector(0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0);
        pb.set_vel(cmd);
        pb.update();
        std::cout << "Pos limit " << q_lim[n] << " is reached. qdot = " << qdot_meas[n]  << " and torq = " << tor_meas[n] << std::endl;
        sleep(1);

    }



    cmd = makeVector(0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0);
    pb.set_vel(cmd);
    pb.update();

    std::ofstream dataFile;
    dataFile.open("friction_n"+ std::to_string(node_num)  +"_01.txt");
    for (int k=0; k<it; k++) {
        dataFile << qdot_meas[k] << "   " << tor_meas[k] << std::endl;
    }
    dataFile.close();

    std::cout << "Exiting main..." << std::endl;
    return 0;

}

