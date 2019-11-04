// Position dependancy of friction


#include "powerball/schunk_powerball.h"
#include <fstream>
#include <chrono>

using namespace::std;

int main(int argc, char **argv) {

    SchunkPowerball pb;
    pb.update();
    pb.set_control_mode(MODES_OF_OPERATION_VELOCITY_MODE);

    TooN::Vector<6, float> cmd = Zeros;

    float Ts = 0.005f;
    float qdot_d = 5.0f*M_PI/180;
    float q_lim = 80.0f*M_PI/180;
    float tf = 2*q_lim/qdot_d;
    int it = 2*tf/Ts;

    TooN::Vector<Dynamic,float> q_meas(it);
    TooN::Vector<Dynamic,float> qdot_meas(it);
    TooN::Vector<Dynamic,float> tor_meas(it);

    // Test axis
    int node_num = NODE_4;
    int motor_ind = node2motor(node_num)-1;


    // Set measure containers
    float actual_pos, actual_vel, actual_tor;

    cmd[motor_ind] = 20.0f*M_PI/180;
    pb.set_vel(cmd);
    pb.update();
    std::cout << "set velocity as\n " << cmd << std::endl;
    actual_pos = pb.get_pos(node_num);
    while( actual_pos < q_lim+50.0f*M_PI/180.0 ) {
        usleep(Ts*1000*1000);
        pb.update();
        actual_pos = pb.get_pos(node_num);
    }
    cmd = makeVector(0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0);
    pb.set_vel(cmd);
    pb.update();
    sleep(1);

    cmd[motor_ind] = -qdot_d;
    pb.set_vel(cmd);
    pb.update();
    std::cout << "set velocity as\n " << cmd << std::endl;
    int i = 0;
    while( i< (it/2)  ) {
        usleep(Ts*1000*1000);
        pb.update();
        actual_pos = pb.get_pos(node_num);
        actual_vel = pb.get_vel(node_num);
        actual_tor = pb.get_tor(node_num);

        q_meas[i] = actual_pos;
        qdot_meas[i] = actual_vel;
        tor_meas[i] = actual_tor;
        i++;
    }
    cmd = makeVector(0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0);
    pb.set_vel(cmd);
    pb.update();
    sleep(1);

    cmd[motor_ind] = qdot_d;
    pb.set_vel(cmd);
    pb.update();
    std::cout << "set velocity as\n " << cmd << std::endl;
    i = i-1;
    while( i<it ) {
        usleep(Ts*1000*1000);
        pb.update();
        actual_pos = pb.get_pos(node_num);
        actual_vel = pb.get_vel(node_num);
        actual_tor = pb.get_tor(node_num);

        q_meas[i] = actual_pos;
        qdot_meas[i] = actual_vel;
        tor_meas[i] = actual_tor;
        i++;
    }
    cmd = makeVector(0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0);
    pb.set_vel(cmd);
    pb.update();
    sleep(1);


    std::ofstream dataFile;
    dataFile.open("friction_n"+ std::to_string(node_num)  + "_vel5.txt");
    for (int k=0; k<2*it; k++) {
        dataFile << q_meas[k] << "   " << qdot_meas[k] << "   " << tor_meas[k] << std::endl;
    }
    dataFile.close();

    std::cout << "Exiting main..." << std::endl;
    return 0;

}

