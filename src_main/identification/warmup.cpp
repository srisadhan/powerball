
#include "powerball/schunk_powerball.h"
#include <fstream>
#include <chrono>

using namespace::std;

int main(int argc, char **argv) {

    SchunkPowerball pb;

    // Set sampling options
    int i = 0;
    float Ts = 0.007f;
    float tf = 96.0f*2;
    float time = 0.0f;
    int osc = 24;
    int it = 2*osc;

    int node_num = NODE_5;

    TooN::Vector<Dynamic,float> qdot_meas(it);
    TooN::Vector<Dynamic,float> tor_meas(it);

    pb.update();
    pb.goHome();
    pb.update();
    
    // Set trajectory
    TooN::Vector<6, float> q, q_0;
    q_0 = pb.get_pos();
    q = q_0;

    std::chrono::time_point<std::chrono::system_clock> timeNow, timeStart;
    timeStart = timeNow = std::chrono::system_clock::now();
    std::chrono::duration<float> elaps;

    TooN::Vector<6,float> A = Zeros;
    TooN::Vector<6,float> f = Zeros;
    TooN::Vector<6,float> w = Zeros;
     float actual_pos, actual_vel, actual_tor;

    A = makeVector(20.0, 20.0, 0.0, 0.0, 45.0, 0.0)*M_PI/180.0; // makeVector(0.0, 0.0, 0.0, 0.0, 70.0, 0.0)*M_PI/180.0;
    f = makeVector(10.0/tf,10/tf,-1.5/tf,osc/tf,osc/tf,2.6/tf); // makeVector(1.0/tf,1.25/tf,-1.5/tf,osc/tf,osc/tf,2.6/tf);
    w = 2.0*M_PI*f;

    int n = 0;
    int j = 0;

    while(i*Ts<tf) {


        // q[4] = q_0[4] + A[4]*(1-cos(w[4]*i*Ts)); // movement of joint 4
        // q[1] = q_0[1] + A[1]*(1-cos(w[1]*i*Ts)); // movement of joint 2
        q[0] = q_0[0] + A[0]*(1-cos(w[0]*i*Ts)); // movement of joint 1

        pb.set_pos(q);
        pb.update();
        i++;

        actual_pos = pb.get_pos(node_num);
        actual_vel = pb.get_vel(node_num);
        actual_tor = pb.get_tor(node_num);

        j++;
        qdot_meas[n] =  ((j-1)*qdot_meas[n] + actual_vel)/j;
        tor_meas[n] =  ((j-1)*tor_meas[n] + actual_tor)/j;

        if (w[3]*j*Ts >= M_PI) {
            n++;
            j = 0;
            std::cout << "n is " << n << std::endl;
        }


        usleep( Ts*1000*1000);

    }

    std::ofstream dataFile;
    dataFile.open("jan07_5.txt");
    for (int k=0; k<it; k++) {
        dataFile << qdot_meas[k] << "  " << tor_meas[k] << std::endl;
    }
    dataFile.close();

    std::cout << "Exiting main..." << std::endl;
    return 0;

}

