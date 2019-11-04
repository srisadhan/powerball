
#include "powerball/schunk_powerball.h"
#include <fstream>
#include <chrono>

using namespace::std;

int main(int argc, char **argv) {

    SchunkPowerball pb;



    // Set sampling options
    int i = 0;
    float Ts = 0.005f;
    float tf = 60.0;
    int it = tf/Ts;

    // Set measure containers. It will contain time -- q -- qdot -- tor
    float time_meas[it];
    TooN::Matrix<Dynamic,6,float> q_meas(it,6);
    TooN::Matrix<Dynamic,6,float> qdot_meas(it,6);
    TooN::Matrix<Dynamic,6,float> tor_meas(it,6);

    pb.update();

    // Set trajectory
    TooN::Vector<6, float> q, q_read, q_0, qdot_read, tor_read;
    q_read = pb.get_pos();
    q_0 = q_read;
    q = q_0;
    qdot_read = tor_read;

    std::chrono::time_point<std::chrono::system_clock> timeNow, timeStart, timeLoop;
    timeLoop = timeStart = timeNow = std::chrono::system_clock::now();
    std::chrono::duration<float> elaps, elaps_loop;

    TooN::Vector<6,float> A = Zeros;
    TooN::Vector<6,float> f_1 = Zeros;
    TooN::Vector<6,float> f_2 = Zeros;
    TooN::Vector<6,float> f = Zeros;
    TooN::Vector<6,float> w = Zeros;

    A = makeVector(45.0, 60.0, 60.0, 40.0, 40.0, 40.0)*M_PI/180.0;
    f_1 = 0.01*makeVector(1.0,1.25,-1.6,3.0,2.5,2.6);
    f_2 = 7.5f*f_1;

    while(i*Ts<tf) {

        timeLoop = std::chrono::system_clock::now();

        if(i*Ts<=5.0f) {
            f = f_1 + (f_2-f_1)*sin(M_PI/10*i*Ts); // ( (f_2-f_1)/(5.0f) )*i*Ts;
        }
        else {
            f = f_2;
        }
        w = 2.0*M_PI*f;

        q[0] = q_0[0] + A[0]*(1-cos(w[0]*i*Ts));
        q[1] = q_0[1] + A[1]*sin(w[1]*i*Ts);
        q[2] = q_0[2] + A[2]*sin(w[2]*i*Ts);
        q[3] = q_0[3] + A[3]*(1-cos(w[3]*i*Ts));
        q[4] = q_0[4] + A[4]*(1-cos(w[4]*i*Ts));
        q[5] = q_0[5] - A[5]*(1-cos(w[5]*i*Ts));
        pb.set_pos(q);
        pb.update();

        q_read = pb.get_pos();
        qdot_read = pb.get_vel();
        tor_read = pb.get_tor();
        q_meas[i] = q_read;
        qdot_meas[i] = qdot_read;
        tor_meas[i] = tor_read;

        i++;
        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (Ts-elaps_loop.count()) > 0 ) {
            usleep( (Ts-elaps_loop.count())*1000*1000 );
        }

    }


    std::ofstream dataFile;
    dataFile.open("measures.txt");
    for (int k=0; k<it; k++) {
        dataFile << " " << q_meas[k] << qdot_meas[k] << tor_meas[k] << std::endl;
    }
    dataFile.close();


    sleep(1);
    std::cout << "Exiting main..." << std::endl;
    return 0;

}

