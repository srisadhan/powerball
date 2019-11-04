#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <ostream> // included for color output to the terminal

// boost headers
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>

// myo band headers
#include "myolinux/myoclient.h"
#include "myolinux/serial.h"

// plotting and linear algebra libraries
#include "matplotlibcpp.h"
#include "sigpack.h"
#include <armadillo>

// Toon headers
// #include <TooN/LU.h>
#include <TooN/SVD.h>


using namespace::std;
using boost::asio::ip::tcp;
using namespace::TooN;
using namespace myolinux;
namespace plt = matplotlibcpp;

// code for color display in the terminal
namespace Color {
    enum Code {
        FG_RED      = 31,
        FG_GREEN    = 32,
        FG_BLUE     = 34,
        FG_DEFAULT  = 39,
        BG_RED      = 41,
        BG_GREEN    = 42,
        BG_BLUE     = 44,
        BG_DEFAULT  = 49
    };
    class Modifier {
        Code code;
    public:
        Modifier(Code pCode) : code(pCode) {}
        friend std::ostream&
        operator<<(std::ostream& os, const Modifier& mod) {
            return os << "\033[" << mod.code << "m";
        }
    };
}

Color::Modifier red(Color::FG_RED);
Color::Modifier green(Color::FG_GREEN);
Color::Modifier def(Color::FG_DEFAULT);

// Initializing the myo connection
myo::Client client(Serial{"/dev/ttyACM0", 115200});


// Initializing the variables
Vector<8,float> EMG = Zeros;
Vector<4,float> ORI = Zeros;
Vector<3,float> ACC = Zeros;
Vector<3,float> GYR = Zeros;
Vector<6,float> FT  = Zeros;
bool Start_record   = false;
arma::mat emgVec(10000,8,arma::fill::zeros);
int counter         = 0;
float dt            = 0.005;

//---------------------- Functions

// stops the program
void stop(bool* flag){
    char in;
    cin.get(in);
    *flag = true;
    Start_record = false;
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
        cout<< red << "Unable to connect to Myo band"<<def<<endl;
    }else{
        cout<< green <<"Connection established with the Myo band .........."<<def<<endl;
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
    EMGFile.open("Team6/EMG.csv");
    EMGFile << "Time,EMG1,EMG2,EMG3,EMG4,EMG5,EMG6,EMG7,EMG8"<< endl;

    IMUFile.open("Team6/IMU.csv");
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

              for(int i = 0; i < 8;i++){
                emgVec(counter,i)= abs(EMG[i]);
              }
              counter += 1;

              EMGFile<< EMG[0]<< "," << EMG[1]<< "," << EMG[2]<< "," << EMG[3]<< "," << EMG[4]<< "," << EMG[5]<< "," << EMG[6]<< "," << EMG[7]<< endl;
              IMUFile<< ORI[0]<< "," << ORI[1]<< "," << ORI[2]<< "," << ORI[3]<< "," << ACC[0]<< "," << ACC[1]<< "," << ACC[2]<< "," << GYR[0]<< "," << GYR[1]<< "," << GYR[2] <<endl;
            }
        }
        catch(myo::DisconnectedException &) {
            cout << "MYO Disconnected" << endl;
        }
    }
}

void EMG_calib(bool *errFlag)
{
    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;

    // Open recording file
    std::ofstream EMGFile, IMUFile;
    EMGFile.open("Team6/Calib.csv");
    EMGFile << "Time,EMG1,EMG2,EMG3,EMG4,EMG5,EMG6,EMG7,EMG8"<< endl;

    while(!*errFlag){
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

//------------------Main function ------------------------------//
int main(int argc, char** argv){
  // timestamp vars
  char buffer[10];
  struct timeval tv;
  time_t curtime;
  std::chrono::time_point<std::chrono::system_clock> timeLoop;
  std::chrono::duration<float> elaps_loop;

  // Initialize Myo band
  myo::Client client = Myo_init(); // initializing the myo band here works (sometimes it works here and sometimes after initializing the admittance control thread).

  #if 0
  // Muscle activity calibration using the MYO armband
  bool Calib_errFlag = false;
  boost::thread Calib_thread(EMG_calib,&Calib_errFlag);
  cout<< red <<"Relax your muscles while maintaining the hand configuration.."<<def<<endl;
  usleep(5*1000*1000);
  cout<< green <<"Get ready to hold the handle firmly"<<def<<endl;
  usleep(1*1000*1000);
  cout<< red <<"Now practice high grasp------------"<< def<< endl;
  usleep(5*1000*1000);
  cout<<"Calibration completed, you can relax now"<<endl;
  Calib_errFlag = true;
  Calib_thread.interrupt();
  #endif

  // Stop thread
  bool stopFlag = false;
  boost::thread stop_thread(stop,&stopFlag);

  // Myo thread for the actual experiment
  bool Myo_errFlag = false;
  boost::thread Myo_thread(Myo_receive,&Myo_errFlag);

  // Open recording file
  std::ofstream dataFile;
  dataFile.open("Team6/Force.csv");
  dataFile << "Time,FT1,FT2,FT3,FT4,FT5,FT6" << endl;

  // Initializing FT sensor
  bool errFlag=false;
  boost::thread FT_thread(TCP_receive,&errFlag);

  while(!stopFlag)
  {
    timeLoop = std::chrono::system_clock::now();

    Start_record = true;
    // timestamp
    gettimeofday(&tv, NULL);
    curtime=tv.tv_sec;
    strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));
    dataFile << buffer << ":" << tv.tv_usec << ",";

    dataFile<<FT[0]<<","<<FT[1]<<","<<FT[2]<<","<<FT[3]<<","<<FT[4]<<","<<FT[5]<< endl;

    elaps_loop = std::chrono::system_clock::now() - timeLoop;
    if ( (dt-elaps_loop.count()) > 0 ) {
        usleep( (dt-elaps_loop.count())*1000*1000 );
    }
    else {cout << "Communication Time Out!" << endl;}
  }

  Myo_thread.interrupt();
  FT_thread.interrupt();
  stop_thread.interrupt();

  cout << "End of the loop"<< endl;

}
