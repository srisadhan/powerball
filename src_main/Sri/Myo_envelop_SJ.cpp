#include "myolinux/myoclient.h"
#include "myolinux/serial.h"
#include "gnuplot-iostream.h"
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <TooN/TooN.h>
#include <boost/thread.hpp>
#include "matplotlibcpp.h"
#include "sigpack.h"
#include <armadillo>

using namespace myolinux;
using namespace std;
using namespace std::chrono;
using namespace TooN;
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

// Global variables
Vector<8,float> EMG = Zeros;
Vector<4,float> ORI = Zeros;
Vector<3,float> ACC = Zeros;
Vector<3,float> GYR = Zeros;

// std::vector<std::vector<float>> emgVec(1000, std::vector<float>(8));
arma::mat emgVec(10000,8,arma::fill::zeros);
int counter = 0;
int windLen = 50;

// GNUplot
Gnuplot gp;

// stop the program if enter is pressed in the command prompt
void stop(bool *stopFlag){
  char in;
  cin.get(in);
  *stopFlag = true;
}

// Myo armband initialization code
myo::Client Myo_init(myo::Client client)
{
    client.connect();// Autoconnect to the first Myo device
    if (!client.connected()) {
        cout<< red << "Unable to connect to Myo band"<<def<<endl;
    }else{
        cout<< green <<"Connection established with the Myo band"<<def<<endl;
    }
    // Vibrate
    client.vibrate(myo::Vibration::Short);
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
    return client;
}

// Starts recording data from Myo armband
void Myo_receive(myo::Client *client, std::string subject_Name, bool *stopFlag){
  // timestamp vars
  char buffer[10];
  struct timeval tv;
  time_t curtime;

  // Open recording file
  std::ofstream EMGFile, IMUFile;
  EMGFile.open("Sri/"+subject_Name+"_"+"EMG.csv");
  EMGFile << "Time,EMG1,EMG2,EMG3,EMG4,EMG5,EMG6,EMG7,EMG8"<< endl;

  IMUFile.open("Sri/"+subject_Name+"_"+"IMU.csv");
  IMUFile << "Time,ORI1,ORI2,ORI3,ORI4,ACC1,ACC2,ACC3,GYR1,GYR2,GYR3"<< endl;
  while(!*stopFlag){
      try {
            client->listen();
            gettimeofday(&tv, NULL);
            curtime=tv.tv_sec;
            strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));

            EMGFile << buffer << ":" << tv.tv_usec << ",";
            IMUFile << buffer << ":" << tv.tv_usec << ",";

            for(int i = 0; i < 8;i++){
              emgVec(counter,i)= abs(EMG[i]);
            }
            counter += 1;

            EMGFile<< EMG[0]<< "," << EMG[1]<< "," << EMG[2]<< "," << EMG[3]<< "," << EMG[4]<< "," << EMG[5]<< "," << EMG[6]<< "," << EMG[7]<<endl;
            IMUFile<< ORI[0]<< "," << ORI[1]<< "," << ORI[2]<< "," << ORI[3]<< "," << ACC[0]<< "," << ACC[1]<< "," << ACC[2]<< "," << GYR[0]<< "," << GYR[1]<< "," << GYR[2] <<endl;
      }
      catch(myo::DisconnectedException &) {
          cout << red << "MYO Disconnected!" << def <<endl;
          cout << green <<"Trying to reconnect...." << def << endl;
          myo::Client client1(Serial{"/dev/ttyACM0", 115200});
          *client = Myo_init(client1);
      }
  }
}

// GNUplot based plotting of EMG
void plot_EMG(Gnuplot *gp, std::vector<std::vector<float>> draw,bool *stopFlag)
{
  if(!*stopFlag){
    for(int i = 0; i < 8;i++){
      draw[0].push_back(i+1);
      draw[1].push_back(0);
      draw[2].push_back(0);
      draw[3].push_back(abs(EMG[i]));
    }
    *gp << "set xrange [0:9]\nset yrange [-100:100]\n";
    *gp << "plot '-' with vectors \n";
    gp->send(draw);
  }
}

// matplot lib based plotting
void matplot_EMG(std::vector<float> draw,bool *stopFlag){
  if(!*stopFlag){
    std::vector<float> x = {1,2,3,4,5,6,7,8};
    std::vector<float> y = {0,0,0,0,0,0,0,0};
    for(int i = 0; i < 8;i++){
      draw.push_back(abs(EMG[i]));
    }
    plt::quiver(x,y,y,draw);
    plt::pause(0.001);
    plt::clf();
  }
}

// plot emg envelop
void matplot_Enlp(){
  // std::vector<float> sumVec(8);
  // if(counter >= windLen){
  //   for(int i=0; i< 8;i++){
  //     for(int j=counter-windLen;j<counter+windLen; j++){
  //         sumVec[i] += emgVec(j,i);
  //     }
  //     sumVec[i] = sumVec[i]/windLen;
  //     // cout<<emgVec[i][0]<<emgVec[i][1]<<emgVec[i][2]<<emgVec[i][3]<<emgVec[i][4]<<emgVec[i][5]<<emgVec[i][6]<<emgVec[i][7]<<endl;
  //     cout<< sumVec[0]<< sumVec[1]<< sumVec[2]<< sumVec[3]<< sumVec[4]<< sumVec[5]<< sumVec[6]<< sumVec[7]<< endl;
  //   }
  // }

  // std::vector<float> sumVec(10000,-1.0);
  std::vector<float> cnt(100000,0.0);
  std::vector<float> x(2,counter);
  std::vector<float> ymin(2,0.0);         // min envelop among 8 EMG channels
  std::vector<float> ymax(2,0.0);         // max envelop among 8 EMG channels
  std::vector<float> ydiff(2,0.0);        // difference of max and min envelop
  arma::rowvec temp1(8,arma::fill::zeros);
  float temp = 0;
  int srt = 0, stp = 0;

  if(counter >= windLen){
      srt     = counter-std::floor(windLen/2);
      stp     = counter+std::floor(windLen/2);
      temp1   = (arma::sum(emgVec.rows(srt,stp))/50)/windLen;
      ymin[1] = arma::min(temp1);
      ymax[1] = arma::max(temp1);
      ydiff[1]= ymax[1] - ymin[1];
      // for(int i = counter-windLen;i<counter+windLen; i++){
      //   temp += emgVec(i,1);
      //   cnt[i]  = counter;
      // }
      // y[1] = temp/windLen;
      // y[1] = arma::max(temp1);

      plt::subplot(3,1,1);
      plt::plot(x,ymin,"r-");
      plt::xlim(0,10000);plt::ylim(0,1);
      plt::suptitle("Min of EMG channels");

      plt::subplot(3,1,2);
      plt::plot(x,ymax,"r-");
      plt::xlim(0,10000);plt::ylim(0,1);
      plt::suptitle("Max of EMG channels");

      plt::subplot(3,1,3);
      plt::plot(x,ydiff,"r-");
      plt::xlim(0,10000);plt::ylim(0,1);
      plt::suptitle("Diff of EMG envelop");

      plt::pause(0.001);
  }


  // plt::clf();
}

int main()
{
  std::vector<std::vector<float>> draw(4);
  std::vector<float> draw1;
  Gnuplot gp1("gnuplot -persist");

  // Initializing the myo connection
  myo::Client client(Serial{"/dev/ttyACM0", 115200});

  try {
    client = Myo_init(client);
  }
  catch (myo::DisconnectedException &){
    cout << "An exception occurred." << endl;
  }

  std::string subname;
  std::cout<< "Please enter subject name:"<<std::endl;
  std::cin>>subname;
  std::cin.get();

  cout<< "Get ready to perform isometric contraction of muscles"<<endl;

  bool stopFlag = false;
  boost::thread myoThread(Myo_receive, &client, subname, &stopFlag);
  boost::thread stopThread(stop, &stopFlag);

  // usleep(1*1000*1000);
  // int  calib_cnt = 0;
  // arma::mat calEMG(1000,8,arma::fill::zeros);
  // while (calib_cnt<1000){
  //     calEMG.row(calib_cnt) = emgVec.row(counter-1);
  //     calib_cnt += 1;
  // }
  // arma::rowvec maxEMG(8,arma::fill::zeros);
  // maxEMG = max(calEMG,0);
  // maxEMG.replace(0,1);
  // cout << maxEMG<<endl;

  while(!stopFlag){
    // plot_EMG(&gp, draw, &stopFlag); // plot with gnuplot
    // matplot_EMG(draw1,&stopFlag);      // plot with matplotlibcpp
    matplot_Enlp();
  }

  myoThread.interrupt();
  stopThread.interrupt();
}
