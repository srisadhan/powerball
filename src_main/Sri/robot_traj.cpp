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

// schunk powerball headers
#include "powerball/schunk_powerball.h"
#include "vrep/v_repClass.h"
#include "powerball/schunk_kinematics.h"
#include "utils/utils.h"

// Toon headers
#include <TooN/TooN.h>
#include <TooN/LU.h>
#include <TooN/SVD.h>

// myo band headers
#include "myolinux/myoclient.h"
#include "myolinux/serial.h"

// dynamixel headers
#include <utils.h>
#include <USB2Dynamixel.h>
#include <commonOptions.h>

// phidget headers
#include <phidget21.h>

// plotting and linear algebra libraries
#include "matplotlibcpp.h"
#include "sigpack.h"
#include <armadillo>


using namespace::std;
using namespace::TooN;
using boost::asio::ip::tcp;
namespace
{
commonOptions::Option<std::vector<std::string>> cnfDevice("usb2dyn.device", {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"}, "device for usb2BaccaratDealer");
commonOptions::Option<std::string> cfgConfigFile("file", "motorConfig.json", "file to work on");

commonOptions::Option<int> cfgSetupMotorID("id", -1, "setup motor to 1000000 baud and id (parameter)");
commonOptions::Switch swtScanAll("scanAll", "scan all motors");

commonOptions::Switch swtHelp("help", "show help", []() {
    commonOptions::print();
    exit(EXIT_SUCCESS);
});
}


Color::Modifier red(Color::FG_RED);
Color::Modifier green(Color::FG_GREEN);
Color::Modifier def(Color::FG_DEFAULT);

// global vars
Vector<6,float> FT;
const double resolution = 300*M_PI*25.46e-3/180/1023;
const double magWidth = 0.022; // width of the magnet holder
const double forceThres = 1.5; // force threshold on the object
const double dt = 0.005; // sampling time
double K_gain = 200;
Vector<6,float> joints_vrep = Zeros;
bool stopFlag = false;
int num_H;


void stop(bool* flag){
    char in;
    cin.get(in);
    *flag = true;
}

// Vrep function
int vrep_draw(bool* flag){
    // connect to vrep
    int res = -1;
    if (! *flag){
      V_rep vrep;
      res = vrep.connect();
      if (res==-1)
      {
          cout << "V-REP Connection Error!" << endl;
          return 0;
      }

      while(!*flag)
      {
          vrep.setq(joints_vrep);
          usleep(40*1000);
      }
    }
}

void load_matrix(std::istream* is,
                 std::vector< std::vector<double> >* matrix,
                 const std::string& delim = " \t")
{

    string      line;
    string      strnum;

    // clear first
    matrix->clear();

    // parse line by line
    while (getline(*is, line))
    {
        matrix->push_back(vector<double>());

        for (string::const_iterator i = line.begin(); i != line.end(); ++ i)
        {
            // If i is not a delim, then append it to strnum
            if (delim.find(*i) == string::npos)
            {
                strnum += *i;
                if (i + 1 != line.end()) // If it's the last char, do not continue
                    continue;
            }

            // if strnum is still empty, it means the previous char is also a
            // delim (several delims appear together). Ignore this char.
            if (strnum.empty())
                continue;

            // If we reach here, we got a number. Convert it to double.
            double       number;

            istringstream(strnum) >> number;
            matrix->back().push_back(number);

            strnum.clear();
        }
    }
}

Matrix<Dynamic,6,double> traj_interp(Vector<6,float> Q, Vector<6,float> Qdot, Vector<6,float> Qref,float dt, float T)
{
  Kin kin;
  std::vector<std::vector<double>>interpolated_data;
  num_H = kin.HerInter(Q,Qdot,Qref,dt,T,&interpolated_data);

  int col,row = 0;
  Matrix<Dynamic,6,double> reach_point(num_H, 6);
  for (std::vector< std::vector<double> >::const_iterator it = interpolated_data.begin(); it != interpolated_data.end(); ++ it)
  {
     col = 0;
     for (std::vector<double>::const_iterator itit = it->begin(); itit != it->end(); ++ itit)
     {
         reach_point(row,col) = *itit;
         col++;
     }
     row++;
  }
  return reach_point;
}


static uint16_t readPosition(dynamixel::motorID motor, USB2Dynamixel &usb2Dynamixel)
{
    std::mutex mutex;
    uint16_t position;
    usb2Dynamixel.read(motor, dynamixel::Register::PRESENT_POSITION, 2, 0.01 * seconds,
                       [&](dynamixel::motorID, bool success, uint8_t, const uint8_t* receiveBuffer, uint8_t)
    {
        if (success)
        {
            position = *((uint16_t*)receiveBuffer);
        }
    }, &mutex);
    mutex.lock();
    return position;
}

static uint16_t readTorqueLimit(dynamixel::motorID motor, USB2Dynamixel &usb2Dynamixel)
{
    std::mutex mutex;
    uint16_t limit;
    usb2Dynamixel.read(motor, dynamixel::Register::TORQUE_LIMIT, 2, 0.01 * seconds,
                       [&](dynamixel::motorID, bool success, uint8_t, const uint8_t* receiveBuffer, uint8_t)
    {
        if (success)
        {
            limit = *((uint16_t*)receiveBuffer);
        }
    }, &mutex);
    mutex.lock();
    return limit;
}

static void setPosition(dynamixel::motorID motor, uint16_t targetPos, USB2Dynamixel &usb2Dynamixel) {
    uint8_t targetPosLow = (targetPos >> 0) & 0xff;
    uint8_t targetPosHigh = (targetPos >> 8) & 0xff;
    usb2Dynamixel.write(motor, dynamixel::Register::GOAL_POSITION, {targetPosLow, targetPosHigh});
}


static void setMaxTorque(dynamixel::motorID motor, uint16_t torqueLimit, USB2Dynamixel &usb2Dynamixel) {
    uint8_t torqueLimitLow = (torqueLimit >> 0) & 0xff;
    uint8_t torqueLimitHigh = (torqueLimit >> 8) & 0xff;
    usb2Dynamixel.write(motor, dynamixel::Register::TORQUE_LIMIT, {torqueLimitLow, torqueLimitHigh});
}

double linPot[2];

int CCONV SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value)
{
    double potOff0 = 0.004; // 0.002 - offset in the potentiometer reading
    double potOff1 = 0.006; // potentiometer is placed at on offset on both the sides

    if(Index == 0){
      linPot[Index] = (60-Value*60/1023)/1e3 + potOff0;
    }else if (Index == 1){
      linPot[Index] = (Value*60/1023)/1e3 + potOff1;
    }
    // cout << linPot << endl;
    return 0;
}

int CCONV ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
    printf("Error handled. %d - %s", ErrorCode, unknown);
    return 0;
}

// Magnet force model
double magForce(double s){
  double a = 4.8e-6;
  double b = 5.3e-5;
  double c = 2.3e-7;
  return a/(pow(s,3)+b*s+c);
}

// resultant force on the object
double resForce(USB2Dynamixel &usb2Dynamixel, double finger1, double finger2){

  double dyn1pos = 0.1425 - (1023 - readPosition(1,usb2Dynamixel))*resolution;
  return magForce(dyn1pos/2 - (finger1+ magWidth/2)) - magForce(dyn1pos/2 - (finger2+ magWidth/2)) ;
}

// Calculating dynamixel position asjustment for the desired magnet position using control algorithm
double interpMag(USB2Dynamixel &usb2Dynamixel, double pos, int initDyn){
  int Dyn = round((readPosition(1,usb2Dynamixel)*resolution + pos)/resolution);
  if (Dyn > 1023){
    Dyn = 1023;
  }
  else if (Dyn < initDyn){
    Dyn = initDyn;
  }
  return Dyn;
}

// Magnet Position control based on the force threshold on the object
void magnetControl (USB2Dynamixel &usb2Dynamixel, double finger1, double finger2, int initDyn, bool* stopFlag){
  if(!*stopFlag){
    double objForce = abs(resForce(usb2Dynamixel,finger1,finger2));
    if(abs(objForce - forceThres) > 0.5){
      double x = K_gain *(objForce-forceThres)*pow(dt,2);
      // cout << "Set this position to the dynamixel: " << interpMag(usb2Dynamixel, x, initDyn) <<endl;
      setPosition(1, interpMag(usb2Dynamixel, x, initDyn), usb2Dynamixel);
    }
  }else{
    cout << "Could not control the magnet position, exiting the control loop"<<endl;
  }
}



int main (int argc, char** argv){

  bool vrepFlag = false; // connect to vrep
  bool stiffVar = false; // flag for variable stiffness
  int dyn1pos;
  string filename;
  // arg to pass to the code is in the format "sudo ./admittance_3d_velMode vrep arg2 arg3 ..."
  if (argc>1){
    if (strcmp(argv[1],"orange") == 0){
      dyn1pos = 690;
    }else if (strcmp(argv[1],"spaghetti") == 0){
      dyn1pos   = 880;
    }
    if (argc>2){
      if (strcmp(argv[2],"var") == 0){
        stiffVar = true;
        cout << green << "In variable stiffness mode"<< def <<endl;
      }
    }
    filename = argv[1];
  }

  // Initializing dynamixel
  USB2Dynamixel usb2dyn(*cnfDevice, 50);
  int id1 = 1, id2 = 2;
  setMaxTorque(id1, 1023, usb2dyn); // outer magnets max 1023
  setMaxTorque(id2, 1023, usb2dyn); // inner magnets

  // int dyn1pos = 880; // spaghetti Length
  // int dyn1pos = 650;

  setPosition(id1, dyn1pos, usb2dyn);
  setPosition(id2, 600, usb2dyn);
  usleep( 1*1000*1000 );

  // return 0;
  //Initializing phidget
  int result, numSensors;
  const char *err;
  CPhidgetInterfaceKitHandle ifKit = 0;
  CPhidgetInterfaceKit_create(&ifKit);
  CPhidget_set_OnError_Handler((CPhidgetHandle)ifKit, ErrorHandler, NULL);
  CPhidgetInterfaceKit_set_OnSensorChange_Handler (ifKit, SensorChangeHandler, NULL);
  CPhidget_open((CPhidgetHandle)ifKit, -1);
  if((result = CPhidget_waitForAttachment((CPhidgetHandle)ifKit, 10000)))
  {
      CPhidget_getErrorDescription(result, &err);
      printf("Problem connecting to phidget: %s\n", err);
      return 0;
  }
  CPhidgetInterfaceKit_setRatiometric(ifKit, 0);
  CPhidgetInterfaceKit_getSensorCount(ifKit, &numSensors);
  for(int i = 0; i < numSensors; i++)
  {
      CPhidgetInterfaceKit_setSensorChangeTrigger(ifKit, i, 0);  //we'll just use 10 for fun
  }

  cout << "dynamixel and phidget are joined!" << endl;

  // timestamp vars
  char buffer[10];
  struct timeval tv;
  time_t curtime;
  std::chrono::time_point<std::chrono::system_clock> timeLoop;
  std::chrono::duration<float> elaps_loop;



  // insert file
  // std::ifstream is("spaghetti/1.txt");
  std::ifstream is("Collision_var/line_"+filename+".txt");
  if (!is)
  {
      cout<< "error reading file!" << endl;
      return 0;
  }
  cout << green << "Collision_var/line_"+filename+".txt" << def << endl;
  // load the matrix
  std::vector< std::vector<double> > matrix;
  load_matrix(&is, &matrix);

  // Check the input matrix size and set TooN matrix size
  int  ncols, nrows=0;
  for (std::vector< std::vector<double> >::const_iterator it = matrix.begin(); it != matrix.end(); ++ it)
  {
      nrows++;
      ncols = 0;
      for (std::vector<double>::const_iterator itit = it->begin(); itit != it->end(); ++ itit)
      {
          ncols++;
      }
  }
  TooN::Matrix<Dynamic,Dynamic,double> traj(nrows, ncols);
  cout << "size of matrix = " << nrows << "*" << ncols << endl;
  if (ncols!=6)
  {
      cout << "inconsistent input matrix size" << endl;
      return 0;
  }

  // Put the matrix into TooN matrix
  nrows = 0;  ncols = 0;
  for (std::vector< std::vector<double> >::const_iterator it = matrix.begin(); it != matrix.end(); ++ it)
  {
      for (std::vector<double>::const_iterator itit = it->begin(); itit != it->end(); ++ itit)
      {
          traj(nrows,ncols) = *itit;
          ncols++;
      }
      nrows++;
      ncols = 0;
  }


  boost::thread stop_thread(stop, &stopFlag);
  boost::thread vrep_thread(vrep_draw,&vrepFlag);

  SchunkPowerball pb;
  pb.update();

  Vector<6,float> Q = Zeros, Qdot = Zeros;
  Q = pb.get_pos();
  cout << "Current pos of the robot: " << Q <<", Desired start position: "<<traj[0]<< endl;
  Matrix<Dynamic,6,float>  reach_point1 = traj_interp(Q, Qdot, traj[0],dt,5);
  // Q = track_points(pb, reach_point1, dt);
  int i = 0;
  while(i<num_H)
  {
      timeLoop = std::chrono::system_clock::now();
      pb.set_pos(reach_point1[i]);
      pb.update();

      i++;

      elaps_loop = std::chrono::system_clock::now() - timeLoop;
      if ( (dt-elaps_loop.count()) > 0 ) {
          usleep( (dt-elaps_loop.count())*1000*1000 );
      }
  }
  Q = pb.get_pos();
  cout << green << "Robot successfully sent to the start position" << def << endl;

  i = 0;
  while((i<nrows) && (!stopFlag))
  {
    timeLoop = std::chrono::system_clock::now();

    for(int j = 0; j < 6; j++){
        joints_vrep[j] = traj(i,j);
    }

    pb.set_pos(traj[i]);
    pb.update();

    if (stiffVar){
      if (strcmp(argv[1],"orange") == 0){
        magnetControl(usb2dyn,linPot[0],linPot[1],dyn1pos-20, &stopFlag);
      }else{
        magnetControl(usb2dyn,linPot[0],linPot[1],dyn1pos, &stopFlag);
      }

    }

    i++;
    elaps_loop = std::chrono::system_clock::now() - timeLoop;
    if ( (dt-elaps_loop.count()) > 0 ) {
        usleep( (dt-elaps_loop.count())*1000*1000 );
    } else
    {
        cout << red << "communication time out!" << def << endl;
    }
  }

  stop_thread.interrupt();
  vrep_thread.interrupt();
  usleep(0.2*1000*1000);

  return 0;
}
