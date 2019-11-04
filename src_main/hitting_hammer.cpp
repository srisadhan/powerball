
#include "powerball/schunk_powerball.h"
#include <fstream>
#include <chrono>
#include <istream>
#include <string>
#include <sstream>
#include <vector>
#include <iostream>
#include "vrep/v_repClass.h"
#include <boost/array.hpp>
#include "TooN/TooN.h"
#include <boost/thread.hpp>
#include <utils.h>
#include <USB2Dynamixel.h>
#include <commonOptions.h>
#include <phidget21.h>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <chrono>


using namespace::std;
using namespace::TooN;
using boost::asio::ip::tcp;

void stop(bool* flag){
    char in;
    cin.get(in);
    *flag = true;
}


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

//the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET   "\033[0m"
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */

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

int x[8];

int CCONV SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value)
{
    x[Index] = Value;
    return 0;
}

int CCONV ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
    printf("Error handled. %d - %s", ErrorCode, unknown);
    return 0;
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


int main(int argc, char **argv) {
    int vrep_bool = 3;
    // Main function inputs for vrep options =>  -v for vrep and robot and -vo for only vrep

    // input arguments
    if (argc==1)
    {
        vrep_bool = 0;
        cout << "Applying trajectory on PowerBall only" << endl;
    } else
    {
        if (strcmp(argv[1],"-v")==0)
        {
            vrep_bool = 1;
            cout << "Applying trajectory on PowerBall and Vrep" << endl;
        } else if (strcmp(argv[1],"-vo")==0)
        {
            vrep_bool = 2;
            cout << "Applying trajectory on Vrep only" << endl;
        }
    }


    int result, numSensors;
    const char *err;
    USB2Dynamixel usb2dyn(*cnfDevice, 50);
    int id1 = 1, id2 = 2;


    // homing for the gripper
    setMaxTorque(id1, 1023, usb2dyn); // outer magnets
    setMaxTorque(id2, 1023, usb2dyn); // inner magnets
    setPosition(id1, 600, usb2dyn);
    setPosition(id2, 0, usb2dyn);

    // initializing TCP commanding
    bool video_rec=true;


    std::string tcp_msg_start="s";
    std::string tcp_msg_stop="e";
    boost::system::error_code ignored_error;
    boost::asio::io_service io_service;
    tcp::endpoint sender_endpoint = boost::asio::ip::tcp::endpoint(
                boost::asio::ip::address::from_string("127.0.0.1"),  boost::lexical_cast<int>("40000"));
    tcp::socket socket(io_service);
    if (video_rec)
    {
        socket.connect(sender_endpoint);
        socket.write_some(boost::asio::buffer(tcp_msg_start, tcp_msg_start.size()), ignored_error);
    }

    usleep( 1*1000*1000 );

    /*// use for adjusting servos
    while(1){
        int num1,num2;
        cout << "ID1? ";
        cin >> num1;
        cout << "ID2? ";
        cin >> num2;
        setPosition(id1, num1, usb2dyn);
        setPosition(id2, num2, usb2dyn);
    }
    return 0;
    */

    //Declare an InterfaceKit handle
    CPhidgetInterfaceKitHandle ifKit = 0;
    //create the InterfaceKit object
    CPhidgetInterfaceKit_create(&ifKit);
    CPhidget_set_OnError_Handler((CPhidgetHandle)ifKit, ErrorHandler, NULL);
    //Registers a callback that will run if the sensor value changes by more than the OnSensorChange trig-ger.
    //Requires the handle for the IntefaceKit, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
    CPhidgetInterfaceKit_set_OnSensorChange_Handler (ifKit, SensorChangeHandler, NULL);
    //open the interfacekit for device connections
    CPhidget_open((CPhidgetHandle)ifKit, -1);
    if((result = CPhidget_waitForAttachment((CPhidgetHandle)ifKit, 10000)))
    {
        CPhidget_getErrorDescription(result, &err);
        printf("Problem waiting for attachment: %s\n", err);
        return 0;
    }
    CPhidgetInterfaceKit_setRatiometric(ifKit, 0);
    //get the number of sensors available
    CPhidgetInterfaceKit_getSensorCount(ifKit, &numSensors);
    //Change the sensitivity trigger of the sensors
    for(int i = 0; i < numSensors; i++)
    {
        CPhidgetInterfaceKit_setSensorChangeTrigger(ifKit, i, 0);  //we'll just use 10 for fun
    }


    // insert file
    std::ifstream is("skew3ms_new.txt");
    if (!is)
    {
        cout<< "error reading file!" << endl;
        return 0;
    }

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
            if (ncols==5) // for the new installation of the gripper and PI offset in joint 6
            {
                traj(nrows,ncols) = traj(nrows,ncols) - M_PI;
            }
            ncols++;
        }
        nrows++;
        ncols = 0;
    }

    // Set sampling and timing options
    float dt = 0.0030f;
    std::chrono::time_point<std::chrono::system_clock> timeLoop;
    std::chrono::duration<float> elaps_loop;

    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;

    // Open recording file
    std::ofstream dataFile;
    dataFile.open("RSS.csv");
    dataFile << "This is file header" << endl;

    /* ///////////////////////////////// Powerball class */
    // stop by Enter key thread
    bool stop_flag = false;
    boost::thread stop_thread(stop,&stop_flag);

    SchunkPowerball pb;
    pb.update();

    Vector<6,float> Q = Zeros, Qd = Zeros, Qref = Zeros;
    Q = pb.get_pos();
    Qd = pb.get_vel();


    // initial positioning
    Qref = traj[4000];
    Vector<6,float> dQ = Qref-Q;
    float maxq=0;
    for (int n=0;n<6;n++)
    {
        if (abs(dQ[n])>maxq) {maxq=abs(dQ[n]);}
    }
    float Ttravel = maxq*3;
    if (Ttravel<1.0){Ttravel=1.0;}
    int itNum = Ttravel/dt;
    Vector<6,float> Qhold = Q;
    int n = 1;
    while((n<=itNum) && (!stop_flag))
    {
        Vector<6,float> Qt = (1-cos(float(n)/itNum*M_PI))/2 * dQ + Qhold;
        pb.set_pos(Qt);
        pb.update();
        Q = pb.get_pos();
        usleep( dt*1000*1000 );
        n++;
    }

    // stiffness changing timing
    // motor 1:
    int t1_1=0.1/.003+4000;
    int t2_1=0.2/.003+4000;
    int t3_1=0.34/.003+4000; //36 (34)
    int t4_1=0.40/.003+4000; //44 (40)


    int i = 4000; // int i = 4000; for starting from close to ground
    while((i<nrows) && (!stop_flag))
    {
        timeLoop = std::chrono::system_clock::now();

        // timestamp
        gettimeofday(&tv, NULL);
        curtime=tv.tv_sec;
        strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));
        dataFile << buffer << ":" << tv.tv_usec << ",";

        // get joint positions & velocity
        Q = pb.get_pos();
        Qd = pb.get_vel();

        dataFile<<Q[0]<<","<<Q[1]<<","<<Q[2]<<","<<Q[3]<<","<<Q[4]<<","<<Q[5]<<","
                     <<Qd[0]<<","<<Qd[1]<<","<<Qd[2]<<","<<Qd[3]<<","<<Qd[4]<<","<<Qd[5]<<","
                    <<x[0] <<","<< x[1] <<endl;
        //","<< dyn1 <<","<< dyn2 <<endl;


        pb.set_pos(traj[i]);
        pb.update();


        if ((i==t1_1) || (i==t3_1)) // go to high stiffness
        {
           setPosition(id1, 500, usb2dyn);
           setPosition(id2, 200, usb2dyn);
        }
        if ((i==t2_1) || (i==t4_1)) // go to low stiffness
        {
            setPosition(id1, 700, usb2dyn);
            setPosition(id2, 0, usb2dyn);
        }

        i++;
        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        } else
        {
            cout << BOLDRED << "communication time out!" << RESET << endl;
        }

    }


    // additional recordings
    for (int k=i+1;k<i+201;k++ ){
        timeLoop = std::chrono::system_clock::now();
        // timestamp
        gettimeofday(&tv, NULL);
        curtime=tv.tv_sec;
        strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));
        dataFile << buffer << ":" << tv.tv_usec << ",";
        // get joint positions & velocity
        if (k>4167+30){
            pb.set_pos(traj[i]);
        }
        pb.update();
        Q = pb.get_pos();
        Qd = pb.get_vel();

        if ((k==t1_1) || (k==t3_1)) // go to high stiffness
        {
            setPosition(id1, 500, usb2dyn);
            setPosition(id2, 200, usb2dyn);
        }
        if ((k==t2_1) || (k==t4_1)) // go to low stiffness
        {
            setPosition(id1, 700, usb2dyn);
            setPosition(id2, 0, usb2dyn);
        }

        dataFile<<Q[0]<<","<<Q[1]<<","<<Q[2]<<","<<Q[3]<<","<<Q[4]<<","<<Q[5]<<","
                     <<Qd[0]<<","<<Qd[1]<<","<<Qd[2]<<","<<Qd[3]<<","<<Qd[4]<<","<<Qd[5]<<","
                    <<x[0] <<","<< x[1] << endl;
        //","<< dyn1 <<","<< dyn2 <<endl;

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        } else
        {
            cout << BOLDRED << "communication time out!" << RESET << endl;
        }
    }


    // initial positioning
    dQ = Qref-Q;
    maxq=0;
    for (int n=0;n<6;n++)
    {
        if (abs(dQ[n])>maxq) {maxq=abs(dQ[n]);}
    }
    Ttravel = maxq*3;
    if (Ttravel<1.0){Ttravel=1.0;}
    itNum = Ttravel/dt;
    Qhold = Q;
    n = 1;
    while((n<=itNum) && (!stop_flag))
    {
        Vector<6,float> Qt = (1-cos(float(n)/itNum*M_PI))/2 * dQ + Qhold;
        pb.set_pos(Qt);
        pb.update();
        Q = pb.get_pos();
        usleep( dt*1000*1000 );
        n++;
    }
    // return servos
    setPosition(id1, 600, usb2dyn);
    setPosition(id2, 0, usb2dyn);

    stop_thread.interrupt();   // kill the thread
    dataFile.close(); // close file
    CPhidget_close((CPhidgetHandle)ifKit);
    CPhidget_delete((CPhidgetHandle)ifKit);

    std::cout << "Exiting main..." << std::endl;
    if (video_rec)
    {
        usleep(1*1000*1000);
        socket.write_some(boost::asio::buffer(tcp_msg_stop, tcp_msg_stop.size()), ignored_error);
    }
    return 0;

}


