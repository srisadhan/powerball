#include "powerball/schunk_powerball.h"
#include "powerball/schunk_kinematics.h"
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


// global vars
Vector<6,float> FT;

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
    return 0; }

void TCP_receive(bool *escFlag)
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
    //std::string msg="TARE(1)\n";
    //socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
    //len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
    //cout << "TCP recieved: " << recv_buf << endl;

    // continous receiving
    //msg="L1()\n";
    //socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
    //len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
    //cout << "TCP recieved: " << recv_buf << endl;

    // Read one data
    std::string msg="F()\n";
    socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
    cout << "TCP wrote " << msg << endl;
    len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
    cout << "TCP recieved: " << recv_buf << endl;

    // Force data
    while(!(*escFlag))
    {
        msg="F()\n";
        socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
        len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
        int timeStamp;
        sscanf(recv_buf,"F={%f,%f,%f,%f,%f,%f},%d",&FT[0],&FT[1],&FT[2],&FT[3],&FT[4],&FT[5],&timeStamp);
        usleep(0.001*1000*1000);
    }
    cout << "Ending TCP thread" << endl;
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

    // Set sampling and timing options
    float dt = 0.005f;
    std::chrono::time_point<std::chrono::system_clock> timeLoop;
    std::chrono::duration<float> elaps_loop;

    int result, numSensors;
    const char *err;
    USB2Dynamixel usb2dyn(*cnfDevice, 50);
    int id1 = 1, id2 = 2;


    // homing for the gripper
    setMaxTorque(id1, 500, usb2dyn); // outer magnets max 1023
    setMaxTorque(id2, 500, usb2dyn); // inner magnets
    int servo_out=850, servo_in=100;
    setPosition(id1, servo_out, usb2dyn); //850
    setPosition(id2, servo_in, usb2dyn); //100
    usleep( 1*1000*1000 );


    // use for adjusting servos
    /*
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


    // load input file to a matrix
    Kin kin;
    std::vector< std::vector<double> > matrix;
    if (!(kin.inputFile("RockStacking.txt",&matrix)))
    {
        return 0;
    }
    TooN::Matrix<Dynamic,Dynamic,double> traj(kin.nrows, kin.ncols);
    cout << "size of matrix = " << kin.nrows << "*" << kin.ncols << endl;
    int  col=0; int row=0;
    for (std::vector< std::vector<double> >::const_iterator it = matrix.begin(); it != matrix.end(); ++ it)
    {
        col = 0;
        for (std::vector<double>::const_iterator itit = it->begin(); itit != it->end(); ++ itit)
        {
            traj(row,col) = *itit;
            col++;
        }
        row++;
    }


    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;


    // Open recording file
    std::ofstream dataFile;
    dataFile.open("RockStacking_readings.csv");
    dataFile << "This is file header" << endl;

    // VREP Class
    Vector<6,float> Qvrep_des = Zeros;
    // joint direction  in vrep
    int res = -1;
    V_rep vrep;
    if (vrep_bool!=0)
    {
        res = vrep.connect();
        if (res==-1)
        {
            cout << "V-REP Connection Error" << endl;
            return 0;
        }
    }


    // Powerball class
    SchunkPowerball pb;
    pb.update();
    Vector<6,float> Q = Zeros, Qdot = Zeros;
    Q = pb.get_pos();
    Qdot = pb.get_vel();

    // Safety stop by Enter key thread
    bool stop_flag = false;
    boost::thread stop_thread(stop,&stop_flag);


    // initializing TCP commanding
    boost::asio::io_service io_service;
    tcp::endpoint sender_endpoint = boost::asio::ip::tcp::endpoint(
                boost::asio::ip::address::from_string("127.0.0.1"),  boost::lexical_cast<int>("40000"));
    tcp::socket socket(io_service);
    socket.connect(sender_endpoint);
    boost::system::error_code ignored_error;
    std::string tcp_msg_start="s";
    std::string tcp_msg_stop="e";
    //socket.write_some(boost::asio::buffer(tcp_msg, tcp_msg.size()), ignored_error);
    //

    // Initializing FT sensor
    bool escFlag=false;
    boost::thread FT_thread(TCP_receive,&escFlag);
    usleep(0.010*1000*1000);

    for (int trial=0;trial<row;trial++)
    {
        /*
        // ask for trial
        char response='n';
        while (response != 'y')
        {
            cout << "Do you want trial " << trial << " x:"<< traj(trial,0)*1000 <<" , theta:"<< traj(trial,1)/M_PI*180 << endl;
            cin >> response;
            cout << response << endl;
            if (response != 'y')
            {
                trial++;
            }
        }
        pb.set_control_mode(MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE);
        */

        // initial position
        cout << "Going to initial pose" << endl;
        //Vector<6,float> Qref=makeVector(0.0 , -0.4068 , 2.4369 , 0 , -0.8219 , -1.5708);
        //Vector<6,float> Qref=makeVector(0.0 , -0.3243 , 2.4254 , 0 , -0.728 , -1.5708);
        Vector<6,float> Qref=makeVector(0, -0.324282, 2.4254, 0, -0.727977, -1.5708);


        Vector<6,float> dQ = Qref-Q;
        float maxq=0;
        for (int n=0;n<6;n++)
        {
            if (abs(dQ[n])>maxq) {maxq=abs(dQ[n]);}
        }
        float Ttravel = maxq*5;
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

        // wait for tapping
        cout << "Wait for tapping" << endl;
        cout << "Do you want trial " << trial << ", x:"<< traj(trial,0)*1000 <<" , theta:"<< traj(trial,1)/M_PI*180 << endl;
        float F_yesNo = FT[1];
        float F_reset = FT[0];
        double dF1 = 0, dF2 = 0, tapThreshold = 2.0;
        bool reset = true;
        while((dF1<=tapThreshold) && (!stop_flag))
        {
            dF1 = FT[1]-F_yesNo;
            if ((dF1<-tapThreshold) && (reset))
            {
                reset = false;
                trial++;
                cout << "Do you want trial " << trial << ", x:"<< traj(trial,0)*1000 <<" , theta:"<< traj(trial,1)/M_PI*180 << endl;
            }

            dF2 = FT[0]-F_reset;
            if ((dF2>tapThreshold) && (!reset))
            {
                reset = true;
                cout << "reset" << endl;
            }

            pb.update();
            Q = pb.get_pos();
            usleep( dt*1000*1000 );
        }


        // go pickup-up
        cout << "Going for pick-up" << endl;
        Matrix<3,4,float> T_mat = Zeros;
        kin.FK_T(Q,&T_mat);
        T_mat(1,3) = -0.2;
        kin.IK(T_mat,Q,&Qref);
        dQ = Qref-Q;
        Ttravel = 2.5;
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
        servo_out=900; servo_in=700;
        setPosition(id1, servo_out, usb2dyn);
        setPosition(id2, servo_in, usb2dyn);


        // go pickup-down
        T_mat = Zeros;
        kin.FK_T(Q,&T_mat);
        T_mat(2,3) = 0.045;
        kin.IK(T_mat,Q,&Qref);
        dQ = Qref-Q;
        Ttravel = 2.0;
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
        servo_out=500; servo_in=200;
        setPosition(id1, servo_out, usb2dyn);
        setPosition(id2, servo_in, usb2dyn);


        // wait for grasp
        n = 1;
        while((n<=250) && (!stop_flag))
        {
            pb.update();
            Q = pb.get_pos();
            usleep( dt*1000*1000 );
            n++;
        }

        // go pickup-up
        T_mat = Zeros;
        kin.FK_T(Q,&T_mat);
        T_mat(2,3) = 0.15;
        kin.IK(T_mat,Q,&Qref);
        dQ = Qref-Q;
        Ttravel = 2.0;
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

        // go to Y assigned
        cout << "Go to Y assigned" << endl;
        T_mat = Zeros;
        kin.FK_T(Q,&T_mat);
        T_mat(1,3) = traj(trial,0); // assig y position
        kin.IK(T_mat,Q,&Qref);
        cout << Qref << endl;
        cout << traj(trial,1) << endl;
        Qref[5] = Qref[5] + traj(trial,1); // assign rotation at joint 6
        cout << Qref << endl;
        dQ = Qref-Q;
        Ttravel = 3.0;
        itNum = Ttravel/dt;
        Qhold = Q;
        n = 1;
        Vector<5,int> s1 = Zeros; s1[0]=x[0]; s1[1]=x[0]; s1[2]=x[0]; s1[3]=x[0]; s1[4]=x[0]; // for collision detection
        while((n<=itNum) && (!stop_flag))
        {
            Vector<6,float> Qt = (1-cos(float(n)/itNum*M_PI))/2 * dQ + Qhold;
            pb.set_pos(Qt);
            pb.update();
            Q = pb.get_pos();

            /* collision detection
            s1[0] = s1[1];
            s1[1] = s1[2];
            s1[2] = s1[3];
            s1[3] = s1[4];
            s1[4] = x[0];
            float ds = abs( -s1[4]+8*s1[3]-8*s1[1]+s1[0] ); // velocity calc
            //cout << ds << endl;
            if (ds>18)
            {
                stop_flag=true;
                cout << "Motors Stopped By Collision Detection. ds = " << ds << endl;
            }
            */

            usleep( dt*1000*1000 );
            n++;
        }


        n = 1;
        while((n<=100) && (!stop_flag))
        {
            pb.update();
            Q = pb.get_pos();
            usleep( dt*1000*1000 );
            n++;
        }

        // go down till touch
        cout << "Go down till touch" << endl;
        T_mat = Zeros;
        kin.FK_T(Q,&T_mat);
        T_mat(2,3) = 0.05;
        kin.IK(T_mat,Q,&Qref);
        dQ = Qref-Q;
        Ttravel = 12.0;
        itNum = Ttravel/dt;
        Qhold = Q;
        n = 1;

        Vector<3,float> T_now=FT.slice<3,3>();
        float dF = 0;
        tapThreshold = 0.02;
        CPhidgetInterfaceKit_setOutputState(ifKit, 0, 1);
        while((n<=itNum) && (dF<=tapThreshold) && (!stop_flag))
        {
            timeLoop = std::chrono::system_clock::now();

            // timestamp
            gettimeofday(&tv, NULL);
            curtime=tv.tv_sec;
            strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));
            dataFile << trial << "," << "state1," << buffer << ":" << tv.tv_usec << ",";

            Vector<6,float> Qt = (1-cos(float(n)/itNum*M_PI))/2 * dQ + Qhold;
            pb.set_pos(Qt);
            pb.update();
            Q = pb.get_pos();

            dataFile<<Q[0]<<","<<Q[1]<<","<<Q[2]<<","<<Q[3]<<","<<Q[4]<<","<<Q[5]<<","
                         <<FT[0]<<","<<FT[1]<<","<<FT[2]<<","<<FT[3]<<","<<FT[4]<<","<<FT[5]<< ","
                        <<x[0] <<","<< x[1] << endl;

            dF = TooN::norm(FT.slice<3,3>() - T_now);
            if (dF>tapThreshold) {cout << "Surface touched" << endl;}

            elaps_loop = std::chrono::system_clock::now() - timeLoop;
            if ( (dt-elaps_loop.count()) > 0 ) {
                usleep( (dt-elaps_loop.count())*1000*1000 );
            }
            n++;
        }


        // wait 1 sec
        Ttravel = 1.0;
        itNum = Ttravel/dt;
        n = 1;
        while((n<=itNum) && (!stop_flag))
        {
            timeLoop = std::chrono::system_clock::now();

            // timestamp
            gettimeofday(&tv, NULL);
            curtime=tv.tv_sec;
            strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));
            dataFile << trial << ","  << "state2," << buffer << ":" << tv.tv_usec << ",";

            pb.update();
            Q = pb.get_pos();

            dataFile<<Q[0]<<","<<Q[1]<<","<<Q[2]<<","<<Q[3]<<","<<Q[4]<<","<<Q[5]<<","
                         <<FT[0]<<","<<FT[1]<<","<<FT[2]<<","<<FT[3]<<","<<FT[4]<<","<<FT[5]<< ","
                        <<x[0] <<","<< x[1] << endl;


            elaps_loop = std::chrono::system_clock::now() - timeLoop;
            if ( (dt-elaps_loop.count()) > 0 ) {
                usleep( (dt-elaps_loop.count())*1000*1000 );
            }
            n++;
        }


        // tcp messagr for video recording
        socket.write_some(boost::asio::buffer(tcp_msg_start, tcp_msg_start.size()), ignored_error);

        // release grasp
        int Pservo_out=servo_out, Pservo_in=servo_in;
        cout << "Releasing Grasp" << endl;
        Ttravel = 6.0;
        itNum = Ttravel/dt;
        n = 1;
        while((n<=itNum) && (!stop_flag))
        {
            timeLoop = std::chrono::system_clock::now();

            // timestamp
            gettimeofday(&tv, NULL);
            curtime=tv.tv_sec;
            strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));
            dataFile << trial << "," << "state3," << buffer << ":" << tv.tv_usec << ",";

            if (n*dt<Ttravel/2)
            {
                float tmp = Pservo_out + n*dt*2/Ttravel*(950-Pservo_out);
                servo_out = int(tmp);
                setPosition(id1, servo_out, usb2dyn);
            }
            else
            {
                float tmp = Pservo_in + (n*dt*2/Ttravel-1)*(600-Pservo_in);
                servo_in = int(tmp);
                setPosition(id2, servo_in, usb2dyn);
            }

            pb.update();
            Q = pb.get_pos();

            dataFile<<Q[0]<<","<<Q[1]<<","<<Q[2]<<","<<Q[3]<<","<<Q[4]<<","<<Q[5]<<","
                         <<FT[0]<<","<<FT[1]<<","<<FT[2]<<","<<FT[3]<<","<<FT[4]<<","<<FT[5]<< ","
                        <<x[0] <<","<< x[1] << endl;


            elaps_loop = std::chrono::system_clock::now() - timeLoop;
            if ( (dt-elaps_loop.count()) > 0 ) {
                usleep( (dt-elaps_loop.count())*1000*1000 );
            }
            n++;
        }
        CPhidgetInterfaceKit_setOutputState(ifKit, 0, 0);


        // go up
        T_mat = Zeros;
        kin.FK_T(Q,&T_mat);
        T_mat(2,3) = 0.15;
        kin.IK(T_mat,Q,&Qref);
        dQ = Qref-Q;
        Ttravel = 2.0;
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
        socket.write_some(boost::asio::buffer(tcp_msg_stop, tcp_msg_stop.size()), ignored_error);

cout << Q << endl;
    } // end for trial loop
    setPosition(id1, 850, usb2dyn); //850
    setPosition(id2, 100, usb2dyn); //100


    escFlag = true;
    usleep(0.2*1000*1000);
    stop_thread.interrupt();   // kill the threads
    FT_thread.detach();
    dataFile.close(); // close file
    CPhidget_close((CPhidgetHandle)ifKit);
    CPhidget_delete((CPhidgetHandle)ifKit);
    socket.close();
    usleep(0.2*1000*1000);
    std::cout << "Exiting main..." << std::endl;
    return 0;

}


