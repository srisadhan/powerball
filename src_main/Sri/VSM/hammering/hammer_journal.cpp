#include <fstream>
#include <chrono>
#include <istream>
#include <string>
#include <sstream>
#include <vector>
#include <iostream>

#include "powerball/schunk_powerball.h"
#include "powerball/schunk_kinematics.h"
#include "utils/powerball_utils.h"
#include "vrep/v_repClass.h"

#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>

#include "TooN/TooN.h"
#include <TooN/LU.h>
#include <TooN/SVD.h>

#include <phidget21.h>
#include <sys/time.h>
#include "dynamixel_sdk.h"

# include <typeinfo>

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     2

#if defined(__linux__)
  //the following are UBUNTU/LINUX ONLY terminal color codes.
    #define RESET       "\033[0m"
    #define RED         "\033[31m"              /* Red */
    #define GREEN       "\033[32m"              /* Green */
    #define BOLDRED     "\033[1m\033[31m"       /* Bold Red */
    #define BOLDGREEN   "\033[1m\033[32m"       /* Bold Green */
#endif

using namespace::std;
using boost::asio::ip::tcp;
using namespace::TooN;


/*-------Global variables-----*/
// Vector<6,float> FT;
float dt = 0.005f; // sampling time
const double dxl_resolution = 300 * M_PI/180 * 25.46e-3/2 /1024;       // multiply this value with the dynamixel encode value to get displacement
double passive_mag_width = 0.03; //.028;                               // width of the magnet block holding hammer

// dynamixel calibration parameters
uint16_t VSM_MIN_POS = 525; // 555 (previous min)                      // calibrated position of the VSM to set the active magnets to 0.002 m on each side
uint16_t VSM_MAX_POS = 1023;                                           // calibrated maximum position of the active magnets in the VSM

// Initialize the PortHandler instance
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

// Initialize PacketHandler instance
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

int dxl_comm_result = COMM_TX_FAIL;         // Communication result 
uint8_t dxl_error = 0;                      // Dynamixel error
uint16_t dxl_pos_t;                         // dxl position at time step t (present position)
uint16_t dxl_pos_t1;                        // dxl position at previous time step t-1
uint8_t torque_status = 0;
uint16_t dxl_init_position = 600;           // initialize the goal position with a safe value

double K_virtual = 20;                    // initialize the virtual stiffness value for adaptation 
double force_threshold = 15.0;

// enter key press terminates the code
void stop(bool* flag){
    char in;
    cin.get(in);
    *flag = true;
}

// Check the communication (packet transfer and receiving) status of the dynamixel
void dxl_comm_status(int dxl_comm_result, uint8_t dxl_error, dynamixel::PacketHandler *packetHandler)
{
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf(RED "%s \n" RESET, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf(RED "%s \n" RESET, packetHandler->getRxPacketError(dxl_error));
    }
}

// Connect to dynamixel, set the baudrate and enable torque
bool dxl_enable(int dxl_comm_result, uint8_t dxl_error, uint16_t dxl_pos_t, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
    // Open port
    if (!portHandler->openPort())
    {
        printf(RED "Failed to open the port! \n" RESET);
        return 0;
    }

    // Set port baudrate
    if (!portHandler->setBaudRate(BAUDRATE))
    {   
        printf(RED "Failed to change the baudrate! \n" RESET);
        return 0;
    }

    // Enable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    dxl_comm_status(dxl_comm_result, dxl_error, packetHandler);

    printf(GREEN "Successfully connected to dynamixel! \n" RESET);
    return 1;
}

// disable the dynamixel
void dxl_disable(int dxl_comm_result, uint8_t dxl_error, uint16_t dxl_pos_t, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
    // Disable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    dxl_comm_status(dxl_comm_result, dxl_error, packetHandler);
}

// verify if the new dynamixel position is in the range of operation
uint16_t set_safe_dxl_pos(uint16_t new_dxl_pos)
{
    if (new_dxl_pos < VSM_MIN_POS) 
    {  
        new_dxl_pos = VSM_MIN_POS;
    }else if (new_dxl_pos > VSM_MAX_POS) 
    {   
        new_dxl_pos = VSM_MAX_POS;
    }
    return new_dxl_pos;
}

uint16_t convert_pos2dxl(float magPos){
    // uint16_t goalpos  =  (magPos - passive_mag_width/2) / dxl_resolution + VSM_MIN_POS; 
    uint16_t goalpos  =  (magPos - passive_mag_width) / dxl_resolution + VSM_MIN_POS; 
    set_safe_dxl_pos(goalpos);
    return goalpos;
}

float convert_dxl2pos(uint16_t dxl_pos){
    // return passive_mag_width/2 + (dxl_pos - VSM_MIN_POS) * dxl_resolution;
    return passive_mag_width + (dxl_pos - VSM_MIN_POS) * dxl_resolution;
}

double linPot = 0;
// Store the value of the potentiometer into linPot
int CCONV SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value)
{
    double potCenter = 513; // get the center position of the handle during calibration
    
    linPot = (Value - potCenter)*60/1024/1e3;
    // printf("Pot value: %d, distance from center: %f \n", Value, linPot);
    return 0;
}
// error 
int CCONV ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
    printf("Error handled. %d - %s", ErrorCode, unknown);
    return 0;
}

// Connect to phidget kit and set the detect if the sensor value is changed
bool phidget_enable()
{   
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
        printf("Problem connecting to phidget: %s \n", err);
        return 0;
    }

    CPhidgetInterfaceKit_setRatiometric(ifKit, 0);
    CPhidgetInterfaceKit_getSensorCount(ifKit, &numSensors);
    
    CPhidgetInterfaceKit_setSensorChangeTrigger(ifKit, 0, 0);  //0 represents higher sensitivity

    printf(GREEN "Successfully connected to Phidget with %d sensors \n" RESET, numSensors);
    return 1;
}    

// Magnet force model
double magForce(double s){
  double C1 = 28.41;
  double C2 = 206.35;

  return C1 * exp(-C2 * s);
}

// resultant force on the object
double resForce(uint16_t dxl_pos_t, double handle_position){

    // 0.29 (width of passive magnet block) is the distance between outer magnets when the dynamixel is set to position 545
    double pos1 = -convert_dxl2pos(dxl_pos_t);
    double pos2 = convert_dxl2pos(dxl_pos_t);
    
    double sep1 = handle_position - pos1 - (passive_mag_width/2);
    double sep2 = pos2 - handle_position - (passive_mag_width/2);

    if (sep1 < 0) sep1 = 0.0;
    if (sep2 < 0) sep2 = 0.0;

    // printf("%f, %f, %f \n", pos1, pos2, handle_position);
    // printf("Sep1: %f, Sep2: %f \n", sep1, sep2);
    return magForce(sep1) - magForce(sep2);
}


/*-------------------------------------------------------------------*/
// Force data from TCP
// void TCP_receive(bool *errFlag){

//     boost::asio::io_service io_service;
// //    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(host), port);
//     tcp::endpoint sender_endpoint(boost::asio::ip::address::from_string("192.168.1.30"), boost::lexical_cast<int>("1000"));

//     //socket connection
//     tcp::socket socket(io_service);
//     socket.connect(sender_endpoint);

//     boost::system::error_code ignored_error;
//     int len = 0;
//     char recv_buf[128];

//     // TARE the sensor
//     std::string msg="TARE(1)\n";
//     socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
//     len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
//     cout << BOLDGREEN <<"TCP recieved: " << recv_buf << RESET << endl;
//     usleep(1000*50);

//     // continous receiving
//     msg="L1()\n";
//     socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
//     len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
//     cout << BOLDGREEN <<"TCP recieved: " << recv_buf << RESET << endl;

//     // Force data
//     for (;;)
//     {
//         //msg="F()\n";
//         //socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
//         len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
//         int timeStamp;
//         sscanf(recv_buf,"F={%f,%f,%f,%f,%f,%f},%d",&FT[0],&FT[1],&FT[2],&FT[3],&FT[4],&FT[5],&timeStamp);
//     }
// }


/* ----Main Function----*/
// Input : -v  for vrep and robot
//         -vo for vrep only
int main(int argc, char **argv){
    int vrep_bool = 3;
    string filename = "high_stiffness";
    bool start_pos_bool = false; // to stop the code after sending the robot to the start position

    // Initializing FT sensor
    bool errFlag=false;
    bool FT_calibrated=true;
    boost::thread FT_thread(TCP_receive,&errFlag, &FT_calibrated);

    // connect to the available dynamixel
    if (!dxl_enable(dxl_comm_result, dxl_error, dxl_pos_t, portHandler, packetHandler)) return 0;
    // return 0;
    //Initializing phidget
    if (!phidget_enable()) return 0; // exit the code if cannot connect to phidget
    

    // timestamp vars
    char buffer[10];
    struct timeval tv;
    time_t curtime;

    if (argc < 2)
    {
        vrep_bool = 0;
        cout << GREEN << "Applying trajectory on PowerBall only" << RESET << endl;
    } else
    {
        if (strcmp(argv[1],"-v")==0)
        {
            vrep_bool = 1;
            cout << GREEN << "Applying trajectory on PowerBall and Vrep" << RESET << endl;
        } else if (strcmp(argv[1],"-vo")==0)
        {
            vrep_bool = 2;
            cout << GREEN << "Applying trajectory on Vrep only" << RESET << endl;
        }

        if (argc > 2){
            if (strcmp(argv[2],"start")==0){
                start_pos_bool = true;
            }else{
                filename = argv[2];
            }
            cout << filename<<endl;
        }
    }
    cout << GREEN << "Hammer trajectory set to "<< RED << filename << " condition"<< RESET << endl;

    //*------- Read file ----------*//
    cout<< "Reading file...."<<endl;
    Kin kin;
    std::vector< std::vector<double> > matrix;
    kin.inputFile("hammer_traj/"+filename + ".txt", &matrix);

    int traj_col = 0;
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
    cout << "size of imported matrix = " << nrows << "*" << ncols << endl;
    TooN::Matrix<Dynamic,Dynamic,double> imported_traj(nrows, ncols);

    // first 6 cols for the joint angles and the 7th col for the magnet position
    if (ncols == 7){

        // Put the matrix into TooN matrix
        nrows = 0;  ncols = 0;
        for (std::vector< std::vector<double> >::const_iterator it = matrix.begin(); it != matrix.end(); ++ it)
        {
            ncols = 0;
            for (std::vector<double>::const_iterator itit = it->begin(); itit != it->end(); ++ itit)
            {
                imported_traj(nrows,ncols) = *itit;
                ncols++;
            }
            nrows++;
            traj_col = ncols;
        }
    }else{
        cout<< BOLDRED <<"Inconsistent matrix size for trajectory generation" << RESET << endl;
        return 0;
    }
    cout << "File read succesful" <<endl;
    /*------------------------------------*/

    /* Set sampling and timing options*/
    std::chrono::time_point<std::chrono::system_clock> timeLoop;
    std::chrono::duration<float> elaps_loop;
    /*-----------------------------------------*/
    /*  Powerball class */
    SchunkPowerball pb;
    pb.update();

    /* VREP Class */
    TooN::Vector<6,float> joint_angle = Zeros;
    Vector<6,float> joint_angle_initializing = Zeros;

    // joint direction  in vrep
    int res = -1;
    V_rep vrep;
    if (vrep_bool!=0)
    {
        res = vrep.connect();
        if (res==-1)
        {
            cout << BOLDRED <<"V-REP Connection Error" << RESET << endl;
            return 0;
        }
    }

    /*Bring the end-effector of the robot to the starting point of trajectory*/
    Vector<6,float>Q = pb.get_pos();
    Vector<6,float>Qs ; //= Data(0.0f, -0.523599f, 1.76278f, 0.0f, 0.802851f, 0.0f) ;//joint angle for starting position
    Vector<6,float>Qdot = Zeros;
    Matrix<3,4,float> T_mat; //=Data(0,0,1,0.48,0,1,0,0,-1,0,0,0.2);
    Vector<3,float> pos_temp = Zeros;
    kin.FK_pos(Qs,&pos_temp);

    // starting point of the traj
    for (int n=0;n<6;n++)
    {
        joint_angle[n] = imported_traj(0,n);
    }

    Qs = joint_angle;

    // also set the the dynamixel position
    dxl_pos_t  =  convert_pos2dxl(imported_traj(0,6));
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_pos_t, &dxl_error);
    dxl_comm_status(dxl_comm_result, dxl_error, packetHandler);

    // interpolate the traj between current robot position and the starting point of the traj
    std::vector<std::vector<double>>interpolated_data;
    // float T_travel = calculate_travel_time(Q, Qs, 10); // used 20 deg/s instead of 72 deg/s for safety
    // printf("Please wait! Estimated time to travel to start position is : %f \n", T_travel);
    
    int num_H = kin.HerInter(Q,Qdot,Qs,0.005f,5,&interpolated_data);

    int i = 0;
    int col,row = 0;
    Matrix<Dynamic,6,double> reach_start(num_H, 6);
    for (std::vector< std::vector<double> >::const_iterator it = interpolated_data.begin(); it != interpolated_data.end(); ++ it)
    {
       col = 0;
       for (std::vector<double>::const_iterator itit = it->begin(); itit != it->end(); ++ itit)
       {
           reach_start(row,col) = *itit;
           col++;
       }
       row++;
    }
    cout<<"Homing....."<<endl;
    while(i<num_H)
    {
        timeLoop = std::chrono::system_clock::now();
        if (vrep_bool!=2)
        {
          pb.set_pos(reach_start[i]);
          pb.update();
        }
        if (res!=-1)
        {
            if (vrep.isConnected())
            {
                for (int n=0;n<6;n++)
                {
                    joint_angle_initializing[n] = reach_start(i,n);
                }
                vrep.setq(joint_angle_initializing);
            } else
            {
                cout << BOLDRED <<"V-REP Disconnected!" << RESET << endl;
                return 0;
            }
        }
        i++;

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }
    }
    cout<<"Finished sending the robot to the start point of the trajectory"<<endl;

    if (start_pos_bool){
        return 0;
    }

    std::ofstream dataFile;
    dataFile.open("hammer_traj/output_files/"+filename+"_output.csv");
    dataFile<< "Time,X,Y,Fx,Fy,Fz,Mx,My,Mz,Potentiometer_bit,handle_disp, magnet_pos"<< endl;

    /*------------------*/
    Vector<2,float> pos = Zeros;
    Matrix<3,4,double> T_mat1 = Zeros;
    Vector<6,float> Q_present = pb.get_pos();
    Vector<6,float> Q_IK = Zeros;
    i = 0;

    Vector<3,float> X = Zeros; // position of end-effector
    // Trajectory and data acquisition start
    while(i<nrows)
    {
        timeLoop = std::chrono::system_clock::now();

        dxl_pos_t  = convert_pos2dxl(imported_traj(i,6));
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_pos_t, &dxl_error);
        dxl_comm_status(dxl_comm_result, dxl_error, packetHandler);

        for (int n=0;n<6;n++)
        {
            joint_angle[n] = imported_traj(i,n);
        }
        if (vrep_bool!=2)
        {
            pb.set_pos(joint_angle);
        }
        pb.update();
        kin.FK_pos(joint_angle,&X);

        if (res!=-1)
        {
            if (vrep.isConnected())
            {
                vrep.setq(joint_angle);
            } else
            {
                cout << "V-REP Disconnected!" << endl;
                return 0;
            }
        }

        // timestamp
        gettimeofday(&tv, NULL);
        curtime=tv.tv_sec;
        strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));
        dataFile << buffer << ":" << tv.tv_usec << ",";
        dataFile<< X[0]<< "," <<X[1] <<"," <<FT[0] <<","<< -FT[1] << "," << FT[2] << "," << FT[3] << "," << FT[4] << "," << FT[5] << "," << linPot << "," << (linPot - 514) * 0.06/1023 << "," << dxl_pos_t <<endl;

        i++;
        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
            // cout << (elaps_loop.count())<< endl;
        }
        else {cout << "Communication Time Out!" << endl;}
    }
    // Keep collecting the position and force information for 1 sec after the hammer hits the nail
    float counter = 0.0;
    while(counter < 1){
        counter += dt;
        // timestamp
        gettimeofday(&tv, NULL);
        curtime=tv.tv_sec;
        strftime(buffer, 10, "%H:%M:%S", localtime(&curtime));
        dataFile << buffer << ":" << tv.tv_usec << ",";
        dataFile<< X[0]<< "," <<X[1] <<"," <<FT[0] <<","<< -FT[1] << "," << FT[2] << "," << -FT[3] << "," << FT[4] << "," << FT[5] << "," << linPot << "," << (linPot - 468) * 0.06/1023 <<endl;

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }

    }
    // shutdown the powerball motors
    pb.update();

    usleep(1000*500);

    cout << "Exiting main..." << endl;
    printf("Closing phidget...\n");
    // CPhidget_close((CPhidgetHandle)ifKit);
    // CPhidget_delete((CPhidgetHandle)ifKit);
    FT_thread.interrupt();
    dataFile.close();
    return 0;
}
