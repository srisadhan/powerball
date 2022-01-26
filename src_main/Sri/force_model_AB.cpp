// Robot movement to estimate the force model for Abhinaba's foot orthosis device
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
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include "TooN/TooN.h"
#include <TooN/LU.h>
#include <TooN/SVD.h>
#include <sys/time.h>
#include <commonOptions.h>

# include <typeinfo>

using namespace::std;
using boost::asio::ip::tcp;
using namespace::TooN;


/*-------Global variables-----*/
Vector<6,float> FT;

//the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET       "\033[0m"
#define RED         "\033[31m"              /* Red */
#define GREEN       "\033[32m"              /* Green */
#define BOLDRED     "\033[1m\033[31m"       /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"       /* Bold Green */


/*-------------------------------------------------------------------*/
// Force data from TCP
void TCP_receive(bool *errFlag){

    boost::asio::io_service io_service;
//    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(host), port);
    tcp::endpoint sender_endpoint(boost::asio::ip::address::from_string("192.168.1.30"), boost::lexical_cast<int>("1000"));

    //socket connection
    tcp::socket socket(io_service);
    socket.connect(sender_endpoint);

    boost::system::error_code ignored_error;
    int len = 0;
    char recv_buf[128];

    // TARE the sensor
    std::string msg="TARE(1)\n";
    socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
    len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
    cout << BOLDGREEN <<"TCP recieved: " << recv_buf << RESET << endl;
    usleep(1000*50);

    // continous receiving
    msg="L1()\n";
    socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
    len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
    cout << BOLDGREEN <<"TCP recieved: " << recv_buf << RESET << endl;

    // Force data
    for (;;)
    {
        //msg="F()\n";
        //socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
        len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
        int timeStamp;
        sscanf(recv_buf,"F={%f,%f,%f,%f,%f,%f},%d",&FT[0],&FT[1],&FT[2],&FT[3],&FT[4],&FT[5],&timeStamp);
    }
}


/* ----Main Function----*/
// Input : -v  for vrep and robot
//         -vo for vrep only

int main(int argc, char **argv){
    int vrep_bool = 3;
    string filename = "test";
    bool start_pos_bool = false; // to stop the code after sending the robot to the start position

    // Initializing FT sensor
    bool errFlag=false;
    boost::thread FT_thread(TCP_receive,&errFlag);


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
    cout << GREEN << "Robot trajectory set to "<< RED << filename << " condition"<< RESET << endl;

    //*------- Read file ----------*//
    cout<< "Reading file...."<<endl;
    Kin kin;
    std::vector< std::vector<double> > matrix;
    kin.inputFile("AB/traj/"+filename + ".txt", &matrix);

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
    if (ncols == 6){

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
    float dt = 0.02f;
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


    // interpolate the traj between current robot position and the starting point of the traj
    std::vector<std::vector<double>>interpolated_data;
    int num_H = kin.HerInter(Q,Qdot,Qs,dt,1,&interpolated_data);

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
    dataFile.open("AB/output_files/"+filename+"_output.csv");
    dataFile<< "Time,X,Y,Fx,Fy,Fz,Mx,My,Mz"<< endl;

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
        dataFile<< X[0]<< "," <<X[1] <<"," <<FT[0] <<","<< FT[1] << "," << FT[2] << "," << FT[3] << "," << FT[4] << "," << FT[5] <<endl;

        i++;
        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ( (dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
            // cout << (elaps_loop.count())<< endl;
        }
    }
    // shutdown the powerball motors
    pb.update();

    // usleep(1000*500);

    cout << "Exiting main..." << endl;
    printf("Closing phidget...\n");
    FT_thread.interrupt();
    dataFile.close();
    return 0;
}
