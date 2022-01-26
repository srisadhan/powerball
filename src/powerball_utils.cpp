#include <stdio.h>
#include <iostream>
#include <math.h>

#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/array.hpp>

# include <powerball_utils.h>
# include "utils.h"

using boost::asio::ip::tcp;

TooN::Vector<6, float> FT;

/**
 * \brief find travel time for the robot
 * \param Q_start, starting pose of the robot
 * \param Q_end, destination pose of the robot
 * \param max_Qdot, max velocity of the robot in degree/s, values in between 0 and 72
 * \return travel_time
 */
float calculate_travel_time(TooN::Vector<6, float> Q_start, TooN::Vector<6, float> Q_end, float max_Qdot)
{
    TooN::Vector<6, float> dQ = (Q_end - Q_start) * 180/M_PI;
    // maximum value of the change in joint angle 
    float max_dq = 0;
    for (int i=0; i<6; i++){
        if (max_dq < abs(dQ[i]))
        {
            max_dq = abs(dQ[i]);
        }
    } 
    // the velocity limits of joints are 72 degree/s according to the manual
    // the acceleration limits of joints are 150 degree/s^2 according to the manual
    if (max_Qdot < 0)
    {
        max_Qdot = 0;
    }else if (max_Qdot > 72)
    {
        max_Qdot = 72;
    }

    float T = max_dq / max_Qdot; 
    // if (T < 1.0) T = 1.0;

    return T;
}

/**
 * \brief Read force-torque values from the Weiss KMS40 sensor
 * \param stopFlag, Read the sensor data until this flag is true
 * \param FT_calibrated, change this flag to true after TARE the sensor
 */
void TCP_receive(bool *stopFlag, bool *FT_calibrated)
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
    std::cout << "TCP recieved: " << recv_buf << std::endl;


    if (strcmp(&recv_buf[0], "TARE=1\n") == 0)
    {
        printf(GREEN "Successfully TARE the sensor \n" DEFAULT);
        *FT_calibrated = true;
    }else if(strcmp(&recv_buf[0], "ERROR(18)\n") == 0)
    {
        printf(RED "Unable to TARE the sensor, terminating the code \n" DEFAULT);
        *stopFlag = true;
    }

    // continous receiving
    msg="L1()\n";
    socket.write_some(boost::asio::buffer(msg, msg.size()), ignored_error);
    len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
    std::cout << "TCP recieved: " << recv_buf << std::endl;

    // Force data
    for (;;)
    {
        len = socket.read_some(boost::asio::buffer(recv_buf), ignored_error);
        int timeStamp;
        sscanf(recv_buf,"F={%f,%f,%f,%f,%f,%f},%d",&FT[0],&FT[1],&FT[2],&FT[3],&FT[4],&FT[5],&timeStamp);
    }
}


//---------------These 3 functions are for the robotiq force-sensor------------//
/**
* \brief check the state of Robotiq FT 300 sensor
* \param max_retries_, maximum number of retries to check the running state of the sensor
* \param ftdi_id, check the ID of the connected sensor
* \return state of the sensor (type int)
**/
static INT_8 sensor_state_machine(unsigned int max_retries_, std::string &ftdi_id)
{
    if (ftdi_id.empty())
    {
        return rq_sensor_state(max_retries_);
    }

    return rq_sensor_state(max_retries_, ftdi_id);
}

/**
* \brief wait for the connection to be established with the force sensor
* \param max_retries_, maximum number of retries to check the running state of the sensor
* \param ftdi_id, check the ID of the connected sensor
**/
int wait_for_connection(unsigned int max_retries_, std::string &ftdi_id)
{   
    INT_8 ret;
    ret = sensor_state_machine(max_retries_, ftdi_id);
	if(ret == -1)
	{
		usleep(1e5);
        ret = sensor_state_machine(max_retries_, ftdi_id);
	}
    printf("Current state of the FT-sensor: %d\n", rq_sensor_get_current_state());
    
    return ret;
}

/**
* \brief Read the force value from the sensor at an interval of dt
* \param dt, (float) the period at which the data is to be obtained
* \param Tare, true if the sensor has to be TARED
* \param stopFlag, bool to stop reading data from the sensor
* \param FT_calibrated, change the status of this flag to true afte the force-sensor has been TARED
**/
void Robotiq_ft_receive(float dt, bool Tare, bool *stopFlag, bool *FT_calibrated)
{   
    INT_8 serial_number;
    std::string ftdi_id;
    INT_8 ret, i;
    static int max_retries_(100);

    // timestamp vars
    std::chrono::time_point<std::chrono::system_clock> timeLoop;
    std::chrono::duration<float> elaps_loop;

    //If we can't initialize, we return an error
	ret = wait_for_connection(max_retries_, ftdi_id);

	//Reads basic info on the sensor
	ret = wait_for_connection(max_retries_, ftdi_id);

	//Starts the stream
	ret = wait_for_connection(max_retries_, ftdi_id);

    if(ret == 0)
    {
        printf(GREEN "FT-Sensor connected!\n" DEFAULT);
    }else
    {
        printf(RED "ERROR connecting to the Robotiq FT sensor\n" DEFAULT);
    }

    // Tare the sensor
    if (Tare)
    {   
        printf(GREEN "TARE the sensor\n" DEFAULT);
        rq_state_do_zero_force_flag();
        usleep(5e5);
        
        // check if the force offset has been tared
        float vector_norm = 0;
        for(i=0; i < 6; i++)
        {
            FT[i] = rq_state_get_received_data(i);
            vector_norm  += FT[i] * FT[i];
        }
        vector_norm = sqrt(vector_norm);
        
        // TARE the sensor again if previous command was unable to TARE the sensor
        while (vector_norm > 10)
        {
            rq_state_do_zero_force_flag();
            usleep(1e5);
            vector_norm = sqrt(vector_norm);
        }

        rq_sensor_state(max_retries_);
        *FT_calibrated = true;
    }

    // char recv_buf[512];
    while(!(*stopFlag))
    {   
        timeLoop = std::chrono::system_clock::now();
        rq_sensor_state(max_retries_);

        if (rq_sensor_get_current_state() == RQ_STATE_RUN)
        {
            for(i=0; i < 6; i++)
            {
                FT[i] = rq_state_get_received_data(i);
                // printf("Force data:{%f, %f, %f, %f, %f, %f}\n", FT[0],FT[1],FT[2],FT[3],FT[4],FT[5]);
            }
        }
        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ((dt-elaps_loop.count()) > 0 ) {
            usleep( (dt-elaps_loop.count())*1000*1000 );
        }
        else {printf("Communication Time Out!\n");}
    }
    
}