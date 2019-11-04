#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <boost/thread.hpp>
#include "powerball/schunk_powerball.h"
#include "vrep/v_repClass.h"
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>
#include <chrono>

using namespace::std;
using boost::asio::ip::udp;


void UDP_send(char* IP, char* Port, float data[])
{

    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket(io_service);
    boost::asio::ip::udp::endpoint remote_endpoint;
    bool broadcast = true;

    socket.open(boost::asio::ip::udp::v4());

    // TO ENABLE BROADCAST
    if(broadcast)
      {
      boost::asio::socket_base::broadcast option(true);
      socket.set_option(option);
      }

    remote_endpoint = boost::asio::ip::udp::endpoint(
    boost::asio::ip::address::from_string(IP),  boost::lexical_cast<int>(Port));

    //std::cout << "Data Sent to " << remote_endpoint << std::endl;

    boost::system::error_code ignored_error;
    socket.send_to(boost::asio::buffer(data, 28), remote_endpoint, 0, ignored_error);
}


void UDP_receive(char* IP, char* Port, float* data, bool* NewData)
{
    *NewData = false;
    std::cout << "UDP Receiver thread started" << std::endl;

    boost::asio::io_service io_service;
    char recv_buf[128];
    udp::endpoint sender_endpoint;
    udp::endpoint local_endpoint = boost::asio::ip::udp::endpoint(
            boost::asio::ip::address::from_string(IP), boost::lexical_cast<int>(Port));

    std::cout << "Local bind " << local_endpoint << std::endl;

    udp::socket socket(io_service);
    socket.open(udp::v4());
    socket.bind(local_endpoint);

    int len=0;
    while(1)
    {
        len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);
        if (len==28)
        {
            *NewData = false;
            std::memcpy(data,recv_buf,28);
            *NewData = true;
        } else
        {
            std::cout << "Error of Packet Size : Data With Length of " << len << " Received." << std::endl;
        }
    }

}


int main(int argc, char* argv[])
{
    /* ./vrep_server <local ip> <local port> <remote ip> <remote port> */

    // Initializing UDP communication
    float data[7] = {};
    float demand_joint_angles[6] = {};
    bool data_flag = false;
    boost::thread my_thread(UDP_receive,argv[1],argv[2],data,&data_flag);


    // vrep loop
    V_rep vrep;
    float Ts = 0.01; // time interval s
    std::chrono::duration<float> elaps_loop;
    std::chrono::time_point<std::chrono::system_clock> timeLoop;
    timeLoop = std::chrono::system_clock::now();

    int res = vrep.connect();
    if (res!=-1)
    {
        while (vrep.isConnected())
        {
            timeLoop = std::chrono::system_clock::now();

            if (data_flag)
            {

                std::cout << "Demand Joint Angles = " ;
                for (int n=0;n<7;n++)
                {
                    std::cout << data[n] << " , ";
                }
                std::cout << std::endl;
                data_flag = false;

                for (int n=0;n<6;n++)
                {
                    demand_joint_angles[n] = data[n];
                }
                vrep.setq(demand_joint_angles);
            }

            UDP_send(argv[3],argv[4],data);

            elaps_loop = std::chrono::system_clock::now() - timeLoop;
            if ( (Ts-elaps_loop.count()) > 0 ) {
                usleep( (Ts-elaps_loop.count())*1000*1000 );
            }

        }
    } else
    {
        cout << "V-REP Connection Error" << endl;
    }


    cout << "Waiting for threads to be finished ..." << endl;
    //my_thread.join();
    my_thread.interrupt();   // kill the thread
    cout << "Exiting ..." << endl;
	return(0);
}

