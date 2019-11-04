#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <chrono>
#include <TooN/TooN.h>

using boost::asio::ip::udp;


void UDP_send(char* IP, char* Port, float joint_angle[])
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
    socket.send_to(boost::asio::buffer(joint_angle, 24), remote_endpoint, 0, ignored_error);
}



void UDP_receive(char* IP, char* Port, float* joint_angle, bool* NewData)
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
        if (len==24)
        {
            *NewData = false;
            std::memcpy(joint_angle,recv_buf,24);
            *NewData = true;
        } else
        {
            std::cout << "Error of Packet Size : Data With Length of " << len << " Received." << std::endl;
        }
    }
}

int main(int argc, char* argv[])
{
     /* ./vrep_client <local ip> <local port> <remote ip> <remote port> */

    // Initializing UDP communication
    float joint_angle_actual[6] = {};
    bool data_flag = false;
    boost::thread my_thread(UDP_receive,argv[1],argv[2],joint_angle_actual,&data_flag);


    float cnt=0.0f;
    float joint_angles_demand[6] = {};
    while (1)
    {
        cnt = cnt+0.2f;
        joint_angles_demand[0] = cnt;
        joint_angles_demand[1] = cnt;
        joint_angles_demand[2] = cnt;
        joint_angles_demand[3] = cnt;
        joint_angles_demand[4] = cnt;
        joint_angles_demand[5] = cnt;
        if (cnt>M_PI)
        {
            break;
        } else
        {
            if (data_flag)
            {
                std::cout << "Actual Joint Angles = " ;
                for (int n=0;n<6;n++)
                {
                    std::cout << joint_angle_actual[n] << " , ";
                }
                std::cout <<  std::endl;
                data_flag = false;
            }

            UDP_send(argv[3] , argv[4], joint_angles_demand);
            usleep(500*1000);
        }
    }


        std::cout << "Waiting for threads to be finished ..." << std::endl;
        //my_thread.join();
        my_thread.interrupt();   // kill the thread
        std::cout << "Exiting ..." << std::endl;
        return(0);
}
