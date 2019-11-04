#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <chrono>
#include <TooN/TooN.h>

using boost::asio::ip::udp;


void UDP_send(char *IP, char *Port, TooN::Vector<6, float> joint_angle_actual)
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

    boost::system::error_code ignored_error;

    std::cout << "Send to " << remote_endpoint << std::endl;

    socket.send_to(boost::asio::buffer(&joint_angle_actual, 24), remote_endpoint, 0, ignored_error);

}


void UDP_receive(double* joint_angle_demand)
{
    std::cout << "thread is running" << std::endl;

    boost::asio::io_service io_service;
    char recv_buf[128];
    udp::endpoint sender_endpoint;
    udp::endpoint local_endpoint = boost::asio::ip::udp::endpoint(
            boost::asio::ip::address::from_string("127.0.0.1"), boost::lexical_cast<int>("25000"));

    std::cout << "Local bind " << local_endpoint << std::endl;

    udp::socket socket(io_service);
    socket.open(udp::v4());
    socket.bind(local_endpoint);

    //std::chrono::time_point<std::chrono::system_clock> time1;
    //std::chrono::duration<float> elaps;

    int len=0 , cnt=0;
    while(1)
    {
        cnt++;

        // time1 = std::chrono::system_clock::now();

        len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);
        //std::cout << "len = " << len << std::endl;
        if (len==28)
        {
            std::memcpy(joint_angle_demand,recv_buf,28);
            /*
            for (int n=0;n<6;n++)
            {
                std::cout << "joint " << n << " = " << joint_angle_demand[n] << std::endl;
            }
            */
        }

       // elaps = std::chrono::system_clock::now() - time1;
       // std::cout << "dt = " << elaps.count() << std::endl;
    }

}


int main(int argc, char* argv[])
{
    double rec[1] = {};

    boost::thread my_thread(UDP_receive,rec);

    while(1)
    {
        for (int n=0;n<1;n++)
        {
            std::cout << "joint " << n << " = " << rec[n] << std::endl;
        }
        usleep(1000*1000);
    }


    my_thread.join();
    my_thread.interrupt();   // kill the thread

  return 0;
}
