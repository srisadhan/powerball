#ifndef FT_SENSOR_H
#define FT_SENSOR_H

#include <stdio.h>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>
#include <TooN/TooN.h>

using namespace::std;
using boost::asio::ip::udp;

class FT_sensor {
public:



    FT_sensor();
    ~FT_sensor();

    int setTCP(char* remoteIP, char* remotePort , string mode);

    int setTare(string OnOff);

    Vector<7,float> getForce();



private:

    int startContinu();
    int stopContinu();

    std::string protocol;


};

#endif // FT_SENSOR_H
