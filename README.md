# C++ library for schunk powerball lwa4p (linux)
### contributor: Amir Memar

- Description of files can be found in [docs/file_description.md](docs/file_description.md)
# Required libraries
### Hardware
- ntcan (CAN-USB) : https://esd.eu/en 
(Download the latest driver from https://esd.eu/en/software-downloads/27071 - ntcanSck (or) install it from the ESD driver CD). 
  - Socketcan : http://www.pengutronix.de/software/libsocketcan/download/
  - iproute2  : https://github.com/shemminger/iproute2
    - iproute2 might require flex and bison (sudo apt-get install flex; sudo apt-get install bison). "ip link show : should show the connected can device"
    if you face issues with the installation (aligned_u64), try this https://github.com/cloudflare/bpftools/commit/b1bbc6b2a35be84d38355f5e9da1382554ba7f70

- Phidget21 : https://www.phidgets.com/downloads/phidget21/libraries/linux/libphidget/

### Software
- Boost library : sudo apt-get install libboost-all-dev
- BLAS and LAPACK : sudo apt-get install libblas-dev liblapack-dev
- OpenBLAS : http://www.openblas.net/
- Matplotlib-cpp : https://github.com/lava/matplotlib-cpp
- TooN library : Edward Rosten - https://github.com/edrosten/TooN
- Armadillo : http://arma.sourceforge.net/download.html
- CSV stream : https://github.com/vincentlaucsb/csv-parser#integration
  A useful cpp file for reading the csv files

# Instructions to install the package
```
- git clone https://github.com/srisadhan/powerball.git
- cd powerball
- mkdir build
- cmake CMakeLists.txt -Bbuild
- make Makefile
- make
```

# Instructions for the Weiss force sensor
#### For using force sensor along with ethernet in linux, modify the /etc/network/interfaces to
```
auto lo
iface lo inet loopback

# Obtain DHCP address from server  
auto en01
iface en01 inet dhcp

# Connect to 192.168.1.10 network over the pcie-ethernet card for force sensor
auto enp2s0
iface enp2s0 inet static
    address 192.168.1.10
    netmask 255.255.255.0
```
replace the en01, enp2s0 to the ethernet adapters recognized on your computer using 'ifconfig'

For more information, please check: https://stackoverflow.com/questions/42922949/two-wired-connection-at-the-same-time


# How to include dynamixel libraries in your own projects
 - Install dynamixel sdk
 - place the "DynamixelSDK/c++/include" folder in your current project or include the directory "/usr/local/include/dynamixel_sdk" in your project 
 - Run the following command
```
g++ read_write.cpp -o read_write -ldxl_x64_cpp -lrt -I include/dynamixel_sdk

(or)

g++ read_write.cpp -o read_write -ldxl_x64_cpp -lrt -I <dynamixel_include_files>
```

The linking libraries are -lrt (for multi-threading) and -ldxl_x64_cpp (dynamixel libraries)

# How to improve the dynamixel communication frequency
### The communication frequency is throttled by the latency_timer parameter in **port_handler_linux.cpp** - (default value is listed at 16 above ubuntu 16.04)

**Change this value to "0" to increase the communication frequency** - currently the best I can achieve is 200 Hz for a single dynamixel

Follow the following instructions (also listed in **port_handler_linux.cpp**) in the terminal to set the latency_timer automatically:
```
$ echo ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"0\" > 99-dynamixelsdk-usb.rules
$ sudo cp ./99-dynamixelsdk-usb.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger --action=add
$ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```
If you are getting RxTx errors and observe a lot of timeout errors on the dynamixel. It is sometimes a good practice to use the recovery tool from the dynamixel wizard and hope it resolves the issue. 

# 11/14/21 Added Robotiq FT-300 sensor support 
There are 3 files included in the "powerball/include/robotiq_ft_sensor folder":
  - rq_int.h
  - rq_sensor_com.h
  - rq_sensor_state.h

and, 2 files included in the "powerball/src":
  - rq_sensor_com.cpp
  - rq_sensor_state.cpp
  