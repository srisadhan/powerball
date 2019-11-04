# C++ library for schunk powerball lwa4p (linux)
### Main contributor: Amir Memar


# Required libraries
### Hardware
- ntcan (CAN-USB) : https://esd.eu/en (better option is to install it from the ESD driver CD)
  - Socketcan : http://www.pengutronix.de/software/libsocketcan/download/
  - iproute2  : https://github.com/shemminger/iproute2
    - iproute2 might require flex and bison (sudo apt-get install flex; sudo apt-get install bison). "ip link show : should show the connected can device"
- Phidget21 : https://www.phidgets.com/downloads/phidget21/libraries/linux/libphidget/

### Software
- Boost library : sudo apt-get install libboost-all-dev
- BLAS and LAPACK : sudo apt-get install libblas-dev liblapack-dev
- OpenBLAS : http://www.openblas.net/
- Matplotlib-cpp : https://github.com/lava/matplotlib-cpp
- TooN library : Edward Rosten - https://github.com/edrosten/TooN
- Armadillo : http://arma.sourceforge.net/download.html


# Instructions to install the package
- git clone https://github.com/srisadhan/powerball.git
- cd powerball
- mkdir build
- cmake CMakeLists.txt -Bbuild
- make Makefile
- make

# Instructions for the Weiss force sensor
#### For using force sensor along with ethernet in linux modify the /etc/network/interfaces to

- auto lo
- iface lo inet loopback

#### Obtain DHCP address from server  
- auto en01
- iface en01 inet dhcp

#### Connect to 192.168.1.10 network over the pcie-ethernet card for force sensor

  - auto enp2s0
  - iface enp2s0 inet static
      - address 192.168.1.10
      - netmask 255.255.255.0

replace the en01, enp2s0 to the ethernet adapters recognized on your computer using 'ifconfig'

For more information, please check: https://stackoverflow.com/questions/42922949/two-wired-connection-at-the-same-time
