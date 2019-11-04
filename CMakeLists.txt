# set required version
cmake_minimum_required(VERSION 2.8)

# set project name
project(Powerball)

# set include drectories - this is the directory, from which cmake was started, i.e. the top level source directory
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include/vrep)
include_directories(${CMAKE_SOURCE_DIR}/include/ximu)
include_directories(${CMAKE_SOURCE_DIR}/include/utils)
include_directories(${CMAKE_SOURCE_DIR}/include/dynamixel)
include_directories(${CMAKE_SOURCE_DIR}/remoteApi)
include_directories(${CMAKE_SOURCE_DIR}/include/plot)
include_directories(${CMAKE_SOURCE_DIR}/include/sigpack)
include_directories(${CMAKE_SOURCE_DIR}/include/gnuplot-5.2.6)

# set output paths
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# set canopen and powerball source files
set(extApi_LIB_SRC
    remoteApi/extApi.c
)

set(extApiPlatform_LIB_SRC
    remoteApi/extApiPlatform.c
)

set(CANOPEN_LIB_SRC
    src/schunk_canopen.cpp
    src/schunk_canopen_io.cpp
)

set(KINEMATIC_LIB_SRC
    src/schunk_kinematics.cpp
)

set(POWERBALL_LIB_SRC
    src/schunk_powerball.cpp
    src/schunk_powerball_state_fcns.cpp
    src/schunk_powerball_accessors.cpp
)

set(DYNAMIXEL_LIB_SRC
    src/utils.cpp
    src/USB2Dynamixel.cpp
)

# add custom flags to c++ compiler
set(CMAKE_CXX_FLAGS "-std=c++0x")
add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)

# find boost libraries
find_package(Boost COMPONENTS system signals thread REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)
find_package(PythonLibs 2.7)
include_directories(PRIVATE ${PYTHON_INCLUDE_DIRS})

#find sdl library
#find_package(SDL REQUIRED)
#find_Package(SDL_mixer REQUIRED)

# find ntcan library
link_directories(/usr/local/esd/lib32)
link_directories(/usr/local/esd/lib64)
link_directories(${CMAKE_SOURCE_DIR}/imu)

# compile schunk library
add_library(schunk_canopen ${CANOPEN_LIB_SRC})
target_link_libraries(schunk_canopen ntcan)

add_library(schunk_kinematics ${KINEMATIC_LIB_SRC})

add_library(schunk_powerball ${POWERBALL_LIB_SRC})
target_link_libraries(schunk_powerball schunk_canopen schunk_kinematics)

add_library(dynamixel ${DYNAMIXEL_LIB_SRC})

add_library(extApi ${extApi_LIB_SRC})

add_library(extApiPlatform ${extApiPlatform_LIB_SRC})


# compile executables
add_executable(homing src_main/homing.cpp)
target_link_libraries(homing schunk_powerball)

#add_executable(manualControl src_main/manualControl.cpp)
#target_link_libraries(manualControl schunk_powerball pthread ${Boost_LIBRARIES})

#add_executable(mag_identify src_main/mag_identify.cpp)
#target_link_libraries(mag_identify schunk_powerball  schunk_kinematics extApi extApiPlatform pthread phidget21 ${Boost_LIBRARIES})

add_executable(admittance_pb src_main/admittance_pb.cpp)
target_link_libraries(admittance_pb schunk_powerball schunk_kinematics extApi extApiPlatform pthread ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

add_executable(Myo_envelop_SJ src_main/Sri/Myo_envelop_SJ.cpp)
target_link_libraries(Myo_envelop_SJ ${Boost_LIBRARIES} myolinux -lboost_iostreams ${PYTHON_LIBRARIES}) # gnuplot-iostream requires lboost_iostreams

add_executable(admittance_3d src_main/Sri/admittance_3d.cpp)
target_link_libraries(admittance_3d schunk_powerball schunk_kinematics extApi extApiPlatform pthread myolinux dynamixel ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

add_executable(admittance_3d_velMode src_main/Sri/admittance_3d_velMode.cpp)
target_link_libraries(admittance_3d_velMode schunk_powerball schunk_kinematics extApi extApiPlatform pthread myolinux dynamixel ${Boost_LIBRARIES} ${LAPACK_LIBRARIES} ${PYTHON_LIBRARIES})

add_executable(admittance_pb_velMode src_main/admittance_pb_velMode.cpp)
target_link_libraries(admittance_pb_velMode schunk_powerball schunk_kinematics extApi extApiPlatform pthread ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

add_executable(admittance_var src_main/admittance_var.cpp)
target_link_libraries(admittance_var schunk_powerball schunk_kinematics extApi extApiPlatform pthread ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

add_executable(datarecord src_main/Team6/datarecord.cpp)
target_link_libraries(datarecord schunk_powerball schunk_kinematics extApi extApiPlatform pthread myolinux dynamixel ${Boost_LIBRARIES} ${LAPACK_LIBRARIES} ${PYTHON_LIBRARIES})

add_executable(admittance_team1 src_main/Sri/admittance_team1.cpp)
target_link_libraries(admittance_team1 schunk_powerball schunk_kinematics extApi extApiPlatform pthread myolinux dynamixel ${Boost_LIBRARIES} ${LAPACK_LIBRARIES} ${PYTHON_LIBRARIES})

#add_executable(file_input_gripper_pb_vrep src_main/file_input_gripper_pb_vrep.cpp)
#target_link_libraries(file_input_gripper_pb_vrep schunk_powerball extApi extApiPlatform pthread dynamixel phidget21 ${Boost_LIBRARIES})

add_executable(rock_stacking src_main/rock_stacking.cpp)
target_link_libraries(rock_stacking schunk_powerball schunk_kinematics extApi extApiPlatform pthread dynamixel phidget21 ${Boost_LIBRARIES})

add_executable(collision_reaction src_main/collision_reaction.cpp)
target_link_libraries(collision_reaction schunk_powerball schunk_kinematics extApi extApiPlatform pthread dynamixel phidget21 ${Boost_LIBRARIES})

add_executable(hitting_hammer src_main/hitting_hammer.cpp)
target_link_libraries(hitting_hammer schunk_powerball schunk_kinematics extApi extApiPlatform pthread dynamixel phidget21 ${Boost_LIBRARIES})

add_executable(egg_collision src_main/egg_collision.cpp)
target_link_libraries(egg_collision schunk_powerball schunk_kinematics extApi extApiPlatform pthread dynamixel phidget21 ${Boost_LIBRARIES})

add_executable(gripper src_main/Sri/gripper.cpp)
target_link_libraries(gripper schunk_powerball schunk_kinematics extApi extApiPlatform pthread myolinux dynamixel phidget21 ${Boost_LIBRARIES} ${LAPACK_LIBRARIES} ${PYTHON_LIBRARIES})

add_executable(robot_vrep_sim src_main/Sri/robot_vrep_sim.cpp)
target_link_libraries(robot_vrep_sim schunk_powerball schunk_kinematics extApi extApiPlatform pthread ${Boost_LIBRARIES} ${LAPACK_LIBRARIES} ${PYTHON_LIBRARIES})

add_executable(robot_traj src_main/Sri/robot_traj.cpp)
target_link_libraries(robot_traj schunk_powerball schunk_kinematics extApi extApiPlatform pthread myolinux dynamixel phidget21 ${Boost_LIBRARIES} ${LAPACK_LIBRARIES} ${PYTHON_LIBRARIES})


#add_executable(file_input_pb_vrep src_main/file_input_pb_vrep.cpp)
#target_link_libraries(file_input_pb_vrep schunk_powerball extApi extApiPlatform pthread)

#add_executable(vrep_server src_main/vrep_server.cpp)
#target_link_libraries(vrep_server extApi extApiPlatform pthread ${Boost_LIBRARIES})

#add_executable(IP_test src_main/IP_test.cpp)
#target_link_libraries(IP_test schunk_powerball)

#add_executable(vel_test src_main/vel_test.cpp)
#target_link_libraries(vel_test schunk_powerball)

#add_executable(vrep_client src_main/vrep_client.cpp)
#target_link_libraries(vrep_client ${Boost_LIBRARIES})

#add_executable(UDP_server src_main/UDP_server.cpp)
#target_link_libraries(UDP_server ${Boost_LIBRARIES})

#add_executable(powerball_server src_main/powerball_server.cpp)
#target_link_libraries(powerball_server schunk_powerball extApi extApiPlatform pthread ${Boost_LIBRARIES})

#add_executable(IK src_main/IK.cpp)
#target_link_libraries(IK schunk_powerball extApi extApiPlatform pthread ximu ${Boost_LIBRARIES})

#add_executable(trust1_vrep src_main/trust1_vrep.cpp)
#target_link_libraries(trust1_vrep schunk_powerball extApi extApiPlatform pthread ximu SDL_mixer SDL ${Boost_LIBRARIES})

#add_executable(cooperation_imu_follower_vrep src_main/cooperation_imu_follower_vrep.cpp)
#target_link_libraries(cooperation_imu_follower_vrep schunk_powerball extApi extApiPlatform pthread ximu SDL_mixer SDL ${Boost_LIBRARIES})

######### Experiments ##########
#add_executable(trust1_pb src_main/trust1_pb.cpp)
#target_link_libraries(trust1_pb schunk_powerball extApi extApiPlatform pthread ximu SDL_mixer SDL ${Boost_LIBRARIES})

#add_executable(cooperation_imu_follower_pb src_main/cooperation_imu_follower_pb.cpp)
#target_link_libraries(cooperation_imu_follower_pb schunk_powerball ximu SDL_mixer SDL ${Boost_LIBRARIES})

#add_executable(HH_cooperation src_main/HH_cooperation.cpp)
#target_link_libraries(HH_cooperation ximu SDL_mixer SDL ${Boost_LIBRARIES})

#add_executable(dynamixel_phidget src_main/dynamixel_phidget.cpp)
#target_link_libraries(dynamixel_phidget dynamixel phidget21 ${Boost_LIBRARIES})

#add_executable(hammer src_main/hammer.cpp)
#target_link_libraries(hammer schunk_powerball extApi extApiPlatform pthread dynamixel phidget21 ${Boost_LIBRARIES})

######## Moveis ############
#add_executable(egg src_main/egg.cpp)
#target_link_libraries(egg schunk_powerball extApi extApiPlatform pthread dynamixel phidget21 ${Boost_LIBRARIES})

#add_executable(egg2 src_main/egg2.cpp)
#target_link_libraries(egg2 schunk_powerball extApi extApiPlatform pthread dynamixel phidget21 ${Boost_LIBRARIES})


# install library
#install(TARGETS mbed DESTINATION ${MY_INSTALL_PATH}/lib)
#install(FILES ${CMAKE_SOURCE_DIR}/include/mbed/mbed.h ${CMAKE_SOURCE_DIR}/include/mbed/mbed_commands.h
#	 	DESTINATION ${MY_INSTALL_PATH}/include/mbed)
