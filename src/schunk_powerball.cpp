#include "powerball/schunk_powerball.h"

SchunkPowerball::SchunkPowerball() {
    memset(&_pb_state_read,0,sizeof(Powerball_State));

    // Init communication
    init_comm();

    // Configure PDOs
    configure_PDOs();

    // Communication search
    set_control_mode(MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE);
    commutation_search(NODE_ALL);

}


SchunkPowerball::~SchunkPowerball() {
    shutdown_motors();
}


void SchunkPowerball::shutdown_motors() {
    // Shutdown all motors
    for (int k=NODE_1; k<= NODE_6; k++) {
        canopen.RPDOwrite(1, k, CONTROLWORD_SHUTDOWN, 0);
        brake(k);
    }
}


void SchunkPowerball::init_comm() {
    canopen.init();
}


void SchunkPowerball::update() {
    _pb_state_read = canopen.TPDOread();
}


void SchunkPowerball::goHome() {

    float tf = 2.5f;
    float w = 2*M_PI*(0.5/tf);
    float Ts = 0.01f;
    int j = 0;
    float dq = 0.0f;

    float tol = 1.0f*M_PI/180.0f;

    TooN::Vector<6,float> q0 = Zeros;
    TooN::Vector<6,float> q  = Zeros;

    set_sdo_controlword(NODE_ALL,STATUS_OPERATION_ENABLED);
    set_control_mode(MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE);

    for (int k=NODE_1; k<=NODE_6; k++) {

        unbrake(k);

        update();
        q0 = _pb_state_read.pos;
        q  = _pb_state_read.pos;
        j = 0;

        std::cout << "initial position of motor " << (int)node2motor(k)
                  << ": " << q0[node2motor(k)-1] << std::endl;

        if ( fabs(q0[node2motor(k)-1]) > tol ) {
            std::cout << "Homing motor: " << (int)node2motor(k) << std::endl;

            while(j*Ts<=tf) {

                q[node2motor(k)-1] = q0[node2motor(k)-1] - 0.5*q0[node2motor(k)-1]*(1-cos(w*j*Ts));

                set_pos(q);
                update();
                j++;
                usleep(Ts*1000*1000);
            }

        }
        else {
            std::cout << "Motor " << (int)node2motor(k) << " is already home." << std::endl;
        }

        usleep(0.2*1000*1000);

    }
}


void SchunkPowerball::set_control_mode(int8_t mode) {

    std::cout << std::endl  << "Setting control mode for all nodes ......." << std::endl;
    for (int k=NODE_1; k<= NODE_6; k++) {
        set_node_control_mode(k, mode);
    }

}

void SchunkPowerball::configure_PDOs() {

    std::cout << std::endl  << "Configuring all nodes ......." << std::endl;
    for (int k=NODE_1; k<= NODE_6; k++) {
        configure_PDO(k);
    }

}

void SchunkPowerball::set_node_control_mode(uint8_t CANid, int8_t mode) {

    switch(mode) {

        case(MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE): {
            /////////////////////////////////////////
            // interpolated position configuration //
            /////////////////////////////////////////

            //canopen.SDOwrite_1byte(CANid,IP_TIME_UNITS,0x64);
            // note: 0x32 --> 50 * 10^((int8_t)ip_time_index) --> 50 ms
            //      this is the interpolation period. the onboard controller generates a trajectory
            //      from the actual position the target position in 0x32 time units
            //canopen.SDOwrite_1byte(CANid,IP_TIME_UNITS,0x14);
            canopen.SDOwrite_1byte(CANid,IP_TIME_UNITS,0x0A);
            // note: 0xFD is 253 unsigned or -3 signed. so the time index is -3, and the
            //      interpolation time period is defined as 10^(-3) (see dsp402.pdf page 113)
            canopen.SDOwrite_1byte(CANid,IP_TIME_INDEX,0xFD);
            // note: 0x1E --> 30 * 10^((int8_t)ip_time_index) --> 30 ms timeout.
            //      this means that a timeout occurs after 30ms since the last setpoint has been reached.
            //      in this time window the next setpoint has to be received.
            canopen.SDOwrite_1byte(CANid,SYNC_TIMEOUT_FACTOR,0x1E);
            canopen.SDOwrite_1byte(CANid,MODES_OF_OPERATION,MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE);
            //canopen.SDOwrite_1byte(CANid,MODES_OF_OPERATION,MODES_OF_OPERATION_PROFILE_POSITION_MODE);

            printf(" Interpolated position mode set on motor%d\n",node2motor(CANid));

            /////////////////////////////////////////////
            // end interpolated position configuration //
            /////////////////////////////////////////////

        } break; // interpolated position

        case(MODES_OF_OPERATION_VELOCITY_MODE): {
            canopen.SDOwrite_1byte(CANid,MODES_OF_OPERATION,MODES_OF_OPERATION_VELOCITY_MODE);
            printf(" Velocity mode set on motor%d\n",node2motor(CANid));
        } break;

    } // switch

}


void SchunkPowerball::configure_PDO(uint8_t CANid) {

    canopen.sendNMT(CANid, NMT_ENTER_PRE_OPERATIONAL);

    // TODO: contact dario or amir to complete the code, controlmode is not implemented yet

    int32_t pos_actual = canopen.SDOread(CANid, ACTUALPOSITION);
    _pb_state_read.pos[node2motor(CANid)-1] = pos_actual;

    int32_t pos_limit_min = canopen.SDOread(CANid, POSLIMITMIN);
    int32_t pos_limit_max = canopen.SDOread(CANid, POSLIMITMAX);
    int32_t max_vel = canopen.SDOread(CANid, MAXVEL);
    printf("\nNode Id %d information: \n Actual Position = %d \n Position Limit Min = %d \n Position Limit Max = %d \n Velocity Limit = %d \n" ,
            CANid, pos_actual, pos_limit_min, pos_limit_max, max_vel);

    // Each mapping object is constructed as:
    // object index     object sub-index       size (bits)
    //  xxxx                xx                  xx

    // RPDO #1 control word - target position
    canopen.SDOwrite_8byte(CANid,RPDO1_COMM,0x80000000+RPDO1_msg+CANid);
    canopen.SDOwrite_1byte(CANid,RPDO1_MAPPING,0x00);
    canopen.SDOwrite_8byte(CANid,RPDO1_MAP_OBJ_1,0x60400010);
    canopen.SDOwrite_8byte(CANid,RPDO1_MAP_OBJ_2,0x60C10120);
    canopen.SDOwrite_1byte(CANid,RPDO1_MAPPING,0x02);
    canopen.SDOwrite_1byte(CANid,RPDO1_TRANS,0x01);
    canopen.SDOwrite_8byte(CANid,RPDO1_COMM,RPDO1_msg+CANid);
    std::cout << " rpdo #1: control word - target position" << std::endl;

    // RPDO #2 control word - target velocity
    canopen.SDOwrite_8byte(CANid,RPDO2_COMM,0x80000000+RPDO2_msg+CANid);
    canopen.SDOwrite_1byte(CANid,RPDO2_MAPPING,0x00); // deactive
    canopen.SDOwrite_8byte(CANid,RPDO2_MAP_OBJ_1,0x60400010);
    canopen.SDOwrite_8byte(CANid,RPDO2_MAP_OBJ_2,0x60420010);
    canopen.SDOwrite_1byte(CANid,RPDO2_MAPPING,0x02); // number of objects
    canopen.SDOwrite_1byte(CANid,RPDO2_TRANS,0x01); // active
    canopen.SDOwrite_8byte(CANid,RPDO2_COMM,RPDO2_msg+CANid);
    std::cout << " rpdo #2: control word - target velocity" << std::endl;

    /* it is commneted because it is by defualt set
    // TPDO #1 status word - read torque - read position
    canopen.SDOwrite_8byte(CANid,TPDO1_COMM,0x80000000+TPDO1_msg+CANid);
    canopen.SDOwrite_1byte(CANid,TPDO1_MAPPING,0x00);
    canopen.SDOwrite_8byte(CANid,TPDO1_MAP_OBJ_1,0x60400010);
    canopen.SDOwrite_8byte(CANid,TPDO1_MAP_OBJ_2,0x60C10110);
    canopen.SDOwrite_8byte(CANid,TPDO1_MAP_OBJ_3,0x60C10120);
    canopen.SDOwrite_1byte(CANid,TPDO1_MAPPING,0x03);
    canopen.SDOwrite_1byte(CANid,TPDO1_TRANS,0x01);
    canopen.SDOwrite_8byte(CANid,TPDO1_COMM,TPDO1_msg+CANid);
    */

    std::cout << " tpdo #1: status word - actual currents - actual position" << std::endl;


    // TPDO #2 read velocity
    canopen.SDOwrite_8byte(CANid,TPDO2_COMM,0x80000000+TPDO2_msg+CANid);
    canopen.SDOwrite_1byte(CANid,TPDO2_MAPPING,0x00);
    canopen.SDOwrite_8byte(CANid,TPDO2_MAP_OBJ_1,0x606C0020);
    canopen.SDOwrite_1byte(CANid,TPDO2_MAPPING,0x01);
    canopen.SDOwrite_1byte(CANid,TPDO2_TRANS,0x01);
    canopen.SDOwrite_8byte(CANid,TPDO2_COMM,TPDO2_msg+CANid);
    std::cout << " tpdo #2: actual velocity" << std::endl;


    canopen.sendNMT(CANid, NMT_START_REMOTE_NODE);



}

void SchunkPowerball::print_operation_mode() {

    int32_t mode;

    for (int k=NODE_1; k<=NODE_6; k++) {
        mode = canopen.SDOread(k,MODES_OF_OPERATION_DISPLAY);
        printf("Mode of operation '%02x' set on motor%d\n",mode,node2motor(k));
    }


}

void SchunkPowerball::commutation_search(uint8_t CANid) {

    update();
    TooN::Vector<6,float> q = get_pos();

    float timeout = 10.0f;
    float Ts = 0.05;

    if (CANid <= NODE_6 && CANid >= NODE_1) {
        std::cout << std::endl  << "Starting commutation search for motor # " << (int)node2motor(CANid) << std::endl;

        set_sdo_controlword(CANid,STATUS_OPERATION_ENABLED);

        int32_t ext_status = canopen.SDOread(CANid,EXTENDED_STATUS);
        int8_t comm_search_completed_flag = ext_status && 0x01;
        int8_t pseudo_abs_pos_verified = ext_status && 0x02;

        int counter = 0;

        if (!comm_search_completed_flag) {

             while ( (!comm_search_completed_flag) && (counter++*Ts < timeout) ) {
                 ext_status = canopen.SDOread(CANid,EXTENDED_STATUS);
                 comm_search_completed_flag = ext_status & 0x01;

                 set_pos(CANid, q[node2motor(CANid)-1]+1.0*M_PI/180.0f*sin(counter*Ts*2*M_PI/timeout));
                 update();
                 usleep(Ts*1000*1000);

             }

             if (comm_search_completed_flag) {
                 std::cout << "Commutation search completed!" << std::endl;
             }
             else {
                 std::cout << "Commutation search error: timeout" << std::endl;
             }

        }
        else {
            std::cout << "Commutation search already done" << std::endl;
        }

    }

    else if (CANid == NODE_ALL) {

        set_sdo_controlword(NODE_ALL,STATUS_OPERATION_ENABLED);
        std::cout << std::endl << "Starting commutation search for all motors ......." << std::endl;

        for (int k=NODE_1; k<=NODE_6; k++) {

            std::cout << "Starting commutation search for motor: " << (int)node2motor(k) << std::endl;

            int32_t ext_status = canopen.SDOread(k,EXTENDED_STATUS);
            int8_t comm_search_completed_flag = ext_status && 0x01;
            int8_t pseudo_abs_pos_verified = ext_status && 0x02;

            int counter = 0;

            if (!comm_search_completed_flag) {

                 while ( (!comm_search_completed_flag) && (counter++*Ts < timeout) ) {
                     ext_status = canopen.SDOread(k,EXTENDED_STATUS);
                     comm_search_completed_flag = ext_status & 0x01;

                     set_pos(k, q[node2motor(k)-1]);
                     update();
                     usleep(Ts*1000*1000);

                 }

                 if (comm_search_completed_flag) {
                     std::cout << "Commutation search completed" << std::endl;
                 }
                 else {
                     std::cout << "Commutation search error: timeout" << std::endl;
                 }

            }
            else {
                std::cout << "Commutation search already done" << std::endl;
            }

        } // for
    }
    else {
        std::cout << "Commutation search error. Node id was out of range. It was: " << CANid << std::endl;
    }

}

// added by Sri 
uint32_t SchunkPowerball::get_control_mode()
{
    uint32_t mode; 

    for (int k=NODE_1; k<= NODE_6; k++) {
        mode = canopen.SDOread(k,MODES_OF_OPERATION_DISPLAY);
    }
    return mode;
}



