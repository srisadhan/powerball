#ifndef V_REPCLASS_H
#define V_REPCLASS_H

#include "TooN/TooN.h"
extern "C" {
    #include "extApi.h"
/*	#include "extApiCustom.h" if you wanna use custom remote API functions! */
}

class V_rep {
    public:

        V_rep()
        {

        }

        ~V_rep()
        {
            if (simxGetConnectionId(clientID)!=-1)
                simxFinish(clientID);
        }

        int connect()
        {
            int portNb=19999;
            simxInt res;

            clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,1000,5);

            if (clientID!=-1)
            {
                std::cout << "Connected to remote API server on V-REP" << std::endl;
                res = simxAddStatusbarMessage(clientID,(simxChar*)"Client Connected.",simx_opmode_oneshot);

               char JointName[] = "arm_n_joint";
               for (int n=0; n<6; n++)
                {
                   simxFloat position;
                   JointName[4] = (char)(((int)'0')+n+1);
                   res = simxGetObjectHandle(clientID,(simxChar*)JointName,&joint_handle[n],simx_opmode_oneshot_wait);
                   simxGetJointPosition(clientID,joint_handle[n],&position,simx_opmode_streaming);
                }
               simxGetObjectHandle(clientID,"Sphere",&Sphere_hnd,simx_opmode_oneshot_wait);
            }

            return clientID;
        }


        void disconnect()
        {
            if (simxGetConnectionId(clientID)!=-1)
                simxFinish(clientID);
        }

        bool isConnected()
        {
            if (simxGetConnectionId(clientID)!=-1)
            {
                return true;
            } else
            {
                return false;
            }
        }

/*
        void setq(float joint_angle[])
        {
            float sign[6] = {-1, -1, -1, 1, -1, 1};
            for (int n=0; n<6; n++)
            {
                simxSetJointTargetPosition(clientID,joint_handle[n],joint_angle[n]*sign[n],simx_opmode_oneshot);
            }
        }
*/

        void setq(Vector<6,float> joint_angle)  // TooN version
        {
            float sign[6] = {-1, -1, -1, 1, -1, -1};
            for (int n=0; n<6; n++)
            {
                simxSetJointTargetPosition(clientID,joint_handle[n],joint_angle[n]*sign[n],simx_opmode_oneshot);
            }
        }

        void setSphere(simxFloat* pos)  // TooN version
        {
            simxSetObjectPosition(clientID,Sphere_hnd,-1,pos,simx_opmode_oneshot);

        }


        void getq(Vector<6,float>* joint_angle)
        {
            simxFloat position;
            float sign[6] = {-1, -1, -1, 1, -1, -1};
            Vector<6,float> angles;
            for (int n=0; n<6; n++)
            {
                simxGetJointPosition(clientID,joint_handle[n],&position, simx_opmode_buffer);
                angles[n] = position*sign[n];
            }
            *joint_angle = angles;
        }


    private:

        int clientID;
        TooN::Vector<6,simxInt> joint_handle = Zeros;
        simxInt Sphere_hnd;

};

#endif // V_REPCLASS_H
