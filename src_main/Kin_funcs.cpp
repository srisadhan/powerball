#include "powerball/schunk_powerball.h"
#include "vrep/v_repClass.h"
#include <chrono>

using namespace::std;


int main(int argc, char** argv)
{


    Kin kin;

    std::vector< std::vector<double> > matrix;
    if (kin.inputFile("egg.txt",&matrix)) {
        cout << "successful" << endl;

        TooN::Matrix<Dynamic,Dynamic,double> traj(kin.nrows, kin.ncols);
        cout << "size of matrix = " << kin.nrows << "*" << kin.ncols << endl;

        int  col=0; int row=0;
        for (std::vector< std::vector<double> >::const_iterator it = matrix.begin(); it != matrix.end(); ++ it)
        {
            col = 0;
            for (std::vector<double>::const_iterator itit = it->begin(); itit != it->end(); ++ itit)
            {
                traj(row,col) = *itit;
                col++;
            }
            row++;
        }

        cout << traj << endl;

        Vector<6,float> Q = Zeros , Qn = Zeros;
        Matrix<3,4,float> T_mat = Zeros;
        Q[0] = 0.01;

        int i = 0;
        for (int n=0 ; n<3 ; n++){
            T_mat.slice<Dynamic,0,1,4>(n,0,1,4) = Data(traj(i,n*4+0),traj(i,n*4+1),traj(i,n*4+2),traj(i,n*4+3));
        }

        if (kin.IK(T_mat,Q,&Qn))
        {
            cout << Qn << endl;
        }

        kin.FK_T(Qn,&T_mat);
        cout << T_mat << endl;


        Matrix<3,6,float> Jp = Zeros;
        Matrix<3,6,float> Jo = Zeros;
        kin.JacobPos(Qn,&Jp);
        kin.JacobRot(Qn,&Jo);
        cout << Jp << endl;
        cout << Jo << endl;

        std::vector< std::vector<double> > matrix2;
        Vector<6,float> Qd = Zeros;

        //Qn = Data(0.0f,0.0f,M_PI/2,0.0f,M_PI/2,0.0f);
        int numQ = kin.HerInter(Q,Qd,Qn,0.005,5.0,&matrix2);
        TooN::Matrix<Dynamic,Dynamic,double> traj2(numQ, 6);
        col=0;
        row=0;
        for (std::vector< std::vector<double> >::const_iterator it = matrix2.begin(); it != matrix2.end(); ++ it)
        {
            col = 0;
            for (std::vector<double>::const_iterator itit = it->begin(); itit != it->end(); ++ itit)
            {
                traj2(row,col) = *itit;
                col++;
            }
            row++;
        }


        // Set sampling and timing options
        float dt = 0.005f;
        std::chrono::time_point<std::chrono::system_clock> timeLoop;
        std::chrono::duration<float> elaps_loop;

        // VREP Class
        Vector<6,float> joint_angle_demand = Zeros;
        int res = -1;
        V_rep vrep;

        res = vrep.connect();
        if (res==-1)
        {
            cout << "V-REP Connection Error" << endl;
            return 0;
        }


        for (int n=0; n<numQ ; n++)
        {
            timeLoop = std::chrono::system_clock::now();

            if (vrep.isConnected())
            {
                for (int m=0;m<6;m++)
                {
                    joint_angle_demand[m] = traj2(n,m);
                }
                vrep.setq(joint_angle_demand);
            } else
            {
                cout << "V-REP Disconnected!" << endl;
                return 0;
            }

            elaps_loop = std::chrono::system_clock::now() - timeLoop;
            if ( (dt-elaps_loop.count()) > 0 ) {
                usleep( (dt-elaps_loop.count())*1000*1000 );
            }
        }

    }

}
