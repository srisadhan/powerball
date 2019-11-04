#ifndef SCHUNK_KINEMATICS_H
#define SCHUNK_KINEMATICS_H

#include <TooN/TooN.h>
#include <fstream>
#include <istream>
#include <string>
#include <sstream>
#include <vector>
#include <iostream>
#include <boost/array.hpp>
#include <math.h>

using namespace::TooN;

class Kin {
public:

    Kin();
    ~Kin();

    int nrows;
    int ncols;
    int inputFile(std::string fileName , std::vector< std::vector<double> > *matrix);
    // output is success

    void FK_pos(Vector<6,float> Q, Vector<3,float> *pos);

    void FK_T(Vector<6,float> Q,  Matrix<3,4,float> *T);

    void FK_R(Vector<6,float> Q,  Matrix<3,3,float> *T);

    int IK(Matrix<3,4,float> T_mat, Vector<6,float> Qp, Vector<6,float> *Qn);

    void JacobPos(Vector<6,float> Q,  Matrix<3,6,float> *Jp);

    void JacobRot(Vector<6,float> Q,  Matrix<3,6,float> *Jo);

    void Jacob(Vector<6,float> Q,  Matrix<6,6,float> *J);

    int HerInter(Vector<6,float> Q1, Vector<6,float> Qd1, Vector<6,float> Q2, float dt, float T, std::vector< std::vector<double> > *matrix);
    // output is the number of rows



private:

    const float a1 = .205;
    const float a2 = .350;
    const float a3 = .305;
    const float d6 = .085+.155; // flang surface + gripper height


};

#endif // SCHUNK_KINEMATICS_H
