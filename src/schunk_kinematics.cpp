#include "powerball/schunk_kinematics.h"

using namespace::std;

Kin::Kin() {}


Kin::~Kin() {}

int Kin::inputFile(std::string fileName, std::vector< std::vector<double> > *matrix)
{
    const std::string& delim = " \t";
    string      line;
    string      strnum;

    std::ifstream is(fileName);
    if (!is)
    {
        cout<< "error reading file!" << endl;
        return 0;
    }

    // clear first
    matrix->clear();

    // parse line by line
    while (getline(is, line))
    {
        matrix->push_back(vector<double>());

        for (string::const_iterator i = line.begin(); i != line.end(); ++ i)
        {
            // If i is not a delim, then append it to strnum
            if (delim.find(*i) == string::npos)
            {
                strnum += *i;
                if (i + 1 != line.end()) // If it's the last char, do not continue
                    continue;
            }

            // if strnum is still empty, it means the previous char is also a
            // delim (several delims appear together). Ignore this char.
            if (strnum.empty())
                continue;

            // If we reach here, we got a number. Convert it to double.
            double       number;

            istringstream(strnum) >> number;
            matrix->back().push_back(number);

            strnum.clear();
        }
    }

    ncols =0; nrows=0;
    for (std::vector< std::vector<double> >::const_iterator it = matrix->begin(); it != matrix->end(); ++ it)
    {
        nrows++;
        ncols = 0;
        for (std::vector<double>::const_iterator itit = it->begin(); itit != it->end(); ++ itit)
        {
            ncols++;
        }
    }


    return 1;

}


void Kin::FK_pos(Vector<6,float> Q, Vector<3,float> *pos)
{

    float q1 = Q[0];
    float q2 = Q[1];
    float q3 = Q[2];
    float q4 = Q[3];
    float q5 = Q[4];
    float q6 = Q[5];

    Matrix<3,4,float> T_mat = Data( cos(q6)*(cos(q1)*cos(q3)*sin(q2)*sin(q5) - cos(q1)*cos(q2)*sin(q3)*sin(q5) - cos(q5)*sin(q1)*sin(q4) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) + cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)) - sin(q6)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*sin(q4) + cos(q1)*sin(q2)*sin(q3)*sin(q4)), - cos(q6)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*sin(q4) + cos(q1)*sin(q2)*sin(q3)*sin(q4)) - sin(q6)*(cos(q1)*cos(q3)*sin(q2)*sin(q5) - cos(q1)*cos(q2)*sin(q3)*sin(q5) - cos(q5)*sin(q1)*sin(q4) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) + cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)), cos(q1)*cos(q2)*cos(q5)*sin(q3) - sin(q1)*sin(q4)*sin(q5) - cos(q1)*cos(q3)*cos(q5)*sin(q2) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5), a3*cos(q1)*cos(q2)*sin(q3) - a2*cos(q1)*sin(q2) - a3*cos(q1)*cos(q3)*sin(q2) - d6*sin(q1)*sin(q4)*sin(q5) + d6*cos(q1)*cos(q2)*cos(q5)*sin(q3) - d6*cos(q1)*cos(q3)*cos(q5)*sin(q2) + d6*cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5) + d6*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5),
                                    cos(q6)*(cos(q1)*cos(q5)*sin(q4) - cos(q2)*sin(q1)*sin(q3)*sin(q5) + cos(q3)*sin(q1)*sin(q2)*sin(q5) + cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) + cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)) - sin(q6)*(cos(q2)*cos(q3)*sin(q1)*sin(q4) - cos(q1)*cos(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4)), - cos(q6)*(cos(q2)*cos(q3)*sin(q1)*sin(q4) - cos(q1)*cos(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4)) - sin(q6)*(cos(q1)*cos(q5)*sin(q4) - cos(q2)*sin(q1)*sin(q3)*sin(q5) + cos(q3)*sin(q1)*sin(q2)*sin(q5) + cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) + cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)), cos(q1)*sin(q4)*sin(q5) + cos(q2)*cos(q5)*sin(q1)*sin(q3) - cos(q3)*cos(q5)*sin(q1)*sin(q2) + cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) + cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5), a3*cos(q2)*sin(q1)*sin(q3) - a2*sin(q1)*sin(q2) - a3*cos(q3)*sin(q1)*sin(q2) + d6*cos(q1)*sin(q4)*sin(q5) + d6*cos(q2)*cos(q5)*sin(q1)*sin(q3) - d6*cos(q3)*cos(q5)*sin(q1)*sin(q2) + d6*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) + d6*cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5),
                                    - cos(q6)*(sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q3)*sin(q5) + cos(q2)*cos(q4)*cos(q5)*sin(q3) - cos(q3)*cos(q4)*cos(q5)*sin(q2)) - sin(q2 - q3)*sin(q4)*sin(q6),                                                                                                                              sin(q6)*(sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q3)*sin(q5) + cos(q2)*cos(q4)*cos(q5)*sin(q3) - cos(q3)*cos(q4)*cos(q5)*sin(q2)) - sin(q2 - q3)*cos(q6)*sin(q4),                                                           cos(q2)*cos(q3)*cos(q5) + cos(q5)*sin(q2)*sin(q3) - cos(q2)*cos(q4)*sin(q3)*sin(q5) + cos(q3)*cos(q4)*sin(q2)*sin(q5),                                                                                 a1 + a2*cos(q2) + a3*cos(q2)*cos(q3) + a3*sin(q2)*sin(q3) + d6*cos(q2)*cos(q3)*cos(q5) + d6*cos(q5)*sin(q2)*sin(q3) - d6*cos(q2)*cos(q4)*sin(q3)*sin(q5) + d6*cos(q3)*cos(q4)*sin(q2)*sin(q5) );


    *pos = T_mat.T()[3];

    //cout << T_mat << endl;
}

void Kin::FK_T(Vector<6,float> Q,  Matrix<3,4,float> *T)
{

    float q1 = Q[0];
    float q2 = Q[1];
    float q3 = Q[2];
    float q4 = Q[3];
    float q5 = Q[4];
    float q6 = Q[5];

    Matrix<3,4,float> T_mat = Data( cos(q6)*(cos(q1)*cos(q3)*sin(q2)*sin(q5) - cos(q1)*cos(q2)*sin(q3)*sin(q5) - cos(q5)*sin(q1)*sin(q4) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) + cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)) - sin(q6)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*sin(q4) + cos(q1)*sin(q2)*sin(q3)*sin(q4)), - cos(q6)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*sin(q4) + cos(q1)*sin(q2)*sin(q3)*sin(q4)) - sin(q6)*(cos(q1)*cos(q3)*sin(q2)*sin(q5) - cos(q1)*cos(q2)*sin(q3)*sin(q5) - cos(q5)*sin(q1)*sin(q4) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) + cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)), cos(q1)*cos(q2)*cos(q5)*sin(q3) - sin(q1)*sin(q4)*sin(q5) - cos(q1)*cos(q3)*cos(q5)*sin(q2) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5), a3*cos(q1)*cos(q2)*sin(q3) - a2*cos(q1)*sin(q2) - a3*cos(q1)*cos(q3)*sin(q2) - d6*sin(q1)*sin(q4)*sin(q5) + d6*cos(q1)*cos(q2)*cos(q5)*sin(q3) - d6*cos(q1)*cos(q3)*cos(q5)*sin(q2) + d6*cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5) + d6*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5),
                                    cos(q6)*(cos(q1)*cos(q5)*sin(q4) - cos(q2)*sin(q1)*sin(q3)*sin(q5) + cos(q3)*sin(q1)*sin(q2)*sin(q5) + cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) + cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)) - sin(q6)*(cos(q2)*cos(q3)*sin(q1)*sin(q4) - cos(q1)*cos(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4)), - cos(q6)*(cos(q2)*cos(q3)*sin(q1)*sin(q4) - cos(q1)*cos(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4)) - sin(q6)*(cos(q1)*cos(q5)*sin(q4) - cos(q2)*sin(q1)*sin(q3)*sin(q5) + cos(q3)*sin(q1)*sin(q2)*sin(q5) + cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) + cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)), cos(q1)*sin(q4)*sin(q5) + cos(q2)*cos(q5)*sin(q1)*sin(q3) - cos(q3)*cos(q5)*sin(q1)*sin(q2) + cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) + cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5), a3*cos(q2)*sin(q1)*sin(q3) - a2*sin(q1)*sin(q2) - a3*cos(q3)*sin(q1)*sin(q2) + d6*cos(q1)*sin(q4)*sin(q5) + d6*cos(q2)*cos(q5)*sin(q1)*sin(q3) - d6*cos(q3)*cos(q5)*sin(q1)*sin(q2) + d6*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) + d6*cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5),
                                    - cos(q6)*(sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q3)*sin(q5) + cos(q2)*cos(q4)*cos(q5)*sin(q3) - cos(q3)*cos(q4)*cos(q5)*sin(q2)) - sin(q2 - q3)*sin(q4)*sin(q6),                                                                                                                              sin(q6)*(sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q3)*sin(q5) + cos(q2)*cos(q4)*cos(q5)*sin(q3) - cos(q3)*cos(q4)*cos(q5)*sin(q2)) - sin(q2 - q3)*cos(q6)*sin(q4),                                                           cos(q2)*cos(q3)*cos(q5) + cos(q5)*sin(q2)*sin(q3) - cos(q2)*cos(q4)*sin(q3)*sin(q5) + cos(q3)*cos(q4)*sin(q2)*sin(q5),                                                                                 a1 + a2*cos(q2) + a3*cos(q2)*cos(q3) + a3*sin(q2)*sin(q3) + d6*cos(q2)*cos(q3)*cos(q5) + d6*cos(q5)*sin(q2)*sin(q3) - d6*cos(q2)*cos(q4)*sin(q3)*sin(q5) + d6*cos(q3)*cos(q4)*sin(q2)*sin(q5) );


    *T = T_mat;

}

void Kin::FK_R(Vector<6,float> Q,  Matrix<3,3,float> *T)
{

    float q1 = Q[0];
    float q2 = Q[1];
    float q3 = Q[2];
    float q4 = Q[3];
    float q5 = Q[4];
    float q6 = Q[5];

    Matrix<3,3,float> T_mat = Data( cos(q6)*(cos(q1)*cos(q3)*sin(q2)*sin(q5) - cos(q1)*cos(q2)*sin(q3)*sin(q5) - cos(q5)*sin(q1)*sin(q4) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) + cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)) - sin(q6)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*sin(q4) + cos(q1)*sin(q2)*sin(q3)*sin(q4)), - cos(q6)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*sin(q4) + cos(q1)*sin(q2)*sin(q3)*sin(q4)) - sin(q6)*(cos(q1)*cos(q3)*sin(q2)*sin(q5) - cos(q1)*cos(q2)*sin(q3)*sin(q5) - cos(q5)*sin(q1)*sin(q4) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) + cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)), cos(q1)*cos(q2)*cos(q5)*sin(q3) - sin(q1)*sin(q4)*sin(q5) - cos(q1)*cos(q3)*cos(q5)*sin(q2) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5),
                                    cos(q6)*(cos(q1)*cos(q5)*sin(q4) - cos(q2)*sin(q1)*sin(q3)*sin(q5) + cos(q3)*sin(q1)*sin(q2)*sin(q5) + cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) + cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)) - sin(q6)*(cos(q2)*cos(q3)*sin(q1)*sin(q4) - cos(q1)*cos(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4)), - cos(q6)*(cos(q2)*cos(q3)*sin(q1)*sin(q4) - cos(q1)*cos(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4)) - sin(q6)*(cos(q1)*cos(q5)*sin(q4) - cos(q2)*sin(q1)*sin(q3)*sin(q5) + cos(q3)*sin(q1)*sin(q2)*sin(q5) + cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) + cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)), cos(q1)*sin(q4)*sin(q5) + cos(q2)*cos(q5)*sin(q1)*sin(q3) - cos(q3)*cos(q5)*sin(q1)*sin(q2) + cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) + cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5),
                                    - cos(q6)*(sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q3)*sin(q5) + cos(q2)*cos(q4)*cos(q5)*sin(q3) - cos(q3)*cos(q4)*cos(q5)*sin(q2)) - sin(q2 - q3)*sin(q4)*sin(q6),                                                                                                                              sin(q6)*(sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q3)*sin(q5) + cos(q2)*cos(q4)*cos(q5)*sin(q3) - cos(q3)*cos(q4)*cos(q5)*sin(q2)) - sin(q2 - q3)*cos(q6)*sin(q4),                                                           cos(q2)*cos(q3)*cos(q5) + cos(q5)*sin(q2)*sin(q3) - cos(q2)*cos(q4)*sin(q3)*sin(q5) + cos(q3)*cos(q4)*sin(q2)*sin(q5));


    *T = T_mat;

}

int Kin::IK(Matrix<3,4,float> T_mat, Vector<6,float> Qp, Vector<6,float> *Qn)
{
    // inputs = T_mat:tranformation matrix  Qp:joint vector current Qn:pointer of result


    //Matrix<3,4,double> T_mat=Data(1,0,0,0.2,0,1,0,0.3,0,0,1,0.4);
    //Vector<6,double> Qp=makeVector(0.2,0.3,0.5,0.3,0,-0.5);
    /*
    for (int i=0 ; i<3 ; i++){
        for (int j=1 ; j<3 ; j++){
            if (T_mat(i,j)<1e-5){
                T_mat(i,j) = 0;
            }
        }
    }
*/
    Vector<3> n_e = T_mat.T()[0];
    Vector<3> s_e = T_mat.T()[1];
    Vector<3> a_e = T_mat.T()[2];
    Vector<3> p_e = T_mat.T()[3];

    Matrix<8,6,float> Q=Zeros; // 8 different solutions

    // Solution of Anthropomorphic Arm
    Vector<3> p_w = p_e - d6*a_e;
    float p_wx = p_w[0];
    float p_wy = p_w[1];
    float p_wz = p_w[2]-a1;

    float eps = 1e-4;
    if (abs(p_wx)<=eps)     p_wx=0;
    if (abs(p_wy)<=eps)     p_wy=0;

    float q1_i, q1_ii;
    if ((p_wx==0) && (p_wy==0))
    {
        cout << "\033[0m" << "There are infinity solutions" << "\033[0m" << endl;
        return 0;
    } else
    {
        q1_i = atan2(p_wy,p_wx);
        if (p_wy>=0)
        {
            q1_ii = q1_i - M_PI;
        } else
        {
            q1_ii = q1_i + M_PI;
        }
    }

    if (!((sqrt(pow(p_wx,2)+pow(p_wy,2)+pow(p_wz,2)) >= abs(a2-a3)) && (sqrt(pow(p_wx,2)+pow(p_wy,2)+pow(p_wz,2))<=a2+a3)))
    {
        cout << "\033[1m\033[31m" << "The wrist point is outside of the workspace" << "\033[0m" << endl;
        return 0;
    }

    float c3 = (pow(p_wx,2)+pow(p_wy,2)+pow(p_wz,2)-pow(a2,2)-pow(a3,2))/(2*a2*a3);
    float s3 = sqrt(1-pow(c3,2));
    float q3_i = atan2(s3,c3);
    float q3_ii = -q3_i;

    float s2_a = (a2+a3*c3)*p_wz;
    float s2_b = a3*s3*sqrt(pow(p_wx,2)+pow(p_wy,2));
    float c2_a = (a2+a3*c3)*sqrt(pow(p_wx,2)+pow(p_wy,2));
    float c2_b = a3*s3*p_wz;
    float q2_i =   atan2( (s2_a-s2_b),(c2_a+c2_b) );
    float q2_ii =  atan2( (s2_a+s2_b),(-c2_a+c2_b) );
    float q2_iii = atan2( (s2_a+s2_b),(c2_a-c2_b) );
    float q2_iv =  atan2( (s2_a-s2_b),(-c2_a-c2_b) );

    Q.slice<0,0,1,3>() = Data(q1_i, q2_i, q3_i);
    Q.slice<1,0,1,3>() = Data(q1_i, q2_iii, q3_ii);
    Q.slice<2,0,1,3>() = Data(q1_ii, q2_ii, q3_i);
    Q.slice<3,0,1,3>() = Data(q1_ii, q2_iv, q3_ii);
    Q.slice<4,0,4,3>() = Q.slice<0,0,4,3>();

    // SCHUNK 0 position effect using siciliano 2.95 to 2.97
    for (int k=0; k<8; k++)
    {
        if (Q(k,1)<-M_PI/2)
        {
            Q(k,1) =  Q(k,1) + 3*M_PI/2;
        } else
        {
            Q(k,1) =  Q(k,1) - M_PI/2;
        }
    }
    Q.slice<0,2,8,1>() = -Q.slice<0,2,8,1>();

    // Solution of spherical wrist
    Matrix<3,3,float> RR , R_6_0 = Zeros;
    RR[0] = n_e;
    RR[1] = s_e;
    RR[2] = a_e;
    R_6_0 = RR.T();
    //cout<< GREEN << R_6_0 << "\033[0m" << endl << endl;

    float q4_i,q5_i,q6_i,q4_ii,q5_ii,q6_ii;
    for (int k=0 ; k<4 ; k++)
    {
        float q1 = Q(k,0);
        float q2 = Q(k,1);
        float q3 = Q(k,2);
        Matrix<3,3,float> R_3_0 = Data( cos(q2 - q3)*cos(q1), -sin(q1), -sin(q2 - q3)*cos(q1),
                                        cos(q2 - q3)*sin(q1),  cos(q1), -sin(q2 - q3)*sin(q1),
                                        sin(q2 - q3),        0,          cos(q2 - q3));

        Matrix<3,3,float> R_6_3 = R_3_0.T()*R_6_0;
        float a_x = R_6_3(0,2);
        float a_y = R_6_3(1,2);
        float a_z = R_6_3(2,2);
        float s_z = R_6_3(2,1);
        float n_z = R_6_3(2,0);

        if ((abs(s_z)<eps) && (abs(n_z)<eps))
        {
            cout << "\033[1m\033[31m" << "Wrist is singular because T-mat =\n " << T_mat <<  "\033[0m" << endl;
            q6_i = Qp[5];
            q6_ii = Qp[5];
        } else
        {
            q6_i = atan2(s_z , -n_z);
            q6_ii = atan2(-s_z , n_z);
        }

        q4_i = atan2(a_y,a_x);
        q5_i = atan2(sqrt(pow(a_x,2)+pow(a_y,2)) , a_z);

        Q.slice(0,3,1,3)[k] = Data(q4_i , q5_i , q6_i);
        q4_ii = atan2(-a_y,-a_x);
        q5_ii = atan2(-sqrt(pow(a_x,2)+pow(a_y,2)) , a_z);

        //cout << "atan2(" << -s_z << "," << n_z << ")=" << q6_ii << endl;
        Q.slice(4,3,1,3)[k] = Data(q4_ii , q5_ii , q6_ii);
    }
    //cout << "\033[31m" << Q << "\033[0m" << endl;
    // check for acceptable & min norm answer
    float normP = 100.0f;
    int index = 0;
    for (int i=0 ; i<8 ; i++)
    {
        bool accept = true;
        for (int j=0 ; j<6 ; j++)
        {
            if ( (2*M_PI-abs(Q(i,j))) < 0.01 )
            {
                Q(i,j) = 0.0f;
                //cout << GREEN << "changed" << "\033[0m" << endl;
            }
            if (abs(Q(i,j))>=160*M_PI/180)
            {
                accept = false;
            }

        }
        if (accept)
        {
            float normN = norm(Q[i]-Qp);
            if (normN<normP)
            {
                index = i;
                normP = normN;
            }
        }
        //cout << "\033[31m" << Q[i] << "\033[0m" << endl;
    }

    //cout << "All Q = " << endl << Q << endl;
    *Qn = Q[index];
    //cout<< "Q_final\n" << "\033[1m\033[31m" << Q[index] << "\033[0m" << endl;
    return 1;

    //cout<< "Tmat \n" << p_e.as_col() << endl;
}


void Kin::JacobPos(Vector<6,float> Q,  Matrix<3,6,float> *Jp) {

    float q1 = Q[0];
    float q2 = Q[1];
    float q3 = Q[2];
    float q4 = Q[3];
    float q5 = Q[4];
    float q6 = Q[5];

    Matrix<3,6,float> jacob = Data(  (sin(q1)*(61*sin(q2 - q3) + 70*sin(q2)))/200, -(cos(q1)*(61*cos(q2 - q3) + 70*cos(q2)))/200, (61*cos(q2 - q3)*cos(q1))/200, 0, 0, 0,
                                     -(cos(q1)*(61*sin(q2 - q3) + 70*sin(q2)))/200, -(sin(q1)*(61*cos(q2 - q3) + 70*cos(q2)))/200, (61*cos(q2 - q3)*sin(q1))/200, 0, 0, 0,
                                     0,      - (61*sin(q2 - q3))/200 - (7*sin(q2))/20,         (61*sin(q2 - q3))/200, 0, 0, 0   );

    *Jp = jacob;

}

void Kin::JacobRot(Vector<6,float> Q,  Matrix<3,6,float> *Jo) {

    float q1 = Q[0];
    float q2 = Q[1];
    float q3 = Q[2];
    float q4 = Q[3];
    float q5 = Q[4];
    float q6 = Q[5];

    Matrix<3,6,float> jacob = Data( 0,  sin(q1), -sin(q1), -sin(q2 - q3)*cos(q1), - cos(q4)*sin(q1) - cos(q1)*cos(q2)*cos(q3)*sin(q4) - cos(q1)*sin(q2)*sin(q3)*sin(q4), cos(q1)*cos(q2)*cos(q5)*sin(q3) - sin(q1)*sin(q4)*sin(q5) - cos(q1)*cos(q3)*cos(q5)*sin(q2) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5),
                                    0, -cos(q1),  cos(q1), -sin(q2 - q3)*sin(q1),   cos(q1)*cos(q4) - cos(q2)*cos(q3)*sin(q1)*sin(q4) - sin(q1)*sin(q2)*sin(q3)*sin(q4), cos(q1)*sin(q4)*sin(q5) + cos(q2)*cos(q5)*sin(q1)*sin(q3) - cos(q3)*cos(q5)*sin(q1)*sin(q2) + cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) + cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5),
                                    1,        0,        0,          cos(q2 - q3),                                                                 -sin(q2 - q3)*sin(q4),                                                           cos(q2)*cos(q3)*cos(q5) + cos(q5)*sin(q2)*sin(q3) - cos(q2)*cos(q4)*sin(q3)*sin(q5) + cos(q3)*cos(q4)*sin(q2)*sin(q5)   );

    *Jo = jacob;
}

void Kin::Jacob(Vector<6,float> Q,  Matrix<6,6,float> *J) {
    float q1 = Q[0];
    float q2 = Q[1];
    float q3 = Q[2];
    float q4 = Q[3];
    float q5 = Q[4];
    float q6 = Q[5];

    Matrix<6,6,float> jacob = Data(  (sin(q1)*(61*sin(q2 - q3) + 70*sin(q2)))/200, -(cos(q1)*(61*cos(q2 - q3) + 70*cos(q2)))/200, (61*cos(q2 - q3)*cos(q1))/200, 0, 0, 0,
                                     -(cos(q1)*(61*sin(q2 - q3) + 70*sin(q2)))/200, -(sin(q1)*(61*cos(q2 - q3) + 70*cos(q2)))/200, (61*cos(q2 - q3)*sin(q1))/200, 0, 0, 0,
                                     0,      - (61*sin(q2 - q3))/200 - (7*sin(q2))/20,         (61*sin(q2 - q3))/200, 0, 0, 0,
                                     0,  sin(q1), -sin(q1), -sin(q2 - q3)*cos(q1), - cos(q4)*sin(q1) - cos(q1)*cos(q2)*cos(q3)*sin(q4) - cos(q1)*sin(q2)*sin(q3)*sin(q4), cos(q1)*cos(q2)*cos(q5)*sin(q3) - sin(q1)*sin(q4)*sin(q5) - cos(q1)*cos(q3)*cos(q5)*sin(q2) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5),
                                                                        0, -cos(q1),  cos(q1), -sin(q2 - q3)*sin(q1),   cos(q1)*cos(q4) - cos(q2)*cos(q3)*sin(q1)*sin(q4) - sin(q1)*sin(q2)*sin(q3)*sin(q4), cos(q1)*sin(q4)*sin(q5) + cos(q2)*cos(q5)*sin(q1)*sin(q3) - cos(q3)*cos(q5)*sin(q1)*sin(q2) + cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) + cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5),
                                                                        1,        0,        0,          cos(q2 - q3),                                                                 -sin(q2 - q3)*sin(q4),                                                           cos(q2)*cos(q3)*cos(q5) + cos(q5)*sin(q2)*sin(q3) - cos(q2)*cos(q4)*sin(q3)*sin(q5) + cos(q3)*cos(q4)*sin(q2)*sin(q5)   );

    *J = jacob;



}

// Trajectory planning between two points
int Kin::HerInter(Vector<6,float> Q1, Vector<6,float> Qd1, Vector<6,float> Q2, float dt, float T_travel, std::vector< std::vector<double> > *matrix)
{
    // Hermit Interpolation
    Matrix<4,4,float> M_hermit = Data(2,-2, 1, 1, -3, 3, -2, -1, 0, 0, 1, 0, 1, 0, 0, 0);
    Matrix<4,6,float> G = Zeros; // Hermit Coeff
    G[0] = Q1;
    G[1] = Q2;
    G[2] = Qd1;

    float dt_speed = dt/T_travel;    // Speed factor : dt/(scale 1 sec to this)
    float t = 0;
    int i=0;
    // clear first
    matrix->clear();

    while (t<=1){
        Vector<4,float> T = Data(pow(t,3), pow(t,2), t , 1);
        Vector<6,float> Qc = T*(M_hermit*G); // Interpolation
        t += dt_speed;
        i++;

        matrix->push_back(vector<double>());
        for(int j=0;j<6;j++)
        {
            matrix->back().push_back(Qc[j]);
        }
    }

    return i;

}




