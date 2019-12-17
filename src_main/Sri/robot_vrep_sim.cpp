#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <ostream> // included for color output to the terminal

// boost headers
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>

// // schunk powerball headers
#include "powerball/schunk_powerball.h"
#include "vrep/v_repClass.h"
#include "powerball/schunk_kinematics.h"
#include "utils/utils.h"

// // Toon headers
#include <TooN/TooN.h>
#include <TooN/LU.h>
#include <TooN/SVD.h>

// plotting and linear algebra libraries
#include "matplotlibcpp.h"
#include <armadillo>


using namespace::std;
using namespace::TooN;
using boost::asio::ip::tcp;

Color::Modifier red(Color::FG_RED);
Color::Modifier green(Color::FG_GREEN);
Color::Modifier def(Color::FG_DEFAULT);


// global vars
const double dt = 0.005; // sampling time
Vector<6,float> joints_vrep = Zeros;
bool stopFlag = false;

void stop(bool* flag){
    char in;
    cin.get(in);
    *flag = true;
}

// Vrep function
int vrep_draw(bool* flag){
    // connect to vrep
    int res = -1;
    if (! *flag){
      V_rep vrep;
      res = vrep.connect();
      if (res==-1)
      {
          cout << red << "V-REP Connection Error!" << def << endl;
          return 0;
      }

      while(!*flag)
      {
          vrep.setq(joints_vrep);
          usleep(40*1000);
      }
    }
}

void load_matrix(std::istream* is,
                 std::vector< std::vector<double> >* matrix,
                 const std::string& delim = " \t")
{

    string      line;
    string      strnum;

    // clear first
    matrix->clear();

    // parse line by line
    while (getline(*is, line))
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
}

int main (int argc, char** argv){

  // timestamp vars
  char buffer[10];
  struct timeval tv;
  time_t curtime;
  std::chrono::time_point<std::chrono::system_clock> timeLoop;
  std::chrono::duration<float> elaps_loop;

  // insert file
  std::ifstream is("Straight_line.txt");
  if (!is)
  {
      cout<< "error reading file!" << endl;
      return 0;
  }

  // load the matrix
  std::vector< std::vector<double> > matrix;
  load_matrix(&is, &matrix);

  // Check the input matrix size and set TooN matrix size
  int  ncols, nrows=0;
  for (std::vector< std::vector<double> >::const_iterator it = matrix.begin(); it != matrix.end(); ++ it)
  {
      nrows++;
      ncols = 0;
      for (std::vector<double>::const_iterator itit = it->begin(); itit != it->end(); ++ itit)
      {
          ncols++;
      }
  }
  TooN::Matrix<Dynamic,Dynamic,double> traj(nrows, ncols);
  cout << "size of matrix = " << nrows << "*" << ncols << endl;
  if (ncols!=6)
  {
      cout << "inconsistent input matrix size" << endl;
      return 0;
  }

  // Put the matrix into TooN matrix
  nrows = 0;  ncols = 0;
  for (std::vector< std::vector<double> >::const_iterator it = matrix.begin(); it != matrix.end(); ++ it)
  {
      for (std::vector<double>::const_iterator itit = it->begin(); itit != it->end(); ++ itit)
      {
          traj(nrows,ncols) = *itit;
          if (ncols==5) // for the new installation of the gripper and PI offset in joint 6
          {
              traj(nrows,ncols) = traj(nrows,ncols) - M_PI;
          }
          ncols++;
      }
      nrows++;
      ncols = 0;
  }

  boost::thread stop_thread(stop, &stopFlag);
  // connect to V-rep
  boost::thread vrep_thread(vrep_draw,&stopFlag);

  int i = 0;
  while((i<nrows) && (!stopFlag))
  {
    timeLoop = std::chrono::system_clock::now();

    for(int j = 0; j < 6; j++){
        joints_vrep[j] = traj(i,j);
    }



    i++;
    elaps_loop = std::chrono::system_clock::now() - timeLoop;
    if ( (dt-elaps_loop.count()) > 0 ) {
        usleep( (dt-elaps_loop.count())*1000*1000 );
    } else
    {
        cout << red << "communication time out!" << def << endl;
    }
  }

  stop_thread.interrupt();
  vrep_thread.interrupt();
  return 0;
}
