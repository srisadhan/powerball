
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "utils/utils.h"
#include "powerball/schunk_kinematics.h"
#include "utils/csvstream.h"
#include <map>
#include <utility>
#include <TooN/TooN.h>

Color::Modifier red(Color::FG_RED);
Color::Modifier green(Color::FG_GREEN);
Color::Modifier def(Color::FG_DEFAULT);


int main(int argc, char **argv)
{   
    bool header = true;
    TooN::Vector<6, float> Q = Zeros;
    TooN::Vector<3, float> pos = Zeros;

    Kin kin;

    if (argc < 2) {
        std::cout<< red << "Error! Please provide the .csv file location" << def << std::endl;
        std::cout << green << "The path should be provided in the following format" << std::endl;
        std::cout << "pos_from_joint_angles.cpp -filepath folder/file " << def << std::endl;
        return 0;
    } else if (strcmp(argv[1], "-h") == 0) {
            std::cout << green << "The path should be provided in the following format" << std::endl;
            std::cout << "pos_from_joint_angles.cpp -filepath folder/file " << def << std::endl;
            return 0;
    } 

    // open file - read
    csvstream dataFile(argv[2] + std::string(".csv")); 
    // open new file - write
    std::ofstream dataFile1;
    dataFile1.open(argv[2] + std::string("1.csv"));

    std::vector<std::pair<std::string, std::string>> row; 
    std::string::size_type sz;

    // Read file
    while (dataFile >> row) {
        Q = Zeros;
        pos = Zeros; 
        // To write the header
        if (header){
            for (unsigned int i=0; i < row.size(); ++i) {
                const std::string &column_name = row[i].first;
                dataFile1 << column_name << "," ;
            }
            dataFile1 << "X, Y, Z" << std::endl;
            header = false;
        }
        for (unsigned int i=0; i < row.size(); ++i) {
            std::string datum = row[i].second;
            dataFile1 << datum << "," ;
            if (i < 7 & i > 0){
                Q[i-1] = std::stof(datum); 
            }
        }
        kin.FK_pos(Q, &pos);
        // std::cout << Q << std::endl;
        dataFile1 << pos[0] << "," << pos[1] << "," << pos[2] << "\n" ;     
    }
    dataFile1.close();
    return 0;
}