#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "utils/utils.h"
#include "powerball/schunk_kinematics.h"

Color::Modifier red(Color::FG_RED);
Color::Modifier green(Color::FG_GREEN);
Color::Modifier def(Color::FG_DEFAULT);

class CSVRow
{
    public:
        std::string const& operator[](std::size_t index) const
        {
            return m_data[index];
        }
        std::size_t size() const
        {
            return m_data.size();
        }
        void readNextRow(std::istream& str)
        {
            std::string         line;
            std::getline(str, line);

            std::stringstream   lineStream(line);
            std::string         cell;

            m_data.clear();
            while(std::getline(lineStream, cell, ','))
            {
                m_data.push_back(cell);
            }
            // This checks for a trailing comma with no data after it.
            if (!lineStream && cell.empty())
            {
                // If there was a trailing comma then add an empty element.
                m_data.push_back("");
            }
        }
    private:
        std::vector<std::string>    m_data;
};

std::istream& operator>>(std::istream& str, CSVRow& data)
{
    data.readNextRow(str);
    return str;
};

class CSVIterator
{   
    public:
        typedef std::input_iterator_tag     iterator_category;
        typedef CSVRow                      value_type;
        typedef std::size_t                 difference_type;
        typedef CSVRow*                     pointer;
        typedef CSVRow&                     reference;

        CSVIterator(std::istream& str)  :m_str(str.good()?&str:NULL) { ++(*this); }
        CSVIterator()                   :m_str(NULL) {}

        // Pre Increment
        CSVIterator& operator++()               {if (m_str) { if (!((*m_str) >> m_row)){m_str = NULL;}}return *this;}
        // Post increment
        CSVIterator operator++(int)             {CSVIterator    tmp(*this);++(*this);return tmp;}
        CSVRow const& operator*()   const       {return m_row;}
        CSVRow const* operator->()  const       {return &m_row;}

        bool operator==(CSVIterator const& rhs) {return ((this == &rhs) || ((this->m_str == NULL) && (rhs.m_str == NULL)));}
        bool operator!=(CSVIterator const& rhs) {return !((*this) == rhs);}
    private:
        std::istream*       m_str;
        CSVRow              m_row;
};

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cout<< red << "Error! Please provide the .csv file location" << def << std::endl;
        std::cout << green << "The path should be provided in the following format" << std::endl;
        std::cout << "pos_from_joint_angles.cpp -filepath folder/file.csv " << def << std::endl;
        return 0;
    } else {
        if (strcmp(argv[1], "-h") == 0) {
            std::cout << green << "The path should be provided in the following format" << std::endl;
            std::cout << "pos_from_joint_angles.cpp -filepath folder/file.csv " << def << std::endl;
        } else {
            std::cout << "im here " << std::endl;
            std::ifstream dataFile(argv[2] + std::string(".csv"));
            std::ifstream dataFile1(argv[2] + std::string("1.csv"));
            for (CSVIterator loop(dataFile); loop != CSVIterator(); ++loop) {
                for (int col=1; col < 7; ++col) {
                    // std::cout << (*loop)[col];
                }
                // std::cout << std::endl;
            }
            dataFile.close();

            // open the file with write access 
            std::ofstream dataFile1;
            dataFile1.open(argv[2], std::ios::app);
            dataFile1 << 1 << "\n";
            dataFile1.close();
            // for (CSVIterator loop(dataFile1); loop != CSVIterator(); ++loop) {
            //     for (int col=19; col < 22; ++col) {
            //         dataFile1 << (*loop)[col];
            //     }
            //     // std::cout << std::endl;
            // }
        }

    }


    return 0;
}