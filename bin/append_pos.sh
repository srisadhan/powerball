#!/bin/bash

path='/home/srisadha/powerball/data/IEEE_haptics'
trials='HighFine HighGross LowGross LowFine'
filename='PB'
executable='sudo ./pos_from_joint_angles -filepath'
for file in $path/*; 
do
    for trial in $trials; 
    do
        # run the executable to read PB.csv file and create a PB1.csv file 
        sudo ./pos_from_joint_angles -filepath ${file}/$trial/$filename

        # remove the old PB.csv file
        sudo rm -f ${file}/$trial/'PB.csv'

        # rename the PB1.csv to PB.csv
        mv -f ${file}/$trial/'PB1.csv' ${file}/$trial/'PB.csv'
    done
done