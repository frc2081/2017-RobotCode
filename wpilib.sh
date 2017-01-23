#!/usr/bin/env bash

# clean out old installs
rm -rf ~/wpilib
rm -rf ~/wpilib_process

cd ~
mkdir wpilib_process
cd wpilib_process

# WPILib 2017.1.1
# for more versions go to: http://first.wpi.edu/FRC/roborio/release/eclipse/plugins/
wget http://first.wpi.edu/FRC/roborio/release/eclipse/plugins/edu.wpi.first.wpilib.plugins.cpp_2017.1.1.jar
mv edu.wpi.first.wpilib.plugins.cpp_2017.1.1.jar plugins.jar
unzip plugins.jar
mv resources/cpp.zip cpp.zip
mkdir ~/wpilib
mkdir ~/wpilib/cpp
mkdir ~/wpilib/cpp/current
cd ~/wpilib/cpp/current
unzip ~/wpilib_process/cpp.zip

rm -rf ~/wpilib_process
