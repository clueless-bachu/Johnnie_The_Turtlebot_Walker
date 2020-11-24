/**
 *@file       obs_detector.cpp
 *@author     Vasista Ayyagari
 *@copyright  MIT, Vasista Ayyagari, 2020
 *@brief      this file creates the obstacle avoidance node 
 */
#include <iostream>
#include "obs_detector.hpp"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "johnnie_turtlebot");
    ObsDetector robot(0.2, 0.1, 0.4);
    robot.move();
}
