/**
 *@file       obs_detector.hpp
 *@author     Vasista Ayyagari
 *@copyright  MIT, Vasista Ayyagari, 2020
 *@brief      Declaration of obstacle detector class
 */
#ifndef INCLUDE_OBS_DETECTOR_HPP_
#define INCLUDE_OBS_DETECTOR_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

/**
 * @brief This class helps in detecting obstacles and maneuvering away from them
 */
class ObsDetector {
 private:
    float angVel, linVel;
    float distThreshold;
    bool nearObs;
    geometry_msgs::Twist msg;
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;

 public:
    ObsDetector(float angVel, float linVel, float distThreshold);
    ~ObsDetector();
    void computeObstacles(const sensor_msgs::LaserScan::ConstPtr& stat);
    bool isNearObs();
    void move();
};

#endif  // INCLUDE_OBS_DETECTOR_HPP_
