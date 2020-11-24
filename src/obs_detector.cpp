/**
 *@file       obs_detector.cpp
 *@author     Vasista Ayyagari
 *@copyright  MIT, Vasista Ayyagari, 2020
 *@brief      Obstacle detector class definitions are decalred here
 */
#include "obs_detector.hpp"

/**
 * @brief class constructor
 * @param angVel: angular velocity
 * @param linVel: linear velocity
 * @param distThreshold: The distance at which objects are detected
 * @return None 
 */
ObsDetector::ObsDetector(float angVel, float linVel, float distThreshold) {
    this->angVel = angVel;
    this->linVel = linVel;
    this->distThreshold = distThreshold;
    this->nearObs = false;

    this->pub = nh.advertise <geometry_msgs::Twist>("/cmd_vel", 1000);
    this->sub = nh.subscribe<sensor_msgs::LaserScan>
    ("/scan", 1000, &ObsDetector::computeObstacles, this);
}

/**
 * @brief class destructor
 * @param None
 * @return None 
 */
ObsDetector::~ObsDetector() {
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    pub.publish(msg);
}

/**
 * @brief Subscriber callback to detect obstacles
 * @param msg: the laser scan data
 * @return None 
 */
void ObsDetector::computeObstacles
(const sensor_msgs::LaserScan::ConstPtr& msg) {
    for (auto i = 0; i < 70; ++i) {
        if (msg->ranges[i] < this->distThreshold) {
            this->nearObs = true;
            ROS_INFO_STREAM("Found Obstacle");
            return;
        }
    }

    for (auto i = 290; i < 360; ++i) {
        if (msg->ranges[i] < this->distThreshold) {
            this->nearObs = true;
            ROS_INFO_STREAM("Found Obstacle");
            return;
        }
    }

    this->nearObs = false;
    return;
}

/**
 * @brief This method tells if an object is near or not
 * @param None
 * @return bool 
 */
bool ObsDetector::isNearObs() {
    return this->nearObs;
}


/**
 * @brief This method gives motion commands under different scenarios 
 * @param None
 * @return None 
 */
void ObsDetector::move() {
    while (ros::ok()) {
        if (!this->isNearObs()) {
            msg.linear.x = this->linVel;
            msg.angular.z = 0;
        } else {
            msg.linear.x = 0;
            msg.angular.z = this->angVel;
        }

        pub.publish(msg);
        ros::spinOnce();
    }
}
