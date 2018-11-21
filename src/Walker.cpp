/**
 * MIT License

 * Copyright (c) 2018 Venkatraman Narayanan
 *
 * Permission is hereby granted, free of charge,
 * to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software
 * without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to
 * whom the Software is furnished to do so, subject to the
 * following conditions:
 *
 * The above copyright notice and this permission notice shall
 *  be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

 /**
 *
 *  @file    Walker.cpp
 *  @author  Venkatraman Narayanan (vijay4313)
 *  @copyright	MIT
 *  @date    11/20/2018
 *
 *  @brief	A ROS based class to move the turtlebot
 *  without collision
 *
 *  @section DESCRIPTION
 *	Class object to move the turtlebot
 *	without collision
 *
 */
#include "../include/Walker.h"

/**
 * @brief Default Constructor
 * for Walker class
 * @param - None
 * @return - None
 */
Walker::Walker() {
    // Publisher to post the velocity messages
    vel = nh.advertise <geometry_msgs::Twist>
     ("/mobile_base/commands/velocity", 1000);
    // Subscriber to get the laser scan data
    laserScan = nh.subscribe("/scan", 50, &Walker::collisionCallback, this);
    msg.linear.x = 0.0;  // Linear X-axis velocity of turtlebot
    msg.linear.y = 0.0;  // Linear Y-axis velocity of turtlebot
    msg.linear.z = 0.0;  // Linear Z-axis velocity of turtlebot
    msg.angular.x = 0.0;  // Angular X-axis velocity of turtlebot
    msg.angular.y = 0.0;  // Angular Y-axis velocity of turtlebot
    msg.angular.z = 0.0;  // Angular Z-axis velocity of turtlebot

    vel.publish(msg);
}

/**
 * @brief Default Destructor
 * for Walker class
 * @param - None
 * @return - None
 */
Walker::~Walker() {
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;

    vel.publish(msg);
}
/**
 * @brief Routine to move the
 * turtlebot without collision
 * @param - None
 * @return - None
 */
void Walker::step() {
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        if (isCollision) {
            msg.linear.x = 0.0;
            msg.angular.z = 1.0;
        } else {
            msg.linear.x = 0.2;
            msg.angular.z = 0.0;
        }
        vel.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

/**
 * @brief Routine to check the
 * collision of the turtlebot with objects
 * @param msg - Laser Scan data from the turtlebot
 * @return - None
 */
void Walker::collisionCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        for (int i = 0; i < msg->ranges.size(); i++) {
        if (msg->ranges[i] < 0.8) {
        isCollision = true;  // true indicates presence of obstacle
        return;
        }
    }
    isCollision = false;
}

/**
 * @brief Routine to retrieve the
 * collision status of the turtlebot
 * @param - None
 * @return - True?false indicating the
 * 			 collision status of the turtlebot
 */
bool Walker::collisionCheck() {
    return isCollision;
}
