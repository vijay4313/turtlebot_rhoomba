#pragma once
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
 *  @file    Walker.h
 *  @author  Venkatraman Narayanan (vijay4313)
 *  @copyright	MIT
 *  @date    11/20/2018
 *
 *  @brief	Class declaration for Walker routine
 *
 */

#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

class Walker{
    private:
        ros::NodeHandle nh;  // ROS node handler instance
        geometry_msgs::Twist msg;  // Placeholder for velocity messages
        ros::Publisher vel;  // Placeholder for publisher
        ros::Subscriber laserScan;  // Placeholder for subscriber
        bool isCollision = false;  // Indicator for collision
        
    public:
        /**
         * @brief Default Constructor
         * for Walker class
         * @param - None
         * @return - None
         */
        Walker();
        /**
         * @brief Default Destructor
         * for Walker class
         * @param - None
         * @return - None
         */
        ~Walker();
        /**
         * @brief Routine to move the
         * turtlebot without collision
         * @param - None
         * @return - None
         */
        void step();
        /**
         * @brief Routine to check the
         * collision of the turtlebot with objects
         * @param msg - Laser Scan data from the turtlebot
         * @return - None
         */
        void collisionCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
        /**
         * @brief Routine to retrieve the
         * collision status of the turtlebot
         * @param - None
         * @return - True?false indicating the
         * 			 collision status of the turtlebot
         */
        bool collisionCheck();
};
