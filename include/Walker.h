#pragma once

#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
//#include "../include/Collision.h"

class Walker{
    private:
        ros::NodeHandle nh;
        geometry_msgs::Twist msg;
        ros::Publisher vel;
        ros::Subscriber laserScan;
        bool isCollision = false;
        //Collision collObj;
    public:
        Walker();
        ~Walker();
        void step();
        void collisionCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
        bool collisionCheck();
};