#include "../include/Walker.h"

Walker::Walker() {
    vel = nh.advertise <geometry_msgs::Twist>
     ("/mobile_base/commands/velocity", 1000);
    laserScan = nh.subscribe("/scan", 50, &Walker::collisionCallback,this);
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;

    vel.publish(msg);
}

Walker::~Walker() {
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;

    vel.publish(msg);
}

void Walker::step() {
    ros::Rate loop_rate(10);
    while(ros::ok()) {
        if(isCollision) {
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


void Walker::collisionCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        for (int i = 0; i < msg->ranges.size(); i++) {
        if (msg->ranges[i] < 0.8) {
        isCollision = true;  // true indicates presence of obstacle
        return;
        }
    }
    isCollision = false;
}

bool Walker::collisionCheck() {
    return isCollision;
}