

#include <ros/ros.h>
#include "../include/Walker.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "turtlebot_rhoomba");
    Walker walk;
    walk.step();
    return 0;
}