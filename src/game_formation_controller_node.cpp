#include <ros/ros.h>
#include "game_formation_controller.h"

int main(int argc, char ** argv){
    ros::init(argc, argv, "game_formation_controller_node");

    ExpFormationController exp_formation_controller;
    ros::spin();
    return 0;
}