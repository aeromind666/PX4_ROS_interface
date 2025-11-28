#include "game_uav.h"
#include <tf/transform_datatypes.h>
#include <cmath>

ExpUAV::ExpUAV(const ros::NodeHandle & nh, const ros::NodeHandle & pnh, const double vel_restrict_value)\
    :nh_(nh),
    pnh_(pnh){
        vel_restrict_value_ = vel_restrict_value; 
        satellites_number_ = 0;
        common_origin_gps_WGS84_ = Eigen::Vector2d(31.821073, 117.129311);  //init value is the position of USTC gaoxin campus north door from google map
        

        state_sub_ = nh_.subscribe("mavros/state",  10, 
                            &ExpUAV::stateCallback, this);

        pose_sub_ = nh_.subscribe("mavros/local_position/pose", 10,
                            &ExpUAV::poseCallback, this);

        vel_sub_ = nh_.subscribe("mavros/local_position/velocity_local", 10,
                            &ExpUAV::velCallback, this);

        global_position_sub_ = nh_.subscribe("mavros/global_position/global", 10,
                            &ExpUAV::globalPositionCallback, this);
        
        home_position_sub_ = nh_.subscribe("mavros/home_position/home", 10,
                            &ExpUAV::homePositionCallback, this);

        satellites_sub_ = nh_.subscribe("mavros/global_position/raw/satellites", 10,
                            &ExpUAV::satellitesCallback, this);
        
        local_cmd_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
                            "mavros/setpoint_velocity/cmd_vel", 10);

        local_position_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
                            "mavros/setpoint_position/local", 10);

        cartesian_position_yaw_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
                            "self/cartesian_position_yaw", 10);

        yaw_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
                            "yaw", 10);


        set_gp_origin_pub_ = nh_.advertise<geographic_msgs::GeoPointStamped>(
                            "mavros/global_position/set_gp_origin", 10);

        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
                            "mavros/cmd/arming");   

        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
                            "mavros/set_mode");
}

ExpUAV::~ExpUAV(){}

void ExpUAV::stateCallback(const mavros_msgs::StateConstPtr & msg){
    state_ = *msg;
}

void ExpUAV::pubLocalCmdVel(Eigen::Vector2d & u, const double & des_height){
    u = u.cwiseMin(Eigen::Vector2d(vel_restrict_value_, vel_restrict_value_));
    u = u.cwiseMax(Eigen::Vector2d(-vel_restrict_value_, -vel_restrict_value_));

    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.twist.linear.x = u[0];
    msg.twist.linear.y = u[1];
    double uz = - (current_height_ - des_height);
    uz = uz < 0.5 ? (uz  > -0.5 ? uz : -0.5) : 0.5;
    msg.twist.linear.z = uz;
    
    local_cmd_vel_pub_.publish(msg);
}

void ExpUAV::pubLocalCmdVelYawrate(Eigen::Vector2d u, const double des_height, const double yaw_rate){
    u = u.cwiseMin(Eigen::Vector2d(vel_restrict_value_, vel_restrict_value_));
    u = u.cwiseMax(Eigen::Vector2d(-vel_restrict_value_, -vel_restrict_value_));

    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.twist.linear.x = u[0];
    msg.twist.linear.y = u[1];
    
    double uz = - (current_height_ - des_height);
    uz = uz < 0.5 ? (uz  > -0.5 ? uz : -0.5) : 0.5;
    msg.twist.linear.z = uz;

    double max_yaw_rate = 0.6;
    double cmd_yaw_rate = yaw_rate < max_yaw_rate ? (yaw_rate  > -max_yaw_rate ? yaw_rate : -max_yaw_rate) : max_yaw_rate;
    msg.twist.angular.z = cmd_yaw_rate;
    
    local_cmd_vel_pub_.publish(msg);
}

double ExpUAV::wrap_yaw_angle(double yaw_angle, const double low, const double high){
    const double range = high - low;

    if (yaw_angle < low) {
        yaw_angle += range * (floor((low - yaw_angle) / range) + 1);
    }

    return low + fmod((yaw_angle - low), range);
}

void ExpUAV::pubLocalCmdVelYawangle(Eigen::Vector2d u, const double des_height, const double des_yaw_angle){
    u = u.cwiseMin(Eigen::Vector2d(vel_restrict_value_, vel_restrict_value_));
    u = u.cwiseMax(Eigen::Vector2d(-vel_restrict_value_, -vel_restrict_value_));

    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.twist.linear.x = u[0];
    msg.twist.linear.y = u[1];
    
    double uz = - (current_height_ - des_height);
    uz = uz < 0.5 ? (uz  > -0.5 ? uz : -0.5) : 0.5;
    msg.twist.linear.z = uz;

    const double yaw_ctrl_gain = 1.1;
    const double max_yaw_rate = 0.6;
    // current_yaw_ unit is rad; form -180--180, east is 0deg, north is 90deg, south is -90deg 
    double yaw_angle_error_pi = wrap_yaw_angle(des_yaw_angle - current_yaw_, -M_PI, M_PI);
    double yaw_rate = yaw_ctrl_gain * yaw_angle_error_pi;
    double cmd_yaw_rate = yaw_rate < max_yaw_rate ? (yaw_rate  > -max_yaw_rate ? yaw_rate : -max_yaw_rate) : max_yaw_rate;
    msg.twist.angular.z = cmd_yaw_rate;

    local_cmd_vel_pub_.publish(msg);
}

void ExpUAV::pubLocalPosition(Eigen::Vector2d position, double target_height){
    if(position.maxCoeff() > 100 || position.minCoeff() < -100){
        ROS_ERROR("The position command is unavaliable");
        exit(4);
    }

    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = position.x();
    msg.pose.position.y = position.y();
    msg.pose.position.z = target_height;
    local_position_pub_.publish(msg);
}

void ExpUAV::pubSetGPOrigin(const geographic_msgs::GeoPointStamped & msg){
    set_gp_origin_pub_.publish(msg);
}

void ExpUAV::poseCallback(const geometry_msgs::PoseStampedConstPtr & msg){
    pos_[0] = msg->pose.position.x;
    pos_[1] = msg->pose.position.y;
    current_height_ = msg->pose.position.z;
    //todo need to test
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
    // yaw_ =  (quadrotor_common::quaternionToEulerAnglesZYX(
    //     quadrotor_common::geometryToEigen(msg->pose.orientation))).z();
    
    geometry_msgs::Vector3Stamped yaw_msg;
    yaw_msg.header.stamp = ros::Time::now();
    yaw_msg.vector.x = current_yaw_;
    yaw_pub_.publish(yaw_msg);
}

void ExpUAV::velCallback(const geometry_msgs::TwistStampedConstPtr & msg)
{
    vel_[0] = msg->twist.linear.x;
    vel_[1] = msg->twist.linear.y;
}

void ExpUAV::globalPositionCallback(const sensor_msgs::NavSatFixConstPtr & msg){
    global_position_ = *msg;
}

void ExpUAV::homePositionCallback(const mavros_msgs::HomePositionConstPtr & msg){
    mavros_msgs::HomePosition hp = *msg;
    home_height_ = hp.position.z;
}

void ExpUAV::satellitesCallback(const std_msgs::UInt32ConstPtr & msg){
    satellites_number_ = msg->data; 
}

void ExpUAV::pubYawCmdVel(const double des_yaw){
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    double gain = 1.1;
    double yaw_rate = gain * (des_yaw - current_yaw_);
    yaw_rate = yaw_rate < 0.5 ? (yaw_rate  > -0.5 ? yaw_rate : -0.5) : 0.5;
    msg.twist.angular.z = yaw_rate;
    local_cmd_vel_pub_.publish(msg);
}

void ExpUAV::pubCartesianPoseYawSelf(){
    Eigen::Vector2d now_gps = Eigen::Vector2d(global_position_.latitude, global_position_.longitude);
    Eigen::Vector2d ENUposition = calculateENUfromWGS84(common_origin_gps_WGS84_, now_gps);

    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = ENUposition.x();
    msg.pose.position.y = ENUposition.y();
    msg.pose.position.z = 0;
    msg.pose.orientation.x = vel_.x();
    msg.pose.orientation.y = vel_.y();
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = current_yaw_;
    cartesian_position_yaw_pub_.publish(msg);

    self_cartesian_pos_ = ENUposition;
}

void ExpUAV::setCommonOriginGPSWGS84(double lat, double log){
    common_origin_gps_WGS84_[0] = lat;
    common_origin_gps_WGS84_[1] = log;
    ROS_INFO("set common_origin_gps_WGS84_:{%f, %f}", lat, log);
}

double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

#define C_EARTH (double)6378137.0
Eigen::Vector2d ExpUAV::calculateENUfromWGS84(Eigen::Vector2d origin_gps, Eigen::Vector2d now_gps){
    double origin_lat = origin_gps.x();
    double origin_log = origin_gps.y();
    double now_lat = now_gps.x();
    double now_log = now_gps.y();
    double now_NEU_x = C_EARTH * cos(deg2rad(now_lat)) * deg2rad(now_log - origin_log);
    double now_NEU_y = C_EARTH * deg2rad(now_lat - origin_lat);
    return Eigen::Vector2d(now_NEU_x, now_NEU_y);
}



