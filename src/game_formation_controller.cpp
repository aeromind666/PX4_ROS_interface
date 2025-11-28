#include "game_formation_controller.h"

ExpFormationController::ExpFormationController(
    const ros::NodeHandle & nh, const ros::NodeHandle & pnh)
    :nh_(nh),
    pnh_(pnh),
    exp_uav1_(ros::NodeHandle("uav1"), pnh_, 2),
    exp_uav2_(ros::NodeHandle("uav2"), pnh_, 2),
    exp_uav3_(ros::NodeHandle("uav3"), pnh_, 2),
    state_(INIT_STATE){
        param_boundary_x_ = 100.0;
        param_boundary_y_ = 100.0;
        param_boundary_z_ = 20.0;
        min_algorithm_command_thresold_ = 0.1;
        min_safe_diatance_between_uav_ = 3.0;

        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
        initializeState();
        initCenterOfCoordinate();
        loadParameter();
        setMode();

        pos_des_in_process_uav1_ = Eigen::Vector2d(uav1_init_pos_.x(), uav1_init_pos_.y());
        pos_des_in_process_uav2_ = Eigen::Vector2d(uav2_init_pos_.x(), uav2_init_pos_.y());
        pos_des_in_process_uav3_ = Eigen::Vector2d(uav3_init_pos_.x(), uav3_init_pos_.y());
        vel_des_uav1_ = Eigen::Vector2d(0, 0);
        vel_des_uav2_ = Eigen::Vector2d(0, 0);
        vel_des_uav3_ = Eigen::Vector2d(0, 0);
        for(uint32_t i = 10; i < 15; i++){
            has_executed_command_seq_.push(i);
        }
        uav1_vel_yawrate_command_sub = nh_.subscribe("/uav1/algorithm/cmd_vel_yaw_rate", 1,
            &ExpFormationController::uav1VelYawrateCommandCallback, this);
        uav2_vel_yawrate_command_sub = nh_.subscribe("/uav2/algorithm/cmd_vel_yaw_rate", 1,
            &ExpFormationController::uav2VelYawrateCommandCallback, this);
        uav3_vel_yawrate_command_sub = nh_.subscribe("/uav3/algorithm/cmd_vel_yaw_rate", 1,
            &ExpFormationController::uav3VelYawrateCommandCallback, this);

        uav1_vel_yawangle_command_sub = nh_.subscribe("/uav1/algorithm/cmd_vel_yaw_angle", 1,
            &ExpFormationController::uav1VelYawangleCommandCallback, this);
        uav2_vel_yawangle_command_sub = nh_.subscribe("/uav2/algorithm/cmd_vel_yaw_angle", 1,
            &ExpFormationController::uav2VelYawangleCommandCallback, this);
        uav3_vel_yawangle_command_sub = nh_.subscribe("/uav3/algorithm/cmd_vel_yaw_angle", 1,
            &ExpFormationController::uav3VelYawangleCommandCallback, this);

        main_loop_timer_ = nh_.createTimer(ros::Duration(0.05), 
        &ExpFormationController::mainloop, this);
}
ExpFormationController::~ExpFormationController(){}

void ExpFormationController::mainloop(const ros::TimerEvent & time){
    if(true == checkNeedAlgorithmControl()){
        state_ = ALGORITHEM_CONTROL;
    }
    else if(state_ != INIT_STATE){
        state_ = KEEP_CURRENT_POSE;
    }

    if((true == touchGeofenceBoundary()) && (state_ != INIT_STATE)){
        ROS_INFO("touch geofence boundary");
        state_ = KEEP_CURRENT_POSE;
    }

    if((true != minDistanceBetweenUAVIsEnough()) && (state_ != INIT_STATE)){
        ROS_ERROR("min distance too small, not safe, set state to position mode");
        state_ = KEEP_CURRENT_POSE;
    }

    ROS_INFO("state %d", state_);

    // TODO: this state need to consider more for sfatey
    if(INIT_STATE == state_){
        exp_uav1_.pubLocalCmdVel(vel_des_uav1_, uav1_init_pos_.z());
        exp_uav2_.pubLocalCmdVel(vel_des_uav2_, uav2_init_pos_.z());
        exp_uav3_.pubLocalCmdVel(vel_des_uav3_, uav3_init_pos_.z());
        updatePoseInProcessForKeepCurrentPose();
    }

    if(KEEP_CURRENT_POSE == state_){
        exp_uav1_.pubLocalPosition(pos_des_in_process_uav1_, uav1_init_pos_.z());
        exp_uav2_.pubLocalPosition(pos_des_in_process_uav2_, uav2_init_pos_.z());
        exp_uav3_.pubLocalPosition(pos_des_in_process_uav3_, uav3_init_pos_.z());
    }

    if(ALGORITHEM_CONTROL == state_){
        if(fabs(vel_yaw_control_mode_ - 1.0) < 1e-7){
            exp_uav1_.pubLocalCmdVelYawangle(vel_des_uav1_, uav1_init_pos_.z(), yaw_angle_des_uav1_);
            exp_uav2_.pubLocalCmdVelYawangle(vel_des_uav2_, uav2_init_pos_.z(), yaw_angle_des_uav2_);
            exp_uav3_.pubLocalCmdVelYawangle(vel_des_uav3_, uav3_init_pos_.z(), yaw_angle_des_uav3_);
        }
        else{
            exp_uav1_.pubLocalCmdVelYawrate(vel_des_uav1_, uav1_init_pos_.z(), yaw_rate_des_uav1_);
            exp_uav2_.pubLocalCmdVelYawrate(vel_des_uav2_, uav2_init_pos_.z(), yaw_rate_des_uav2_);
            exp_uav3_.pubLocalCmdVelYawrate(vel_des_uav3_, uav3_init_pos_.z(), yaw_rate_des_uav3_);
        }

        updateHasExecuteCommandSeq();
        updatePoseInProcessForKeepCurrentPose();
    }

    exp_uav1_.pubCartesianPoseYawSelf();
    exp_uav2_.pubCartesianPoseYawSelf();
    exp_uav3_.pubCartesianPoseYawSelf();
}

bool ExpFormationController::checkNeedAlgorithmControl()
{
    if(true == has_executed_command_seq_.empty()){
        ROS_INFO("has_executed_command_seq_ error");
        return  false;
    }
    else{
        if(latest_command_seq_ == has_executed_command_seq_.front()){
            ROS_INFO("no new command, not need to execute algorithm");
            return false;
        }
    }

    if((vel_des_uav1_.norm() > min_algorithm_command_thresold_) ||
       (vel_des_uav2_.norm() > min_algorithm_command_thresold_) ||
       (vel_des_uav3_.norm() > min_algorithm_command_thresold_)){
        return true;
    }
    else{
        return false;
    }
}

void ExpFormationController::updateHasExecuteCommandSeq(){
    has_executed_command_seq_.push(latest_command_seq_);
    if(false == has_executed_command_seq_.empty()){
        has_executed_command_seq_.pop();
    }
    else{
        ROS_ERROR("has_executed_command_seq_ is empty, init error");
    }
}

//TODO: need to consider
bool ExpFormationController::touchGeofenceBoundary(){
    if((abs(exp_uav1_.pos_.x()) > param_boundary_x_) ||
       (abs(exp_uav2_.pos_.x()) > param_boundary_x_) || 
       (abs(exp_uav3_.pos_.x()) > param_boundary_x_)){
        return true;
    } 

    if((abs(exp_uav1_.pos_.y()) > param_boundary_y_) || 
       (abs(exp_uav2_.pos_.y()) > param_boundary_y_) || 
       (abs(exp_uav3_.pos_.y()) > param_boundary_y_)){
        return true;
    } 

    if((abs(exp_uav1_.current_height_) > param_boundary_z_) || 
       (abs(exp_uav2_.current_height_) > param_boundary_z_) || 
       (abs(exp_uav3_.current_height_) > param_boundary_z_)){
        return true;
    } 

    return false;
}

bool ExpFormationController::minDistanceBetweenUAVIsEnough(){
    double distance12 = (exp_uav1_.self_cartesian_pos_ - exp_uav2_.self_cartesian_pos_).norm();
    double distance13 = (exp_uav1_.self_cartesian_pos_ - exp_uav3_.self_cartesian_pos_).norm();
    double distance23 = (exp_uav2_.self_cartesian_pos_ - exp_uav3_.self_cartesian_pos_).norm();

    if(distance12 < min_safe_diatance_between_uav_){
        ROS_ERROR("distance12 is too small");
        return false;
    }

    if(distance13 < min_safe_diatance_between_uav_){
        ROS_ERROR("distance13 is too small");
        return false;
    }

    if(distance23 < min_safe_diatance_between_uav_){
        ROS_ERROR("distance23 is too small");
        return false;
    }

    return true;
}

void ExpFormationController::updatePoseInProcessForKeepCurrentPose(){
    pos_des_in_process_uav1_ = Eigen::Vector2d(exp_uav1_.pos_[0], exp_uav1_.pos_[1]); 
    pos_des_in_process_uav2_ = Eigen::Vector2d(exp_uav2_.pos_[0], exp_uav2_.pos_[1]); 
    pos_des_in_process_uav3_ = Eigen::Vector2d(exp_uav3_.pos_[0], exp_uav3_.pos_[1]); 
}

void ExpFormationController::uav1VelYawrateCommandCallback(const geometry_msgs::TwistStampedConstPtr& msg){
    vel_des_uav1_[0] = msg->twist.linear.x;
    vel_des_uav1_[1] = msg->twist.linear.y;
    yaw_rate_des_uav1_ = msg->twist.angular.z;
}

void ExpFormationController::uav2VelYawrateCommandCallback(const geometry_msgs::TwistStampedConstPtr& msg){
    vel_des_uav2_[0] = msg->twist.linear.x;
    vel_des_uav2_[1] = msg->twist.linear.y;
    yaw_rate_des_uav2_ = msg->twist.angular.z;
    latest_command_seq_ = msg->header.seq;
}

void ExpFormationController::uav3VelYawrateCommandCallback(const geometry_msgs::TwistStampedConstPtr& msg){
    vel_des_uav3_[0] = msg->twist.linear.x;
    vel_des_uav3_[1] = msg->twist.linear.y;
    yaw_rate_des_uav3_ = msg->twist.angular.z;
}

void ExpFormationController::uav1VelYawangleCommandCallback(const geometry_msgs::TwistStampedConstPtr& msg){
    vel_des_uav1_[0] = msg->twist.linear.x;
    vel_des_uav1_[1] = msg->twist.linear.y;
    yaw_angle_des_uav1_ = msg->twist.angular.z;
}

void ExpFormationController::uav2VelYawangleCommandCallback(const geometry_msgs::TwistStampedConstPtr& msg){
    vel_des_uav2_[0] = msg->twist.linear.x;
    vel_des_uav2_[1] = msg->twist.linear.y;
    yaw_angle_des_uav2_ = msg->twist.angular.z;
    latest_command_seq_ = msg->header.seq;
}

void ExpFormationController::uav3VelYawangleCommandCallback(const geometry_msgs::TwistStampedConstPtr& msg){
    vel_des_uav3_[0] = msg->twist.linear.x;
    vel_des_uav3_[1] = msg->twist.linear.y;
    yaw_angle_des_uav3_ = msg->twist.angular.z;
}

void ExpFormationController::initializeState(){
    //wait for service of arming and set_mode
    exp_uav1_.arming_client_.waitForExistence();
    exp_uav1_.set_mode_client_.waitForExistence();

    exp_uav2_.arming_client_.waitForExistence();
    exp_uav2_.set_mode_client_.waitForExistence();

    exp_uav3_.arming_client_.waitForExistence();
    exp_uav3_.set_mode_client_.waitForExistence();
    ROS_INFO("The service for arming and set_mode is ready");
    //wait for FCU connection
    while (ros::ok() && exp_uav1_.state_.connected
        && exp_uav2_.state_.connected
        && exp_uav3_.state_.connected){
            ros::Rate rate(20.0);
            rate.sleep();
            ROS_INFO("The onboard computer does not connect the fcu!");
            ros::spinOnce();
    }
}

void ExpFormationController::initCenterOfCoordinate(){
    while((sensor_msgs::NavSatStatus::STATUS_NO_FIX == exp_uav1_.global_position_.status.status)
       || (0 == exp_uav1_.global_position_.latitude)
       || (exp_uav1_.satellites_number_ < 10)){
        ROS_INFO("The gps of uav1 is no fix");
        ros::Rate rate(20.0);
        rate.sleep();
        ros::spinOnce();
    }

    while((sensor_msgs::NavSatStatus::STATUS_NO_FIX == exp_uav2_.global_position_.status.status)
       || (0 == exp_uav2_.global_position_.latitude)
       || (exp_uav2_.satellites_number_ < 10)){
        ROS_INFO("The gps of uav2 is no fix");
        ros::Rate rate(20.0);
        rate.sleep();
        ros::spinOnce();
    }

    while((sensor_msgs::NavSatStatus::STATUS_NO_FIX == exp_uav3_.global_position_.status.status)
       || (0 == exp_uav3_.global_position_.latitude)
       || (exp_uav3_.satellites_number_ < 10)){
        ROS_INFO("The gps of uav3 is no fix");
        ros::Rate rate(20.0);
        rate.sleep();
        ros::spinOnce();
    }

    ROS_INFO("The gps of UAVs is fix");
    ROS_DEBUG("exp_uav1_.satellites_number_:%d", exp_uav1_.satellites_number_);
    ROS_DEBUG("exp_uav2_.satellites_number_:%d", exp_uav2_.satellites_number_);
    ROS_DEBUG("exp_uav3_.satellites_number_:%d", exp_uav3_.satellites_number_);
    
    geographic_msgs::GeoPointStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.position.latitude = exp_uav1_.global_position_.latitude;
    msg.position.longitude = exp_uav1_.global_position_.longitude;
    msg.position.altitude = exp_uav1_.global_position_.altitude;

    exp_uav1_.pubSetGPOrigin(msg);
    exp_uav2_.pubSetGPOrigin(msg);
    exp_uav3_.pubSetGPOrigin(msg);
    ROS_INFO("global_position_origin/latitude: %f, longitude: %f, altitude: %f", 
                                        msg.position.latitude, msg.position.longitude, msg.position.altitude);

    exp_uav1_.setCommonOriginGPSWGS84(exp_uav1_.global_position_.latitude, exp_uav1_.global_position_.longitude);
    exp_uav2_.setCommonOriginGPSWGS84(exp_uav1_.global_position_.latitude, exp_uav1_.global_position_.longitude);
    exp_uav3_.setCommonOriginGPSWGS84(exp_uav1_.global_position_.latitude, exp_uav1_.global_position_.longitude);
}

void ExpFormationController::setMode(){
    mavros_msgs::SetMode set_mode_srv;
    set_mode_srv.request.custom_mode = "OFFBOARD";

    if(exp_uav1_.state_.armed && exp_uav1_.state_.mode != "OFFBOARD"){
        if(exp_uav1_.set_mode_client_.call(set_mode_srv) &&
                set_mode_srv.response.mode_sent){
                        ROS_INFO("uav1 offboard enabled");
        }
    }
    
    if(exp_uav2_.state_.armed && exp_uav2_.state_.mode != "OFFBOARD"){
        if(exp_uav2_.set_mode_client_.call(set_mode_srv) &&
                set_mode_srv.response.mode_sent){
                        ROS_INFO("uav2 offboard enabled");
        }
    }

    if(exp_uav3_.state_.armed && exp_uav3_.state_.mode != "OFFBOARD"){
        if(exp_uav3_.set_mode_client_.call(set_mode_srv) &&
                set_mode_srv.response.mode_sent){
                        ROS_INFO("uav3 offboard enabled");
        }
    }
}

//latitude weidu longitude jingdu
void ExpFormationController::loadParameter(){
    std::vector<double> uav1_init_pos;
    std::vector<double> uav2_init_pos;
    std::vector<double> uav3_init_pos;

    pnh_.param<std::vector<double>>("uav1_init_pos", uav1_init_pos, 
                    std::vector<double>({0.0, 0.0, 4.0}));

    pnh_.param<std::vector<double>>("uav2_init_pos", uav2_init_pos, 
                    std::vector<double>({2.0, 14.0, 4.0}));

    pnh_.param<std::vector<double>>("uav3_init_pos", uav3_init_pos, 
                    std::vector<double>({-14.0, 8.0, 4.0}));

    // TODO: need to check here                
    // while(abs(exp_uav1_.home_height_) < 0.01 || 
    //       abs(exp_uav2_.home_height_) < 0.01 || 
    //       abs(exp_uav3_.home_height_) < 0.01){
    //      ROS_WARN("The home height fail to set");
    //      ros::Rate rate(20.0);
    //      rate.sleep();
    //      ros::spinOnce();
    // }
    uav1_init_pos[2] = uav1_init_pos[2] + exp_uav1_.home_height_;
    uav2_init_pos[2] = uav2_init_pos[2] + exp_uav2_.home_height_;
    uav3_init_pos[2] = uav3_init_pos[2] + exp_uav3_.home_height_;
                                                                    
    uav1_init_pos_ = Eigen::Vector3d(uav1_init_pos[0], uav1_init_pos[1], uav1_init_pos[2]); 
    uav2_init_pos_ = Eigen::Vector3d(uav2_init_pos[0], uav2_init_pos[1], uav2_init_pos[2]); 
    uav3_init_pos_ = Eigen::Vector3d(uav3_init_pos[0], uav3_init_pos[1], uav3_init_pos[2]);

    pnh_.param<double>("param_boundary_x", param_boundary_x_, 100.0);
    pnh_.param<double>("param_boundary_y", param_boundary_y_, 100.0);
    pnh_.param<double>("param_boundary_z", param_boundary_z_, 20.0);
    param_boundary_z_ = param_boundary_z_ + exp_uav1_.home_height_;
    pnh_.param<double>("min_algorithm_command_thresold", min_algorithm_command_thresold_, 0.1);
    pnh_.param<double>("min_safe_diatance_between_uav", min_safe_diatance_between_uav_, 3.0);
    pnh_.param<double>("vel_yaw_control_mode", vel_yaw_control_mode_, 0.0);
}

double ExpFormationController::pow2(double x){
    return x * x;
}