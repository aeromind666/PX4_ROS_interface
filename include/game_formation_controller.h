#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <queue>
#include "game_uav.h"

class ExpFormationController
{
public:
    ExpFormationController(const ros::NodeHandle & nh, const ros::NodeHandle & pnh);
    ExpFormationController():
        ExpFormationController(ros::NodeHandle(), ros::NodeHandle("~")){}
    ~ExpFormationController();
    void mainloop(const ros::TimerEvent & time);
    enum StateType{INIT_STATE=1, ALGORITHEM_CONTROL=2, KEEP_CURRENT_POSE=3};

private:
    ros::NodeHandle nh_, pnh_;
    ExpUAV exp_uav1_, exp_uav2_, exp_uav3_;

    Eigen::Vector3d uav1_init_pos_;
    Eigen::Vector3d uav2_init_pos_;
    Eigen::Vector3d uav3_init_pos_;

    double param_boundary_x_;
    double param_boundary_y_;
    double param_boundary_z_;
    double min_algorithm_command_thresold_;
    double min_safe_diatance_between_uav_;
    double vel_yaw_control_mode_;

    ros::Timer main_loop_timer_;

    ros::Subscriber uav1_vel_yawrate_command_sub;
    ros::Subscriber uav2_vel_yawrate_command_sub;
    ros::Subscriber uav3_vel_yawrate_command_sub;

    ros::Subscriber uav1_vel_yawangle_command_sub;
    ros::Subscriber uav2_vel_yawangle_command_sub;
    ros::Subscriber uav3_vel_yawangle_command_sub;

    Eigen::Vector2d pos_des_in_process_uav1_;
    Eigen::Vector2d pos_des_in_process_uav2_;
    Eigen::Vector2d pos_des_in_process_uav3_;

    Eigen::Vector2d vel_des_uav1_;
    Eigen::Vector2d vel_des_uav2_;
    Eigen::Vector2d vel_des_uav3_;
    
    double yaw_rate_des_uav1_;
    double yaw_rate_des_uav2_;
    double yaw_rate_des_uav3_;

    double yaw_angle_des_uav1_;
    double yaw_angle_des_uav2_;
    double yaw_angle_des_uav3_;

    uint32_t latest_command_seq_;
    std::queue<uint32_t> has_executed_command_seq_;

    Eigen::Vector3d des_yaw_;

    StateType state_;

    void uav1VelYawrateCommandCallback(const geometry_msgs::TwistStampedConstPtr& msg);
    void uav2VelYawrateCommandCallback(const geometry_msgs::TwistStampedConstPtr& msg);
    void uav3VelYawrateCommandCallback(const geometry_msgs::TwistStampedConstPtr& msg);

    void uav1VelYawangleCommandCallback(const geometry_msgs::TwistStampedConstPtr& msg);
    void uav2VelYawangleCommandCallback(const geometry_msgs::TwistStampedConstPtr& msg);
    void uav3VelYawangleCommandCallback(const geometry_msgs::TwistStampedConstPtr& msg);

    double pow2(double x);

    void loadParameter();
    void initializeState();
    void initCenterOfCoordinate();
    void setMode();
    bool touchGeofenceBoundary();
    bool minDistanceBetweenUAVIsEnough();
    bool checkNeedAlgorithmControl();
    void updateHasExecuteCommandSeq();
    void updatePoseInProcessForKeepCurrentPose();
};


