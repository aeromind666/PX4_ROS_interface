#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>
#include <std_msgs/UInt32.h>

/**
PX4 native flight stack 

optional px4 mode for set_mode

String  Description and notes
MANUAL
ACRO
ALTCTL
POSCTL
OFFBOARD
STABILIZED
RATTITUDE
AUTO.MISSION
AUTO.LOITER disable RC failsafe, which can be done by setting NAV_RCL_ACT parameter to 0
AUTO.RTL
AUTO.LAND
AUTO.RTGS
AUTO.READY
AUTO.TAKEOFF
**/


class ExpUAV
{
public:
    ExpUAV(const ros::NodeHandle & nh, const ros::NodeHandle & pnh, const double vel_restrict_value);
    ~ExpUAV();
    
    Eigen::Vector2d pos_;
    Eigen::Vector2d vel_;
    Eigen::Vector2d self_cartesian_pos_;
    double current_height_;
    mavros_msgs::State state_;
    double home_height_;
    uint32_t satellites_number_;
    sensor_msgs::NavSatFix global_position_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    void pubLocalCmdVel(Eigen::Vector2d & u, const double & des_height);
    void pubLocalCmdVelYawrate(Eigen::Vector2d u, const double des_height, const double yaw_rate);
    void pubLocalCmdVelYawangle(Eigen::Vector2d u, const double des_height, const double des_yaw_angle);
    void pubSetGPOrigin(const geographic_msgs::GeoPointStamped & msg);
    void pubYawCmdVel(const double des_yaw);
    void pubLocalPosition(Eigen::Vector2d position, double target_height);
    void pubCartesianPoseYawSelf();

    void setCommonOriginGPSWGS84(double lat, double log);
private:
    ros::NodeHandle nh_, pnh_;
    // subscriber the info of connected, armed, guided, manual_input, mode, system_status
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber vel_sub_;
    ros::Subscriber global_position_sub_;
    ros::Subscriber home_position_sub_;
    ros::Subscriber satellites_sub_;

    ros::Publisher local_cmd_vel_pub_;
    ros::Publisher set_gp_origin_pub_;
    ros::Publisher local_position_pub_;
    ros::Publisher cartesian_position_yaw_pub_;
    ros::Publisher yaw_pub_;
    
    double current_yaw_;

    Eigen::Vector2d common_origin_gps_WGS84_; //(lat, log)
    double vel_restrict_value_;

    void stateCallback(const mavros_msgs::StateConstPtr & msg);
    void poseCallback(const geometry_msgs::PoseStampedConstPtr & msg);
    void velCallback(const geometry_msgs::TwistStampedConstPtr & msg);
    void globalPositionCallback(const sensor_msgs::NavSatFixConstPtr & msg);
    void homePositionCallback(const mavros_msgs::HomePositionConstPtr & msg);
    void satellitesCallback(const std_msgs::UInt32ConstPtr & msg);

    Eigen::Vector2d calculateENUfromWGS84(Eigen::Vector2d origin_gps, Eigen::Vector2d now_gps);
    double wrap_yaw_angle(double yaw_angle, const double low, const double high);
};

