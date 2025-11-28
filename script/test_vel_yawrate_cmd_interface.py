#!/usr/bin/env python
# ROS python API
import rospy
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from icecream import ic

class Agent:
    def __init__(self, topic_name):
        self.state_pose = Point(0.0, 0.0, 0.0)
        self.state_yaw = 0.0
        self.cmd_vel = Point(0.0, 0.0, 0.0)
        self.cmd_yaw = 0.0
        self.setpoint_pub = rospy.Publisher(topic_name, TwistStamped, queue_size=1)
    
    def statePoseYawCallback(self, msg):
        self.state_pose.x = msg.pose.position.x
        self.state_pose.y = msg.pose.position.y
        self.state_yaw = msg.pose.orientation.w
    
    def actionVelYawrateSend(self, vel_x, vel_y, yaw_rate):
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.twist.linear.x = vel_x
        msg.twist.linear.y = vel_y
        msg.twist.angular.z = yaw_rate
        self.setpoint_pub.publish(msg)

def main():
    # initiate node
    rospy.init_node('follow_move_node', anonymous=True) 
    rate = rospy.Rate(10)

    agent1 = Agent(topic_name='/uav1/algorithm/cmd_vel_yaw_rate')
    agent2 = Agent(topic_name='/uav2/algorithm/cmd_vel_yaw_rate')
    agent3 = Agent(topic_name='/uav3/algorithm/cmd_vel_yaw_rate')
    rospy.Subscriber('/uav1/self/cartesian_position_yaw', PoseStamped, agent1.statePoseYawCallback)  
    rospy.Subscriber('/uav2/self/cartesian_position_yaw', PoseStamped, agent2.statePoseYawCallback)  
    rospy.Subscriber('/uav3/self/cartesian_position_yaw', PoseStamped, agent3.statePoseYawCallback)  

    # ROS main loop
    while not rospy.is_shutdown():
        ic(agent1.state_pose)
        ic(agent1.state_yaw)
        ic(agent2.state_pose)
        ic(agent2.state_yaw)
        ic(agent3.state_pose)
        ic(agent3.state_yaw)

        agent2.actionVelYawrateSend(vel_x = 0, vel_y = 0.2, yaw_rate=0.2)

        print(rospy.Time.now())
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass    