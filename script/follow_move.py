#!/usr/bin/env python
# ROS python API
import rospy
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
# import all mavros messages and services
from mavros_msgs.msg import *
#from icecream import ic

class FollowAlgorithm:
    def __init__(self):
        self.agent1Local_vel = Point(0.0, 0.0, 0.0)
        self.agent2VelCommand = Point(0.0, 0.0, 0.0)
        self.agent3VelCommand = Point(0.0, 0.0, 0.0)

    def agent1VelCallback(self, msg):
        self.agent1Local_vel.x = msg.twist.linear.x
        self.agent1Local_vel.y = msg.twist.linear.y

    def update(self):
        target_vel = self.agent1Local_vel
        self.agent2VelCommand.x = target_vel.x
        self.agent2VelCommand.y = target_vel.y
        self.agent3VelCommand.x = target_vel.x
        self.agent3VelCommand.y = target_vel.y

    def getVelCommand(self):
        return self.agent2VelCommand, self.agent3VelCommand

def pubAgentVelCommand(pub, command):
    msg = TwistStamped()
    msg.header.stamp = rospy.Time.now()
    msg.twist.linear.x = command.x
    msg.twist.linear.y = command.y
    msg.twist.angular.z = 0.0
    pub.publish(msg)

def main():
    # initiate node
    rospy.init_node('follow_move_node', anonymous=True) 
    rate = rospy.Rate(10)

    # algorithm object
    controller = FollowAlgorithm()

    # Subscribe to drone's state
    rospy.Subscriber('/uav1/mavros/local_position/velocity_local', TwistStamped, controller.agent1VelCallback)           

    # Setpoint publisher    
    agent2_sp_pub = rospy.Publisher('/uav2/algorithm/cmd_vel_yaw_rate', TwistStamped, queue_size=1)
    agent3_sp_pub = rospy.Publisher('/uav3/algorithm/cmd_vel_yaw_rate', TwistStamped, queue_size=1)

    # ROS main loop
    while not rospy.is_shutdown():
        controller.update()
        agent2_command, agent3_command = controller.getVelCommand()
        pubAgentVelCommand(agent2_sp_pub, agent2_command)
        pubAgentVelCommand(agent3_sp_pub, agent3_command)
        # ic(agent2_command)
        # ic(agent3_command)
        print(rospy.Time.now())

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass    