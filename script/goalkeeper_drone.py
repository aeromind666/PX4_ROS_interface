#!/usr/bin/env python
# ROS python API
import rospy
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
# import all mavros messages and services
from mavros_msgs.msg import *
# from icecream import ic

class GoalKeeperAlgorithm:
    def __init__(self):
        self.agent1Local_vel = Point(0.0, 0.0, 0.0)
        self.agent2VelCommand = Point(0.0, 0.0, 0.0)
        self.agent3VelCommand = Point(0.0, 0.0, 0.0)
        self.agent2LocalPose = Point(0.0, 0.0, 0.0)
        self.agent3LocalPose = Point(0.0, 0.0, 0.0)
        self.agent2StartPoint = Point(0.0, 0.0, 0.0)
        self.agent3StartPoint = Point(0.0, 0.0, 0.0)
        self.flagagent2LocalPoseHasInit = False
        self.flagagent3LocalPoseHasInit = False
        self.goalkeeper_max_back_distance = 5


    def agent1VelCallback(self, msg):
        self.agent1Local_vel.x = msg.twist.linear.x
        self.agent1Local_vel.y = msg.twist.linear.y

    def agent2PoseCallback(self, msg):
        self.agent2LocalPose.x = msg.pose.position.x
        self.agent2LocalPose.y = msg.pose.position.y
        if self.flagagent2LocalPoseHasInit is False:
            self.flagagent2LocalPoseHasInit = True
            self.agent2StartPoint.x = self.agent2LocalPose.x
            self.agent2StartPoint.y = self.agent2LocalPose.y
            print("update agent2 start point")

    def agent3PoseCallback(self, msg):
        self.agent3LocalPose.x = msg.pose.position.x
        self.agent3LocalPose.y = msg.pose.position.y
        if self.flagagent3LocalPoseHasInit is False:
            self.flagagent3LocalPoseHasInit = True
            self.agent3StartPoint.x = self.agent3LocalPose.x
            self.agent3StartPoint.y = self.agent3LocalPose.y
            print("update agent3 start point")

    def update(self):
        target_vel = self.agent1Local_vel
        self.agent2VelCommand.x = target_vel.x
        self.agent2VelCommand.y = target_vel.y
        self.agent3VelCommand.x = target_vel.x
        self.agent3VelCommand.y = target_vel.y

        if (self.agent2LocalPose.y > (self.agent2StartPoint.y + self.goalkeeper_max_back_distance)):
            if self.agent2VelCommand.y > 0:
                self.agent2VelCommand.y = 0
                print("agent2 touch up boundary")
        
        if (self.agent2LocalPose.y < self.agent2StartPoint.y):
            if self.agent2VelCommand.y < 0:
                self.agent2VelCommand.y = 0
                print("agent2 touch low boundary")
        
        if (self.agent3LocalPose.y > (self.agent3StartPoint.y + self.goalkeeper_max_back_distance)):
            if self.agent3VelCommand.y > 0:
                self.agent3VelCommand.y = 0
                print("agent3 touch up boundary")
        
        if (self.agent3LocalPose.y < self.agent3StartPoint.y):
            if self.agent3VelCommand.y < 0:
                self.agent3VelCommand.y = 0
                print("agent3 touch low boundary")

    def getVelCommand(self):
        return self.agent2VelCommand, self.agent3VelCommand

def pubAgentVelCommand(pub, command):
    msg = TwistStamped()
    msg.header.stamp = rospy.Time.now()
    msg.twist.linear.x = command.x
    msg.twist.linear.y = command.y
    pub.publish(msg)

def main():
    # initiate node
    rospy.init_node('goal_keeper_node', anonymous=True) 
    rate = rospy.Rate(10)

    # algorithm object
    controller = GoalKeeperAlgorithm()

    # Subscribe to drone's state
    rospy.Subscriber('/uav1/mavros/local_position/velocity_local', TwistStamped, controller.agent1VelCallback)           
    rospy.Subscriber('/uav2/mavros/local_position/pose', PoseStamped, controller.agent2PoseCallback)
    rospy.Subscriber('/uav3/mavros/local_position/pose', PoseStamped, controller.agent3PoseCallback)

    # Setpoint publisher    
    agent2_sp_pub = rospy.Publisher('/uav2/algorithm/cmd_vel', TwistStamped, queue_size=1)
    agent3_sp_pub = rospy.Publisher('/uav3/algorithm/cmd_vel', TwistStamped, queue_size=1)

    # ROS main loop
    while not rospy.is_shutdown():
        controller.update()
        agent2_command, agent3_command = controller.getVelCommand()
        pubAgentVelCommand(agent2_sp_pub, agent2_command)
        pubAgentVelCommand(agent3_sp_pub, agent3_command)
        # ic(agent2_command)
        # ic(agent3_command)
        # ic(controller.agent2StartPoint)
        # ic(controller.agent3StartPoint)
        print(rospy.Time.now())

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass    