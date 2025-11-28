#!/usr/bin/env python
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from icecream import ic

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes():
    def __init__(self, uav_name):
        # Drone group name in ROS
        self.uav_name = uav_name
        # Drone state
        self.state = State()
        self.stateTopicName = uav_name + '/' + 'mavros/state'

    def setTakeoff(self):
    	rospy.wait_for_service(self.uav_name + '/' + 'mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy(self.uav_name + '/' + 'mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 3)
    	except rospy.ServiceException, e:
    		print "Service takeoff call failed: %s"%e

    def setArm(self):
        rospy.wait_for_service(self.uav_name + '/' + 'mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy(self.uav_name + '/' + 'mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service(self.uav_name + '/' + 'mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy(self.uav_name + '/' + 'mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service(self.uav_name + '/' + 'mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(self.uav_name + '/' + 'mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service(self.uav_name + '/' + 'mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(self.uav_name + '/' + 'mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service(self.uav_name + '/' + 'mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(self.uav_name + '/' + 'mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service(self.uav_name + '/' + 'mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(self.uav_name + '/' + 'mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service(self.uav_name + '/' + 'mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(self.uav_name + '/' + 'mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
               print "service set_mode call failed: %s. Autoland Mode could not be set."%e
    
    def setReturnMode(self):
        rospy.wait_for_service(self.uav_name + '/' + 'mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(self.uav_name + '/' + 'mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.RTL')
        except rospy.ServiceException, e:
               print "service set_mode call failed: %s. AutoReturn Mode could not be set."%e

    def stateCb(self, msg):
        self.state = msg

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 3.0
        # update the setpoint message with the required altitude
        self.sp.position.z = self.ALT_SP

        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0


# Main function
def main():

    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    uav1_modes = fcuModes('uav1')
    uav2_modes = fcuModes('uav2')
    uav3_modes = fcuModes('uav3')

    # Subscribe to drone state
    ic(uav1_modes.stateTopicName)
    rospy.Subscriber(uav1_modes.stateTopicName, State, uav1_modes.stateCb)
    rospy.Subscriber(uav2_modes.stateTopicName, State, uav2_modes.stateCb)
    rospy.Subscriber(uav3_modes.stateTopicName, State, uav3_modes.stateCb)

    # Setpoint publisher    
    sp1_pub = rospy.Publisher('uav1' + '/' + 'mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    sp2_pub = rospy.Publisher('uav2' + '/' + 'mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    sp3_pub = rospy.Publisher('uav3' + '/' + 'mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    # controller object
    cnt = Controller()
    # ROS loop rate
    rate = rospy.Rate(10.0)

    uav1_modes.setOffboardMode()
    uav2_modes.setOffboardMode()
    uav3_modes.setOffboardMode()
    rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass