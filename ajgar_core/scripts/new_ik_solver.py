#!/usr/bin/env python3

# standard library imports
# import sys
import rospy
import math

# ROS specific imports
# import rospkg
# import moveit_commander

# custom imports
# import attach
# import detach
# import ikpy.chain
from prettytable import PrettyTable
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import JointState
# from ajgar_core.srv import tfValueSrv, tfValueSrvResponse


class ikSolverClass(object):
    def __init__(self):
        
        # ROS node and service 
        node = 'move_group_python_interface'
        srvnode = 'ikSolverSrv'
        group_name  = "arm_group"
        # self.moveit_result = "/move_group/result"
        # self.gazebo_collision = "/gazebo/collision/info"        
        
        # ROS node initialization 
        rospy.init_node(node)
        rospy.loginfo("publis krre")
        self.pub_base = rospy.Publisher('Base', Float64, queue_size=10)
        self.pub_elbow = rospy.Publisher('shoulder', Float64, queue_size=10)
        self.pub_upwrist = rospy.Publisher('elbow', Float64, queue_size=10)
        self.pub_shoulder = rospy.Publisher('upper_wrist', Float64, queue_size=10)
        self.pub_lowrist = rospy.Publisher('lower_wrist', Float64, queue_size=10)
        self.pub_states = rospy.Publisher('JointState', JointState, queue_size=10)

        self.joint_states = JointState()

        # # MoveIt! commander initialization
        # moveit_commander.roscpp_initialize(sys.argv)
        # robot = moveit_commander.RobotCommander()
        # self.group = moveit_commander.MoveGroupCommander(group_name)

       # global variables initialization)
        self.obj2 = None
        self.count = 0
        rospy.Subscriber("/slider", Float64MultiArray, self.jointValuesCallback)
        rospy.loginfo("base ho gya")
        


    def jointValuesCallback(self, data):
        # Callback function to update custom_joint_values when a new array is received
        if len(data.data) == 5:
            custom_joint_values = data.data
            rospy.loginfo("base gya")
            rospy.loginfo("Received custom_joint_values: {}".format(custom_joint_values))
            
            # self.moveitGoToJointState(custom_joint_values)
            
        else:
            rospy.logwarn("Received array does not have the expected length (5)")   
    
        checkjointvalues = []

        if custom_joint_values == checkjointvalues:
            rospy.loginfo("same hai veloo")

        else:    
            self.pub_base.publish(custom_joint_values[0])
            rospy.loginfo("base ho gya")
            self.pub_shoulder.publish(custom_joint_values[1])
            self.pub_elbow.publish(custom_joint_values[2])
            self.pub_lowrist.publish(custom_joint_values[3])
            self.pub_upwrist.publish(custom_joint_values[4])
            # string = str(math.floor(custom_joint_values[0]))+ str (math.floor(custom_joint_values[1])) + str (math.floor(custom_joint_values[2]))+str (math.floor(custom_joint_values[3]))+str (math.floor(custom_joint_values[4]))
            self.joint_states.position = [round(custom_joint_values[0], 1), round(custom_joint_values[1], 1), round(custom_joint_values[2], 1),
                                          round(custom_joint_values[3], 1), round(custom_joint_values[4], 1)
                                          ]
            rospy.loginfo('publisedddd')
            self.pub_states.publish(self.joint_states)

            checkjointvalues = custom_joint_values

            
    def moveitGoToJointState(self,custom_joint_values):
     
        print("----------")
        joint_names = [
            
            'Base Joint       ',
            'Shoulder Joint   ',
            'Elbow Joint      ',
            'Lower Wrist Joint',
            'Upper Wrist Joint',
            'Suction          '
        ]

        if custom_joint_values is not None and len(custom_joint_values) == 5:
            # Use custom joint values if provided
            joint_goal = custom_joint_values
            rospy.loginfo("if me enter kr gya")

        else:
            print("Invalid custom joint values. Using default values.")
            # Use default joint values
            joint_goal = [0, 0, 0, 0, 0]

            self.pub_base.publish(0)
            self.pub_shoulder.publish(0)
            self.pub_elbow.publish(0)
            self.pub_lowrist.publish(0)
            self.pub_upwrist.publish(0)



        # Convert angles from radians to degrees for printing
        for angle in range(0, 5):# Adjust the loop range to match the length of joint_goal
            print(joint_names[angle], " ", str(joint_goal[angle] * (180 / math.pi)))



        print("----------")

        # self.group.go(joint_goal, wait=True)
        # self.group.stop()

    def run(self):
        # Run the ROS node
        rospy.spin()



	
if __name__ == "__main__":
    while not rospy.is_shutdown():
        ikSolver = ikSolverClass() 
        rospy.spin()
