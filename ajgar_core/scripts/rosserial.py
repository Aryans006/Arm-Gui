
#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray

def __init__(self):
    node = 'rosserial_publishing'
    # Initialize the ROS node
    rospy.init_node(node, anonymous=True)
    
    # Create a Publisher object
    pub_base = rospy.Publisher('number', Int32, queue_size=10)
    pub_shoulder = rospy.Publisher('number', Int32, queue_size=10)
    pub_elbow = rospy.Publisher('number', Int32, queue_size=10)
    pub_up_wrist = rospy.Publisher('number', Int32, queue_size=10)
    pub_low_wrist = rospy.Publisher('number', Int32, queue_size=10)
    
    rospy.Subscriber("/slider", Float64MultiArray, self.jointpub)
    
    # Set the rate at which messages are published
    rate = rospy.Rate(1)  # 1 Hz
    
    # Initialize a counter
    # count = 0

def jointpub(self,data):







    while not rospy.is_shutdown():
        # Create a message
        msg = Int32()
        msg.data = count


        
        # Log the message to the console
        rospy.loginfo("Publishing: %d", msg.data)
        
        # Publish the message
        pub.publish(msg)
        
        # Increment the counter
        count += 1
        
        # Sleep to maintain the desired publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        number_publisher()
    except rospy.ROSInterruptException:
        pass
