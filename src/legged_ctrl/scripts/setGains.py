import rospy 
import time
import numpy as np
import sys

from unitree_legged_msgs.msg import MotorCmd
from unitree_legged_msgs.msg import MotorState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

class ModelPublisher: 
    def __init__(self): 
        self.gain_publisher = rospy.Publisher("/a1_debug/" + "low_level_gains", Float64MultiArray, queue_size=1)
        self.gain_msg = Float64MultiArray()
        dim = []
        # dim.append(MultiArrayDimension("data", 24, 0))
        # self.gain_msg.layout.dim = dim
        self.gain_msg.data = [0]*6

    def send_gains(self):
        print(self.gain_msg.data)
        self.gain_publisher.publish(self.gain_msg)

if __name__ == "__main__": 
    rospy.init_node("gain_tuner")
    modelStatePub = ModelPublisher()
    rate = rospy.Rate(100)
    rospy.sleep(0.1)
    t_start = rospy.get_time() 
    for i in range(6):
        modelStatePub.gain_msg.data[i] = float(sys.argv[i + 1])
    print(modelStatePub.gain_msg.data)
    # modelStatePub.send_gains()
    for i in range(50):
        rate.sleep()
    modelStatePub.send_gains()
    
    # while not rospy.is_shutdown():
    # #     modelStatePub.gain_msg.data[1] = int(input("Enter gains: "))
    # #     modelStatePub.gain_msg.data[2] = int(input(""))
    #     modelStatePub.send_gains()
