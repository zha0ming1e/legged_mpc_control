import rospy 
import time
import numpy as np

from unitree_legged_msgs.msg import MotorCmd
from unitree_legged_msgs.msg import MotorState
from gazebo_msgs.msg import ModelState

class ModelPublisher: 
    def __init__(self): 
        # Robot name
        robot_name = rospy.get_param("/robot_name", "a1")
        prefix = robot_name + "_gazebo"

        self.joint_pos = np.zeros((3,4))
        self.joint_vel = np.zeros((3,4))

        # Servo publisher 
        self.cmd_publishers = [None for i in range(12)]
        self.cmd_publishers[0] = rospy.Publisher(prefix + "/FL_hip_controller/command", MotorCmd, queue_size=1)
        self.cmd_publishers[1] = rospy.Publisher(prefix + "/FL_thigh_controller/command", MotorCmd, queue_size=1)
        self.cmd_publishers[2] = rospy.Publisher(prefix + "/FL_calf_controller/command", MotorCmd, queue_size=1)

        self.cmd_publishers[3] = rospy.Publisher(prefix + "/FR_hip_controller/command", MotorCmd, queue_size=1)
        self.cmd_publishers[4] = rospy.Publisher(prefix + "/FR_thigh_controller/command", MotorCmd, queue_size=1)
        self.cmd_publishers[5] = rospy.Publisher(prefix + "/FR_calf_controller/command", MotorCmd, queue_size=1)   

        self.cmd_publishers[6] = rospy.Publisher(prefix + "/RL_hip_controller/command", MotorCmd, queue_size=1)
        self.cmd_publishers[7] = rospy.Publisher(prefix + "/RL_thigh_controller/command", MotorCmd, queue_size=1)
        self.cmd_publishers[8] = rospy.Publisher(prefix + "/RL_calf_controller/command", MotorCmd, queue_size=1)

        self.cmd_publishers[9] = rospy.Publisher(prefix + "/RR_hip_controller/command", MotorCmd, queue_size=1)
        self.cmd_publishers[10] = rospy.Publisher(prefix + "/RR_thigh_controller/command", MotorCmd, queue_size=1)
        self.cmd_publishers[11] = rospy.Publisher(prefix + "/RR_calf_controller/command", MotorCmd, queue_size=1) 

        ## Subscribers - Joints
        self.joint_state_subscribers = [None for i in range(12)]
        self.joint_state_subscribers[0] = rospy.Subscriber(prefix + "/FL_hip_controller/state", MotorState, self.FLHipCallback, queue_size=1)
        self.joint_state_subscribers[1] = rospy.Subscriber(prefix + "/FL_thigh_controller/state", MotorState, self.FLThighCallback, queue_size=1)
        self.joint_state_subscribers[2] = rospy.Subscriber(prefix + "/FL_calf_controller/state", MotorState, self.FLCalfCallback, queue_size=1)
        
        self.joint_state_subscribers[3] = rospy.Subscriber(prefix + "/FR_hip_controller/state", MotorState, self.FRHipCallback, queue_size=1)
        self.joint_state_subscribers[4] = rospy.Subscriber(prefix + "/FR_thigh_controller/state", MotorState, self.FRThighCallback, queue_size=1)
        self.joint_state_subscribers[5] = rospy.Subscriber(prefix + "/FR_calf_controller/state", MotorState, self.FRCalfCallback, queue_size=1)
        
        self.joint_state_subscribers[6] = rospy.Subscriber(prefix + "/RL_hip_controller/state", MotorState, self.RLHipCallback, queue_size=1)
        self.joint_state_subscribers[7] = rospy.Subscriber(prefix + "/RL_thigh_controller/state", MotorState, self.RLThighCallback, queue_size=1)
        self.joint_state_subscribers[8] = rospy.Subscriber(prefix + "/RL_calf_controller/state", MotorState, self.RLCalfCallback, queue_size=1)

        self.joint_state_subscribers[9] = rospy.Subscriber(prefix + "/RR_hip_controller/state", MotorState, self.RRHipCallback, queue_size=1)
        self.joint_state_subscribers[10] = rospy.Subscriber(prefix + "/RR_thigh_controller/state", MotorState, self.RRThighCallback, queue_size=1)
        self.joint_state_subscribers[11] = rospy.Subscriber(prefix + "/RR_calf_controller/state", MotorState, self.RRCalfCallback, queue_size=1)

        # Trunk publisher 
        self.trunk_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1000)

        self.model_state_msg = ModelState()
        self.model_state_msg.model_name = robot_name + "_gazebo"
        self.model_state_msg.pose.position.x = 0
        self.model_state_msg.pose.position.y = 0
        self.model_state_msg.pose.position.z = 0.17

        self.model_state_msg.pose.orientation.x = 0
        self.model_state_msg.pose.orientation.y = 0
        self.model_state_msg.pose.orientation.z = 0
        self.model_state_msg.pose.orientation.w = 1


        # Init Motor Publisher parameters 
        self.motor_msgs = [MotorCmd() for x in range(12)]
        for i in range(4):
            self.motor_msgs[i*3].mode = 0x0A
            self.motor_msgs[i*3].Kp = 70 
            self.motor_msgs[i*3].dq = 0
            self.motor_msgs[i*3].Kd = 3
            self.motor_msgs[i*3].tau = 0 

            self.motor_msgs[i*3+1].mode = 0x0A
            self.motor_msgs[i*3+1].Kp = 180 
            self.motor_msgs[i*3+1].dq = 0
            self.motor_msgs[i*3+1].Kd = 8
            self.motor_msgs[i*3+1].tau = 0

            self.motor_msgs[i*3+2].mode = 0x0A
            self.motor_msgs[i*3+2].Kp = 300
            self.motor_msgs[i*3+2].dq = 0
            self.motor_msgs[i*3+2].Kd = 15
            self.motor_msgs[i*3+2].tau = 0

    def servoUpdate(self, position):
        pos = position.ravel(order="F")
        for i, pub in enumerate(self.cmd_publishers):
            self.motor_msgs[i].q = pos[i]
            pub.publish(self.motor_msgs[i])
        self.trunk_pub.publish(self.model_state_msg)

    ### Sensor Callbacks ### 
    def FLHipCallback(self, msg):
        self.joint_pos[0,0] = msg.q
        self.joint_vel[0,0] = msg.dq
    
    def FLThighCallback(self, msg):
        self.joint_pos[1,0] = msg.q 
        self.joint_vel[1,0] = msg.dq

    def FLCalfCallback(self, msg):
        self.joint_pos[2,0] = msg.q 
        self.joint_vel[2,0] = msg.dq

    def FRHipCallback(self, msg):
        self.joint_pos[0,1] = msg.q 
        self.joint_vel[0,1] = msg.dq
    
    def FRThighCallback(self, msg):
        self.joint_pos[1,1] = msg.q 
        self.joint_vel[1,1] = msg.dq 

    def FRCalfCallback(self, msg):
        self.joint_pos[2,1] = msg.q 
        self.joint_vel[2,1] = msg.dq

    def RLHipCallback(self, msg):
        self.joint_pos[0,2] = msg.q 
        self.joint_vel[0,2] = msg.dq 
    
    def RLThighCallback(self, msg):
        self.joint_pos[1,2] = msg.q 
        self.joint_vel[1,2] = msg.dq 

    def RLCalfCallback(self, msg):
        self.joint_pos[2,2] = msg.q 
        self.joint_vel[2,2] = msg.dq 

    def RRHipCallback(self, msg):
        self.joint_pos[0,3] = msg.q 
        self.joint_vel[0,3] = msg.dq 
    
    def RRThighCallback(self, msg):
        self.joint_pos[1,3] = msg.q 
        self.joint_vel[1,3] = msg.dq 

    def RRCalfCallback(self, msg):
        self.joint_pos[2,3] = msg.q 
        self.joint_vel[2,3] = msg.dq 
        
if __name__ == "__main__": 

    position_d = np.array([[0.0, 0.67, -1.3], [-0.0, 0.67, -1.3], 
                          [0.0, 0.67, -1.3],  [-0.0, 0.67, -1.3]]).T
    abd = 0.337
    shift = -0.09
    hip = 1.3
    thigh = 2.5
    position_d = np.array([[abd, hip + shift, -thigh], [-abd, hip + shift, -thigh], 
                          [abd, hip - shift, -thigh],  [-abd, hip - shift, -thigh]]).T
    position_target = np.copy(position_d)

    rospy.init_node("model_reset")
    modelStatePub = ModelPublisher()
    rate = rospy.Rate(100)
    rospy.sleep(0.1)
    t_start = rospy.get_time() 
    while not rospy.is_shutdown():
        t_now = rospy.get_time()
        t_elapsed = t_now - t_start

        if t_elapsed < 0.5:
            position_target[0,:] = modelStatePub.joint_pos[0,:]
            position_target[2,:] = -2.5
            # position_target[1,:] = -0.8
        elif t_elapsed < 1:
            position_target[:,:] = position_d[:,:]
        else:
            break

        modelStatePub.servoUpdate(position_target)
        rate.sleep() 
