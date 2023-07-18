#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose
import sys,math,threading
from geometry_msgs.msg import PoseStamped, Pose
import time
import tf
from std_msgs.msg import Int32,Int16,Int32MultiArray
from copy import deepcopy
import math
this = sys.modules[__name__]
import sys   
import signal
from moveit_simple_grasps.srv import GenerateSolutions
from marm_controller.srv import *
#机械臂调试
arm_debug=False #True False

import yaml
import sys
this = sys.modules[__name__]

this.config_path="/home/zonesion/catkin_ws/src/marm_controller/config/config.yaml"
with open(config_path, "r") as f:
    config = yaml.load(f.read())

grip_query_time=config["grip_query_time"]       #机械臂夹具查询关节位置周期
grip_wait_time=config["grip_wait_time"]         #机械臂夹具运动超时时间系数
gripper_arrive_err=config["gripper_arrive_err"] 
def quit(signum, frame):
    print ''
    print 'EXIT APP'
    sys.exit()

this.arm_joint=[]
this.arm_res=0
class Arm(object):
    # xarm参数：为varm时：由虚拟机控制3399，3399再控制机械臂和夹具
    #           为xarm时：3399直接控制机械臂
    def __init__(self,g_open,xarm="varm",gripper_ty=True):
        self.arm = MoveGroupCommander("manipulator")
        self.xarm=xarm
        if self.xarm=="varm":
            try:
                rospy.wait_for_service('/arm_controller/gripper',timeout=5)
            except (rospy.ServiceException, rospy.ROSException ,Exception) as e:
                print(e)
                sys.exit()
            self.gripper  = rospy.ServiceProxy('/arm_controller/gripper', gripper)

        elif self.xarm=="xarm":
            self.gripper = rospy.Publisher('/xcar/gripper', Int32, queue_size=0, latch=True)
            self.arm_status_pub=rospy.Publisher('/xcar/arm_status_req', Int32, queue_size=0, latch=True)
            def __arm_joint(msg):           #用于查询机械臂的关节状态，这里主要用于查询夹具
                this.arm_joint=msg.data
                this.arm_res=1
            rospy.Subscriber('/xcar/arm_status', Int32MultiArray, __arm_joint)

        self.Solutions_client = rospy.ServiceProxy('/grasp_filter_test/GenerateSolutions', GenerateSolutions)   
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.001)
        self.arm.set_max_acceleration_scaling_factor(1)      #设置允许的最大速度和加速度
        self.arm.set_max_velocity_scaling_factor(1)

        if gripper_ty==True:
            self.gripper_open=g_open
            self.gripper_close=g_open-80
        else:
            self.gripper_open=g_open
            self.gripper_close=g_open+80   
        self.current_pos=''
        # self.all_gohome()
  
        if arm_debug ==True:
            t = threading.Thread(target=self.__arm_tool_pose)
            t.setDaemon(True)
            t.start()

    def __arm_tool_pose(self):
        while True:
            pose=self.getPose()
            print(pose)
            time.sleep(0.1)

    def all_gohome(self,wait=True):
        self.arm.set_named_target('home')                    #控制机械臂先回到初始化位置
        self.arm.go(wait)
        rospy.sleep(1)
        self.setGripper(False)             
        rospy.loginfo("Open the fixture")
        self.current_pos='all_home'

    def arm_goHome(self,wait=True):
        self.arm.set_named_target('home')
        self.arm.go(wait)
        self.current_pos="home"
    
    def arm_goStart(self,wait=True):
        self.arm.set_named_target('start')
        self.arm.go(wait)
        self.current_pos='start'

    def goPose_rpy(self,a, wait=True, tmout = 8):
        '''
        设置机械臂目的位置, 
        a: 长度为6的数组
            a[0],a[1],a[2], 分别为目的x，y,z 坐标
            a[3],a[4],a[5]， 为机械爪的姿态绕x,y，z轴的旋转
        '''
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"  # group.get_pose_reference_frame()
        #target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = a[0] #x
        target_pose.pose.position.y = a[1] #y
        target_pose.pose.position.z = a[2] #z
        # 自带吸盘的四元数
        q = tf.transformations.quaternion_from_euler(a[3], a[4], a[5]) #RPY

        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        self.arm.set_pose_target(target_pose, self.arm.get_end_effector_link())
        return self.arm.go(True)

    def goPose_qua(self,a, wait=True, tmout = 8):
        self.arm.set_goal_orientation_tolerance(0.01)
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"  
        #target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = a[0] #x
        target_pose.pose.position.y = a[1] #y
        target_pose.pose.position.z = a[2] #z

        target_pose.pose.orientation.x = a[3]
        target_pose.pose.orientation.y = a[4]
        target_pose.pose.orientation.z = a[5]
        target_pose.pose.orientation.w = a[6]
        self.arm.set_pose_target(target_pose, self.arm.get_end_effector_link())
        self.arm.go(True)
        self.arm.set_goal_orientation_tolerance(0.001)

    def goPosition(self,a):
        self.arm.set_position_target(a, self.arm.get_end_effector_link())
        return self.arm.go(True)

    def getPose(self):
        pose=self.arm.get_current_pose(self.arm.get_end_effector_link())
        pose=[pose.pose.position.x,pose.pose.position.y,pose.pose.position.z,
        pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,
        pose.pose.orientation.w
        ]
        return pose

    def getRpy(self):
        return self.arm.get_current_rpy(arm.get_end_effector_link())

    def get_joints(self):
        return self.arm.get_current_joint_values()

    def set_joint_value_target(self,joint_positions):
        self.arm.set_joint_value_target(joint_positions)
        self.arm.go(True)

    def set_arm_joint_value_target(self,joint_positions):
        joint=self.get_joints()
        joint_positions=list(joint_positions)
        joint_positions[4]=joint[4]
        self.arm.set_joint_value_target(joint_positions)
        self.arm.go(True)
        
    def setGripper(self,en):
        '''
            机械爪控制en  :True 抓取
                        :False 释放
        '''
        if self.xarm=="varm":
            if en:
                err=self.gripper(self.gripper_close)
                if err.result!=0:
                    print("gripper run error!")
            else:
                err=self.gripper(self.gripper_open)
                if err.result!=0:
                    print("gripper run error!")
        elif self.xarm=="xarm":
            def gripper_arrive(data1,data2,err=10):
                if abs(data1-data2)<err:
                    return True
                else:
                    return False   
            def __setGripper(_data):
                st1=time.time()
                while True:
                    self.gripper.publish(Int32(data=_data))            
                    this.arm_res=0
                    self.arm_status_pub.publish(Int32(1))
                    time.sleep(grip_query_time)
                    st=time.time()
                    while not this.arm_res:
                        time.sleep(0.02)
                        if time.time()-st>grip_query_time:          # 等待
                            print("no arm_res")
                            break 
                    if this.arm_res==1:
                        # print("g_1",_data)
                        # print("g_2",this.arm_joint[0])
                        if gripper_arrive(_data,this.arm_joint[0],gripper_arrive_err)==True:
                            print("gripper run success")
                            break
                    if time.time()-st1>grip_wait_time:               # 等待
                        print("gripper run error!")
                        break  
            if en:
                __setGripper(self.gripper_close)
            else:
                __setGripper(self.gripper_open)
    
    def rotate_gripper(self,angle,offset):  #在当前角度旋转angle度
        angle=math.radians(angle)+offset    #夹具旋转偏移，转换弧度
        joint=self.get_joints()
        joint[4]=joint[4]+angle             #当前位置加上旋转角度
        # print('joint[4]',joint[4])
        self.set_joint_value_target(joint)

    def shutdown(self):
        # Exit
        print "The arm is shutting down."
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

#拍照位
arm_cam_joint=[0.11535425216048287, -0.616704188215822, 1.40664924293023, 1.8002512210459807, 0.17097973936214012]
#放料位
place_orientation=[0.4082922511955792, 0.3883747732255148, -0.5864623925036405, 0.5818284414796017]
green_box_pos=[0.104056638643234, -0.15154166501230143, 0.1610305493810666]
arm_place_joint=[-1.5401102540326523, 0.2780434008584921, 1.5998078696019935, 1.0356559126401998, 0.07418549237673772]

if __name__ == '__main__':
    signal.signal(signal.SIGINT, quit)                                
    signal.signal(signal.SIGTERM, quit)
    rospy.init_node("arm_debug", log_level=rospy.INFO)
    arm=Arm(0)                    #定义arm，此时机械臂的夹具打开角度为-40
    while True:
        #获取机械臂当前位姿
        # __pose=arm.getPose()
        # print(__pose)
        # time.sleep(1)
        
        # joint=[-1.2506586271897269, 0.5286612702579147, 1.0776951416698373, 1.1076528226707714, 0.29343878626744674]
        # arm.set_joint_value_target(joint)
        #获取机械臂的当前关节
        joint=arm.get_joints()
        print(joint)
        time.sleep(1)

        # arm.arm_goHome()
        # time.sleep(1)
        # arm.goPose_qua(green_box_pos+place_orientation)
        # time.sleep(1)
        # green_box_pos[1]-0.025
        # arm.goPose_qua(green_box_pos+place_orientation)
        # time.sleep(1)
        # arm.rotate_gripper(0)

        # 测试夹具
        # arm.setGripper(1)
        # time.sleep(1)
        # arm.setGripper(0)
        # time.sleep(1)

    



