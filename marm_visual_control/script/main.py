#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import time
import copy 
import sys   
import signal
import cvwin
import threading
import math
from geometry_msgs.msg import PoseStamped, Pose
import yaml
this = sys.modules[__name__]

this.config_path="/home/zonesion/catkin_ws/src/marm_visual_control/config/config.yaml"
with open(config_path, "r") as f:
    config = yaml.load(f.read())

#颜色识别参数
this.color_param=config['color_param']
this.bin_param=config['bin_param']
g_open=config["g_open"]

#定位板相关参数
loc_plate=[141,192,465,386]         #初始值，机械臂自己查找 单位像素 不用设置
loc_plate_act=[0.153,0.173,0.092]   #定位板实际长度   单位 m 定位板框实际长度 上底 下底 高[0.155,0.173,0.093]\
loc_plate_act_origin=[0.22,0.014]   #定位板原点偏移   单位 m
loc_x_off_mx=20                     
loc_x_off_mi=0
loc_y_off_mx=0
loc_opt_par_x=1.0                   #X方向的优化参数
loc_opt_par_y=1.1                   #Y方向的优化参数
'''
0.2713 0.0192 0.25
loc_plate            单位 pix 640*480像素 定位板框 左上角x,y 和 右下角x,y
loc_plate_act        单位 m 定位板框实际长度 上底 下底 高[0.147,0.182,0.082]
loc_x_off_mx         单位 pix 相机倾斜视角X像素偏移 最大值
loc_y_off_mx         单位 pix 相机倾斜视角y像素偏移 最大值
loc_plate_origin     单位 pix cv2下像素原点
loc_plate_act_origin 单位 m  实际定位原点 相对base_link
'''
#机械臂
arm_g_height=0.15
#拍照位
# arm_cam_joint=[0.11535425216048287, -0.616704188215822, 1.40664924293023, 1.8002512210459807, 0.17097973936214012]
# arm_cam_joint=[0.1691319535249742, -0.36728181721768693, 1.164085808834055, 1.7663305763447448, 0.16]  #新鱼眼
arm_cam_joint=[0.1485716895448097, -0.23415630833973927, 1.0167376947524824, 1.7713593460377164, 0.14]  #旧鱼眼
#过渡位
arm_trans_joint=[1.3780610085141091e-05, 0.005306861269549153, 0.97426785523231, 1.3754647497671328, 5.618289536439448e-05]
#放料位
place_green_pre_pos=[-1.2662833162638238, 0.11783865004876276, 1.2993789823553925, 1.059983375965268, 0.27318028818813767]
place_green_pos=[-1.2506586271897269, 0.5286612702579147, 1.0776951416698373, 1.1076528226707714, 0.29343878626744674]
place_red_pre_pos=[-1.5279399601116417, 0.25960003900112727, 0.9214780698913716, 1.3986196039714034, 0.029174379224971552]
place_red_pos=[-1.4986224785330078, 0.8031212209534312, 0.5701763287046037, 1.520364072366262, 0.05670685598282597]
place_blue_pre_pos=[-1.0999189341441404, -0.08111371431671095, 1.4254253875468972, 1.2021911652352864, 0.4052235701070859]
place_blue_pos=[-1.0998658937065855, 0.4593649212523669, 1.1337912628982774, 1.2734753873708295, 0.42431170611445806]
place_yellow_pre_pos=[-1.478353233477545, 0.08287938126816825, 1.2421101286403857, 1.406994964743598, 0.08540515984785084]
place_yellow_pos=[-1.4663658780420075, 0.12841546305913984, 1.742980531732989, 0.8600552946565877, 0.0963037158430255]

def quit(signum, frame):
    print('EXIT APP') 
    sys.exit()
    rospy.signal_shutdown("arm is stopping")            #发出机械臂停止运动信号

from arm import Arm
from camera import AiCamera

class AiArm(Arm,AiCamera):
    def __init__(self,g_open,color,win=[],loc_plate=[141,192,465,386],loc_plate_act=[0.147,0.173,0.092] ,
    loc_plate_act_origin=[0,0],loc_x_off_mx=25,loc_x_off_mi=9,loc_y_off_mx=15,color_par=None,bin_param=None,):
        super(AiArm,self).__init__(g_open,xarm="xarm")
        super(Arm,self).__init__(color,win,loc_plate,loc_plate_act,loc_plate_act_origin,
        loc_x_off_mx,loc_x_off_mi,loc_y_off_mx,color_par,bin_param)

        self.block_pos=[]
        self.solutions=[]
        t = threading.Thread(target=self.__get_solutions)       #获取机械臂抓取位姿的线程
        t.setDaemon(True)
        t.start()
    
    def __get_solutions(self):
        while True:
            while len(self.block_pos)>0:                        #不断的进行解算机械臂的抓取位姿
                point=self.block_pos[0]
                Object_pose=Pose()
                Object_pose.position.x=point[1]*loc_opt_par_x
                Object_pose.position.y=point[2]*loc_opt_par_y     
                Object_pose.position.z=arm_g_height
                Object_pose.orientation.x=0
                Object_pose.orientation.y=0
                Object_pose.orientation.z=0
                Object_pose.orientation.w=1
                response = self.Solutions_client(Object_pose)  
                if  len(response.ik_solutions[0].positions)>0:
                    pos=[point[0],response.ik_solutions[1].positions,response.ik_solutions[0].positions,point[3]]
                    self.solutions.append(pos)                  #将解算结果进行缓存
                    del self.block_pos[0]
                else:
                    __msg=point[0]+" block is go fail"
                    rospy.logerr(__msg)
                    del self.block_pos[0]
            time.sleep(1) 
        
def aiarmAPP():
    # win=[98,420,103,560]      #选择剪裁窗口
    win = []
    aiarm=AiArm(g_open,("red","yellow","blue","green"),win,loc_plate,loc_plate_act,loc_plate_act_origin,loc_x_off_mx,loc_x_off_mi,
    loc_y_off_mx,this.color_param,this.bin_param)               #初始化AIarm，带有四种颜色识别模块
    aiarm.all_gohome()
    while not rospy.is_shutdown():  
        if aiarm.current_pos!='camera':
            aiarm.set_joint_value_target(arm_cam_joint)         #移动到相机位
            aiarm.current_pos='camera'
            rospy.sleep(4)                                      #消除抖动
        aiarm.plate_to_base()                                   #识别物块的位置
        for i in aiarm.success_tag:                             #根据识别成功的颜色进行处理
            if aiarm.rec_cla_dict[i]["pos"]:
                for m in aiarm.rec_cla_dict[i]["pos"]:
                    point=m
                    point.insert(0,i)
                    aiarm.block_pos.append(point)               #添加并进行机械臂抓取位姿查询
        while len(aiarm.block_pos)>0 or len(aiarm.solutions)>0:
            while len(aiarm.solutions)>0:
                solution=aiarm.solutions[0]
                # print(solution)
                __msg="Grabbing "+solution[0]+" wood" 
                rospy.loginfo(__msg)
                joint_positions = solution[1]
                aiarm.set_joint_value_target(joint_positions)   #移动到预抓取位
                aiarm.current_pos='pick'
                rospy.sleep(0.1)
                if solution[3]>45:                              #物体角度大于45度，反向抓取
                    solution[3]=90-solution[3]
                    aiarm.rotate_gripper(solution[3],arm_cam_joint[4])
                else:
                    aiarm.rotate_gripper(-solution[3],arm_cam_joint[4]) #正向抓取，arm_cam_joint[4]为夹具偏移量
                joint_positions = solution[2]
                aiarm.set_arm_joint_value_target(joint_positions)#移动到抓取位
                rospy.sleep(0.1)
                aiarm.setGripper(True)
                rospy.sleep(0.1)
                joint=aiarm.get_joints()
                arm_trans_joint[0]=joint[0]                     #不旋转1号舵机抬起
                aiarm.set_joint_value_target(arm_trans_joint)
                rospy.sleep(0.1)
                if solution[0]=='red':                          #前方红色放料区
                    aiarm.set_joint_value_target(place_red_pre_pos)
                    rospy.sleep(0.1)
                    aiarm.set_joint_value_target(place_red_pos)
                    rospy.sleep(0.1)
                    aiarm.setGripper(False) 
                    rospy.sleep(0.1)
                    aiarm.set_joint_value_target(place_red_pre_pos)
                    rospy.sleep(0.1)
                elif solution[0]=='blue':                       #前方蓝色放料区
                    aiarm.set_joint_value_target(place_blue_pre_pos)
                    rospy.sleep(0.1)
                    aiarm.set_joint_value_target(place_blue_pos)
                    rospy.sleep(0.1)
                    aiarm.setGripper(False) 
                    rospy.sleep(0.1)
                    aiarm.set_joint_value_target(place_blue_pre_pos)
                    rospy.sleep(0.1)
                elif solution[0]=='yellow':                     #前方黄色放料区
                    aiarm.set_joint_value_target(place_yellow_pre_pos)
                    rospy.sleep(0.1)
                    aiarm.set_joint_value_target(place_yellow_pos)
                    rospy.sleep(0.1)
                    aiarm.setGripper(False) 
                    rospy.sleep(0.1)
                    aiarm.set_joint_value_target(place_yellow_pre_pos)
                    rospy.sleep(0.1)
                elif solution[0]=='green':                      #前方绿色放料区
                    aiarm.set_joint_value_target(place_green_pre_pos)
                    rospy.sleep(0.1)
                    aiarm.set_joint_value_target(place_green_pos)
                    rospy.sleep(0.1)
                    aiarm.setGripper(False) 
                    rospy.sleep(0.1)
                    aiarm.set_joint_value_target(place_green_pre_pos)
                    rospy.sleep(0.1)  
                aiarm.all_gohome() 
                rospy.sleep(0.1)
                del aiarm.solutions[0]
            rospy.sleep(0.1)
    aiarm.all_gohome()
    rospy.sleep(0.1)
    print('EXIT APP') 
    aiarm.shutdown()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, quit)                                
    signal.signal(signal.SIGTERM, quit)
    rospy.init_node("AIARM_NODE", log_level=rospy.INFO)     #初始化节点
    aiarmAPP()                  