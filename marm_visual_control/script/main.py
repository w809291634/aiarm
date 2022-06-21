#!/usr/bin/python
# -*- coding: utf-8 -*-

##############################################################################################
# 文件：main.py
# 作者：Zonesion wanghao 20220412
# 说明：aiarm 主应用程序
# 修改：20220510  增加config文件统一存储参数
#       20220621  适配新的机械臂，修改具体点位
# 注释：
##############################################################################################
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
loc_plate_act_origin=[0.21,0.000]   #定位板原点偏移   单位 m
loc_x_off_mx=20                     
loc_x_off_mi=0
loc_y_off_mx=10
loc_opt_par_x=1.0                   #X方向的优化参数
loc_opt_par_y=1.0                   #Y方向的优化参数
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
arm_g_height=0.134
#拍照位
# arm_cam_joint=[0.1485716895448097, -0.23415630833973927, 1.0167376947524824, 1.7713593460377164, 0.14]  #旧鱼眼
arm_cam_joint=[-0.09012192015910722, -0.04428321319882354, 0.8573005610356529, 1.9794888436393088, -0.018]
#过渡位
arm_trans_joint=[1.3780610085141091e-05, 0.005306861269549153, 0.97426785523231, 1.3754647497671328, 5.618289536439448e-05]
#放料位
place_green_pre_pos=[-1.3265524118232723, 0.3655966425860558, 0.958677844890208, 1.3130885033703161, 0.26166659383462304]
place_green_pos=[-1.3448637572281599, 0.7148921367093759, 0.8301213995891474, 1.383962933474151, 0.2529428516342751]
place_red_pre_pos=[-1.569808929949258, 0.30870103957173256, 0.8855745835008368, 1.540034105658909, 0.033599209040843736]
place_red_pos=[-1.5681242281313106, 0.6839995863555675, 0.8778300129793662, 1.3738957859199925, 0.03458501462580535]
place_blue_pre_pos=[-1.271269903982289, 0.2946518684133466, 0.8931614029547874, 1.6595248994610352, 0.303024230637421]
place_blue_pos=[-1.286016086089582, 0.3698417631423453, 1.4507393370560877, 1.1048128607753822, 0.3105957893466142]
place_yellow_pre_pos=[-1.5616071144115444, 0.1852763916838321, 1.066325580111766, 1.6094755035417987, 0.02467022151749461]
place_yellow_pos=[-1.5579521399156477, 0.2752727085124672, 1.5744065335667794, 1.085966129981658, 0.04446556249672398]

def quit(signum, frame):
    print('EXIT APP') 
    sys.exit()
    # rospy.signal_shutdown("arm is stopping")                  #发出机械臂停止运动信号

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
                response = self.Solutions_client(Object_pose)   #向服务器查询机械臂最佳的抓取姿态
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
    # win=[98,420,103,560]                                      #选择剪裁窗口
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
                del aiarm.solutions[0]                          #该夹取执行完成，清除数据
            rospy.sleep(0.1)
    aiarm.all_gohome()
    rospy.sleep(0.1)
    print('EXIT APP') 
    aiarm.shutdown()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, quit)                                
    signal.signal(signal.SIGTERM, quit)
    rospy.init_node("AIARM_NODE", log_level=rospy.INFO)         #初始化节点
    aiarmAPP()                                                  #开始机械臂的物料筛选应用