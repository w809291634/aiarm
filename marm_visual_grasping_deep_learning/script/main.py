#!/usr/bin/python
# -*- coding: utf-8 -*-

##############################################################################################
# 文件：main.py
# 作者：Zonesion wanghao 20220412
# 说明：aiarm 
# 修改：20220510  增加config文件统一存储参数
#       20220621  适配新的机械臂，修改具体点位
# 注释：
##############################################################################################
import rospy
import time
import sys   
import signal
from geometry_msgs.msg import PoseStamped, Pose
import yaml
this = sys.modules[__name__]

##############################################################################################
# 公共参数配置文件
##############################################################################################
this.config_path="/home/zonesion/catkin_ws/src/marm_controller/config/config.yaml"
with open(config_path, "r") as f:
    config = yaml.load(f.read())

##############################################################################################
# 机械臂相关参数和位置点定义
##############################################################################################
#机械臂夹具打开角度
g_open=config["g_open"]
#拍照位
arm_cam_joint=[-1.500564611619939, -0.1672537316580661, 1.7541373748021383, 1.2458516378870481, 0.1416934401114232]
#过渡位
arm_trans_joint=[1.3780610085141091e-05, 0.005306861269549153, 0.97426785523231, 1.3754647497671328, 5.618289536439448e-05]
#放料位
place_yellow_pre_pos= [-1.5324442566485892, 0.41451335449088383, 0.9375988140481452, 1.6653015644802336, 0.10932592647823254]
place_yellow_pos= [-1.5565545424472809, 0.5058198364510939, 1.3704384523665245, 1.1413851074988943, 0.0854840684136222]
#分拣区
place_Sorting_area= [0.0021030099132403734, 0.33394815604215156, 1.399292175553113, 0.986600701497403, 0.002012568299220012]
#相机检测时间
camera_det_time=20

def quit(signum, frame):
    print('EXIT APP') 
    sys.exit()
    # rospy.signal_shutdown("arm is stopping")                  #发出机械臂停止运动信号

from arm import Arm
from camera import AiCamera

class AiArm(Arm,AiCamera):
    def __init__(self,g_open):
        super(AiArm,self).__init__(g_open,xarm="xarm")          #定义为在arm（3399）端运行此程序。初始化Arm类,定义为"varm"是在虚拟机远程控制
        super(Arm,self).__init__("pill_detection_20220426",['box1', 'box2', 'box3', 'box4'])#初始化AiCamera类
       
def aiarmAPP():
    aiarm=AiArm(g_open)                                         #初始化AIarm
    aiarm.all_gohome()                                          #机械臂和夹具都回原位
    while not rospy.is_shutdown():  
        if aiarm.current_pos!='camera':
            aiarm.set_joint_value_target(arm_cam_joint)         #移动到相机位
            aiarm.current_pos='camera'
            rospy.sleep(2)                                      #消除抖动
        
        sta,rect,type=aiarm.pill_detect(camera_det_time)
        if sta:
            rospy.logwarn('Target Detected:%s'%type) 
            #开始抓取
            def go_warehouse(pre_pos,pos):                      #前往仓库区
                aiarm.set_joint_value_target(pre_pos)
                rospy.sleep(0.1)
                aiarm.set_joint_value_target(pos)
                rospy.sleep(0.1)
                aiarm.setGripper(True) 
                rospy.sleep(0.1)
                aiarm.set_joint_value_target(pre_pos)
                rospy.sleep(0.1)
                #前方黄色放料区取料
            go_warehouse(place_yellow_pre_pos,place_yellow_pos)
            aiarm.arm_goHome()
            aiarm.close_win()
            rospy.sleep(0.1)
            aiarm.set_joint_value_target(place_Sorting_area)
            rospy.sleep(0.1)
            aiarm.setGripper(False) 
            rospy.sleep(0.1)
            aiarm.all_gohome() 
            rospy.sleep(0.1)
        else:
            rospy.logerr('target detecting timeout,the time is %ds'%camera_det_time) 
        time.sleep(1)
    aiarm.all_gohome()
    rospy.sleep(0.1)
    rospy.logwarn('EXIT APP') 
    aiarm.shutdown()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, quit)                          
    signal.signal(signal.SIGTERM, quit)
    rospy.init_node("AIARM_NODE", log_level=rospy.INFO)         #初始化节点
    aiarmAPP()                                                  #开始机械臂的物料筛选应用