#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import sys
import cvwin
import cv2
import threading
import time
import numpy as np
import copy
from collections import OrderedDict
import yaml
import json

this = sys.modules[__name__]
this.config_path="/home/zonesion/catkin_ws/src/marm_controller/config/config.yaml"
this.dir_f = os.path.abspath(os.path.dirname(__file__))
this.c_dir = os.path.split(os.path.realpath(__file__))[0]
with open(this.config_path, "r") as f:
    config = yaml.load(f.read())

MODELS=1    # 0:使用 tensors_flow pb模型文件 1:使用 yolov5_ncnn 模型

class AiCamera(object):
    def __init__(self):
        if MODELS==0:
            from obj_detection_rk3399 import detection
            self.objdetect=detection.ObjDetect("pill_detection_20220426",['box1', 'box2', 'box3', 'box4'])
        elif MODELS==1:
            import woodenmedicinedet
            self.objdetect=woodenmedicinedet.WoodenMedicineDet()  
            self.objdetect.init(this.c_dir+'/models/wooden_medicine') 
        self.window_name='camera'
        self.open_wins=[]

        #检测摄像头
        cam=self.__camera_check__()
        if cam!=-1:
            self.cap = cv2.VideoCapture(cam)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            print("set cam number %d"%cam)

        
    def __camera_check__(self):
        if os.path.exists("/dev/video0"):
            return 0
        if os.path.exists("/dev/video5"):
            return 4
        return -1

    def __undistort(self,src):   #矫正
        DIM=(640, 480)
        K=np.array([[361.6681963247486, 0.0, 331.640979254225], [0.0, 361.1945327740211, 224.49449156302728], [0.0, 0.0, 1.0]])
        D=np.array([[-0.04216543964788291], [0.15543013098889183], [-0.40349493136105163], [0.3373959977368023]])
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
        img = cv2.remap(src, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        return img

    def __win_is_open(self,name):
        for i in self.open_wins:
            if i==name:
                return True
        return False

    def __open_win(self,img):
        if self.__win_is_open(self.window_name)==False:    
            #Canny_Threshold
            self.open_wins.append(self.window_name)
        cvwin.imshow("camera",img)

    def close_win(self):
        if self.__win_is_open(self.window_name)==True:
            cvwin.destroyWindow(self.window_name)
            self.open_wins.remove(self.window_name)
    
    def __check(self,a,b,err=5):
        if type(a)==str and type(b)==str :          #字符串
            if a==b:                        
                return True
            else:
                return False
        if type(a)==int and type(b)==int :          #坐标
            if abs(a-b)<err:
                return True
            else:
                return False

    def list_add(self,lst1,lst2):
        sum_lst = [x + y for x, y in zip(lst1, lst2)]
        return sum_lst

    def pill_detect(self,timeout=20):
        st=time.time()
        __la_rect=[]
        __la_types=[]
        clear_frame=0
        while time.time()-st<timeout:
            success, frame = self.cap.read()
            if not success:
                time.sleep(1)
                print("no camera detect,please check whether the camera is connected normally")
                continue
            if clear_frame<5:                       #清除opencv图像缓存
                clear_frame+=1
                continue
            # frame=self.__undistort(frame)
            if MODELS==0: 
                img, rect, types, pp =self.objdetect.detect(frame)
                if rect:
                    print(rect,types)
                    print(type(types[0]))
                    print(pp,type(pp[0]))
            elif MODELS==1:
                rect = []
                types = []
                pp = []
                result=self.objdetect.detect(frame)
                result=json.loads(result)
                # 遍历所有结果并 兼容 MODELS=0格式
                num=result["result"]["obj_num"]
                for index in range(num):
                    location=result["result"]["obj_list"][index]["location"]
                    pos1=(int(location["left"]),int(location["top"]))
                    pos2=(int(location["left"]+location["width"]),int(location["top"]+location["height"]))
                    name=result["result"]["obj_list"][index]["name"].encode('utf-8')   #python2
                    # print(name,type(name))
                    score=result["result"]["obj_list"][index]["score"]
                    str=name+' %0.2f'%score
                    cv2.rectangle(frame,pos1,pos2,(0,0,255),thickness=2)
                    cv2.putText(frame,str,(pos1[0],pos1[1]-5), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),thickness=2)
                    loc=(pos1[0],pos1[1],pos2[0],pos2[1])
                    rect.append(loc)
                    types.append(name)
                    pp.append(score)
                img=frame                               # 图像中包含所有结果
            if(rect):
                # print(rect[0],types[0],pp[0])         # 取第一个识别目标
                if(__la_rect==[]):
                    __la_rect = [rect[0]] 
                    __la_types = [types[0]] 
                if self.__check(rect[0][0],__la_rect[-1][0])==True and \
                self.__check(rect[0][1],__la_rect[-1][1])==True and \
                self.__check(types[0],__la_types[-1])==True :
                    __la_rect.append(rect[0])     
                    __la_types.append(types[0])  
                else:
                    __la_rect = [rect[0]]
                    __la_types = [types[0]]          #重新取数据
                
                if(len(__la_rect)>=5):
                    # print("__la_rect",__la_rect)
                    rect=[]
                    for i in __la_rect:
                        if(rect==[]):
                            rect=i
                        else:
                            rect=self.list_add(i,rect)
                    rect=[int(x/len(__la_rect)) for x in rect]
                    return True,rect,types
            self.__open_win(img)
        self.close_win()
        return False,0,0

if __name__ == '__main__':
    aicamer=AiCamera()
    while True:
        sta,rect,types=aicamer.pill_detect(20)
        print(sta,rect,types)
        time.sleep(3)

