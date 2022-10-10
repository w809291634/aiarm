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
from obj_detection_rk3399 import detection

this = sys.modules[__name__]
this.config_path="/home/zonesion/catkin_ws/src/marm_controller/config/config.yaml"

this.dir_f = os.path.abspath(os.path.dirname(__file__))

with open(config_path, "r") as f:
    config = yaml.load(f.read())

class AiCamera(object):
    def __init__(self, model_name, class_name):
        self.objdetect=detection.ObjDetect(model_name,class_name)
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
            else:
                return False
        return False

    def __open_win(self,img):
        if self.__win_is_open(self.window_name)==False:    
            #Canny_Threshold
            self.open_wins.append(self.window_name)
        cvwin.imshow("camera",img)

    def __close_win(self):
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
            frame=self.__undistort(frame)
            img, rect, types, pp =self.objdetect.detect(frame)
            if(rect):
                # print(rect[0],types[0],pp[0])      #取第一个识别目标
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
                    self.__close_win()
                    return True,rect,types
            self.__open_win(img)
        self.__close_win()
        return False,0,0

if __name__ == '__main__':
    obj_class_names = ['box1', 'box2', 'box3', 'box4']
    aicamer=AiCamera("pill_detection_20220426",obj_class_names)
    while True:
        sta,rect,types=aicamer.pill_detect(20)
        print(sta,rect,types)
        time.sleep(1)

