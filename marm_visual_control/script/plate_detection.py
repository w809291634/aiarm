#!/usr/bin/env python
# -*- encoding: utf-8 -*-

##############################################################################################
# 文件：plate_detection.py
# 作者：Zonesion wanghao 20220412
# 说明：aiarm 定位框识别程序
# 修改：20220510  发布初始版本
# 注释：
##############################################################################################
import cv2
import numpy as np
import os
import copy
import time
import cvwin
import math
from collections import OrderedDict
from color_detection import Color_Rec
import yaml
import sys
this = sys.modules[__name__]
this.config_path="/home/zonesion/catkin_ws/src/marm_visual_control/config/config.yaml"
with open(config_path, "r") as f:
    config = yaml.load(f.read())
c_dir = os.path.split(os.path.realpath(__file__))[0]

def nothing(self,*arg):
    pass

this.param=config['bin_param']
this.palte_color_param=config['palte_color_param']

class Plate_det(object):
    def __init__(self,win=[],win_show=False,winmain_show=True,par=None,undistort=True):
        self.win_par=win
        self.name="Plate"
        self.window_name=self.name+"det"
        self.open_wins=[]
        self.win_show=win_show
        self.winmain_show=winmain_show
        self.undistort=undistort
        self.plate_cons=[]
        self.slope1=None
        self.slope2=None
        if par==None:
            self.Bin_Thr1=param["Binary"][0]
            self.Bin_Thr2=param["Binary"][1]
        else:
            self.Bin_Thr1=par["Binary"][0]
            self.Bin_Thr2=par["Binary"][1]
        self.get_template()                                             # 获取并计算标准模板中的数据
        # 加载颜色识别模组
        self.rec_cla_dict=OrderedDict()                                 # 颜色识别类,含多个
        for i in this.palte_color_param.keys():                         # 定位板识别配置参数中带有几类，就初始化几类
            det={
                "class":Color_Rec(i,win,win_show=False,winmain_show=False,color_par=this.palte_color_param,comment='plate_'),
                "pos":None
            }
            self.rec_cla_dict[i]=copy.deepcopy(det)

    def __win_is_open(self,name):
        for i in self.open_wins:
            if i==name:
                return True
            else:
                return False
        return False

    def open_win(self):
        if self.winmain_show==True and self.__win_is_open(self.window_name)==False:
            #Binary_Threshold
            cvwin.namedWindow(self.window_name)
            cvwin.createTrackbar('Bin_threshold1',self.window_name,self.Bin_Thr1, 300 , nothing)
            cvwin.createTrackbar('Bin_threshold2',self.window_name,self.Bin_Thr2, 300 , nothing)
            #Canny_Threshold
            self.open_wins.append(self.window_name)

    def __undistort(self,src):   #矫正
        DIM=(640, 480)
        K=np.array([[361.6681963247486, 0.0, 331.640979254225], [0.0, 361.1945327740211, 224.49449156302728], [0.0, 0.0, 1.0]])
        D=np.array([[-0.04216543964788291], [0.15543013098889183], [-0.40349493136105163], [0.3373959977368023]])
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
        __img = cv2.remap(src, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        return __img
    
    def __undistort_new(self,src):   #矫正
        DIM=(640, 480)
        K=np.array([[437.5491127380999, 0.0, 283.9999474900146], [0.0, 438.18423945586335, 252.71116475229556], [0.0, 0.0, 1.0]])
        D=np.array([[-0.06208478317715735], [-0.18283561976377807], [0.5460914487840476], [-0.5218727344866557]])
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
        __img = cv2.remap(src, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        return __img

    def prepare_template(self,img):             
        self.open_win()
        self.img=copy.deepcopy(img)
        if self.undistort==True:
            self.img=self.__undistort(self.img)
        if len(self.win_par)!=0 :
            self.img = self.img[self.win_par[0]:self.win_par[1], self.win_par[2]:self.win_par[3]] #裁剪图像

        if self.winmain_show==True:
            self.Bin_Thr1 = cvwin.getTrackbarPos('Bin_threshold1',self.window_name)
            self.Bin_Thr2 = cvwin.getTrackbarPos('Bin_threshold2',self.window_name)

        self.Gauss = cv2.GaussianBlur(self.img, (5, 5), 0)          #高斯滤波

        self.gray = cv2.cvtColor(self.Gauss,cv2.COLOR_BGR2GRAY)    #图像灰度化
        ret, self.Binary_image = cv2.threshold(self.gray ,self.Bin_Thr1 ,self.Bin_Thr2 ,cv2.THRESH_BINARY_INV)  #图像二值化
        self.dst = cv2.medianBlur(self.Binary_image, 5)              #滤波消除噪点，第二参数为奇数，越大消除的噪点越大
        if self.win_show!=False:
            cvwin.imshow(self.name+'Binary',self.dst)

    def get_template(self,debug=False):
        # 载入标准模板图
        if debug==True:
            self.open_win()
        __img = cv2.imread(c_dir+'/'+config['template']+'.png', 0)
        _,th = cv2.threshold(__img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        _,tem_contours, hierarchy1 = cv2.findContours(th, cv2.RETR_EXTERNAL, 3)
        if debug==True:    
            cv2.drawContours(__img, tem_contours[0], -1, (255, 255, 0), 1)   #画轮廓cv
            cvwin.imshow(self.window_name,__img)
        self.template= tem_contours[0]
        __point_ls=[]
        for i in self.template:
            __point_ls.append([i[0][0],i[0][1]])
        _p1,_p2,_p3,_p4=self.__find_point(__point_ls,check=False)   # P1在P2上，P3在P4上
        self.slope1=math.atan2((_p2[1]-_p1[1]), (_p2[0]-_p1[0]))    # 根据模板获取对角点的斜率
        self.slope2=math.atan2((_p4[1]-_p3[1]), (_p4[0]-_p3[0])) 
        if debug==True:
            self.__draw_all_point(__img,_p1,_p2,_p3,_p4)
            cvwin.imshow(self.window_name,__img)
        
    def __draw_all_point(self,img,p1,p2,p3,p4):
        if self.winmain_show!=False :
            if p1!=None:
                self.__drawpoint(img,p1)   #画点
            if p2!=None:
                self.__drawpoint(img,p2)   #画点
            if p3!=None:
                self.__drawpoint(img,p3)   #画点
            if p4!=None:
                self.__drawpoint(img,p4)   #画点

    def __dis(self,m,n):
        dis = math.sqrt((abs(m[0] - n[0]))**2 + (abs(m[1] - n[1]))**2)
        return dis

    def __drawpoint(self,img,point):
        point=(point[0],point[1])
        cv2.circle(img,point, 0, (255, 255, 0), 3)   #画点

    def __pre_point(self,point1,point2):
        if point1[1]<point2[1]:  #P1在P2上面
            __point1=point1
            __point2=point2
        else:
            __point2=point1
            __point1=point2
        return __point1,__point2

    def __check_point(self,point1,point2,angle1,angle2,error=0.03):
        '''
        点列表
        '''
        __angle=math.atan2((point2[1]-point1[1]), (point2[0]-point1[0]))  
        if abs(__angle-angle1)<error or abs(__angle-angle2)<error :
            return True
        else:
            return False

    def __find_point(self,point_ls,check=False,err=0.03):
        '''
        point_ls 轮廓点列表
        check 开启检查角度
        轮廓点列表 point_ls  [[x,y,x1,y1,dis_mx],[x,y,x1,y1,dis_mx]...]
        点对应的最大距离列表 ls_dis_mx 
        '''
        ls_dis_mx=[]
        for m in range(len(point_ls)):          # 取每点距离
            st_point=point_ls[m]                # 取一个距离计算起始点
            mx_dis=0
            mx_n=0
            for n in range(len(point_ls)):      
                en_point=point_ls[n]            # 取一个距离计算终点
                dis=self.__dis(st_point,en_point)# 计算距离
                if dis>mx_dis:
                    mx_dis= dis
                    mx_n=n
                # print(mx_dis,mx_n)
            ls_dis_mx.append(mx_dis)            # 将该点最大距离值保存起来，用于后面计算
            point_ls[m].append(point_ls[mx_n][0])
            point_ls[m].append(point_ls[mx_n][1])
            point_ls[m].append(mx_dis)          # 将该点、最大距离点、距离绑定

        __p1=__p2=__p3=__p4=None
        #取点1和点2
        if len(ls_dis_mx)>0 and len(point_ls)>0:
            ls_dis_mx.sort(reverse = True)      # 最大值排序
            m=init=0
            while len(ls_dis_mx)>0:
                if init!=0 and m<len(ls_dis_mx):
                    m+=1
                if m>=len(ls_dis_mx):
                    break
                for n in range(len(point_ls)):
                    if ls_dis_mx[m]==point_ls[n][4]:                            # 找到最大距离的两点
                        point1=[point_ls[n][0],point_ls[n][1]]
                        point2=[point_ls[n][2],point_ls[n][3]]
                        __point1,__point2=self.__pre_point(point1,point2)       # 将点1放在点2上方
                        if check==True:
                            if self.slope1!=None and self.slope2!=None:
                                if self.__check_point(__point1,__point2,self.slope1,self.slope2,err)!=True: # 检查是否满足斜率要求
                                    ls_dis_mx.remove(ls_dis_mx[m])
                                    m=0
                                    init=0
                                    if len(ls_dis_mx)>0:
                                        continue
                                    else:
                                        break
                            else:
                                return -1
                        if __point1[0]<__point2[0]:                             # 点1和点2确定了极性，保证和点3、点4是对角线
                            polar=True
                        else:
                            polar=False
                        __p1=__point1
                        __p2=__point2
                        point_ls.remove(point_ls[n])
                        break
                if  __p1!=None and __p2!=None:
                    break

        #取点3和点4
        if len(ls_dis_mx)>0 and len(point_ls)>0:
            m=init=0
            while len(ls_dis_mx)>0:
                if init!=0 and m<len(ls_dis_mx):
                    m+=1
                if m>=len(ls_dis_mx):
                    break
                init=1
                for n in range(len(point_ls)):
                    if ls_dis_mx[m]==point_ls[n][4]:                            # 找到最大距离的两点
                        point3=[point_ls[n][0],point_ls[n][1]]
                        point4=[point_ls[n][2],point_ls[n][3]]
                        __point3,__point4=self.__pre_point(point3,point4)       # 将点3放在点4上方
                        if check==True:
                            if self.slope1!=None and self.slope2!=None:
                                if self.__check_point(__point3,__point4,self.slope1,self.slope2,err)!=True: # 检查是否满足斜率要求
                                    ls_dis_mx.remove(ls_dis_mx[m])              # 删除这个最长的距离，并重新开始查找
                                    m=0
                                    init=0
                                    if len(ls_dis_mx)>0:
                                        continue
                                    else:
                                        break
                            else:
                                return -1
                        if polar==True:                                         # 由于点1和点2已经找到，这里根据极性找点3和4
                            if __point3[0]>__point4[0]:
                                __p3=__point3
                                __p4=__point4
                                point_ls.remove(point_ls[n])
                                break
                        else:
                            if __point3[0]<__point4[0]:
                                __p3=__point3
                                __p4=__point4
                                point_ls.remove(point_ls[n])
                                break
                if  __p3!=None and __p4!=None:
                    break
        return __p1,__p2,__p3,__p4

    def __reverse_color(self,img):                  # 将定位框内木块取反颜色，为了定位框识别准确
        for i in self.rec_cla_dict.keys():
            self.rec_cla_dict[i]["class"].plate_color_det(img)
            canny = cv2.Canny(self.rec_cla_dict[i]["class"].mask, 80, 300, apertureSize = 3)          #检测边缘
            contours_img  ,contours ,hierarchy = cv2.findContours(canny ,cv2.RETR_EXTERNAL ,3) #查找轮廓
            # ls_len=[]
            # for i in range(len(contours)):     
            #     ls_len.append(len(contours[i]))
            # if ls_len:
            #     idx=ls_len.index(max(ls_len))   #取最长轮廓
            #     cv2.fillConvexPoly(img, contours[idx], (0,0,0))

            for i in contours:
                # rect = cv2.minAreaRect(i)      
                # box = cv2.boxPoints(rect)
                # box = np.int0(box)
                # cv2.drawContours(img, [box], 0, (0, 0, 0), thickness = cv2.FILLED)               #画旋转矩形
                cv2.fillConvexPoly(img, i, (0,0,0))
        return img

    def plate_det(self,img,min_value = 0.1,check=True,err=0.03):
        min_value=config['matchShapes_value']
        self.open_win()
        self.img=copy.deepcopy(img)
        if self.undistort==True:
            self.img=self.__undistort(self.img)
        if len(self.win_par)!=0 :
            self.img = self.img[self.win_par[0]:self.win_par[1], self.win_par[2]:self.win_par[3]] #裁剪图像

        if self.winmain_show==True:
            self.Bin_Thr1 = cvwin.getTrackbarPos('Bin_threshold1',self.window_name)
            self.Bin_Thr2 = cvwin.getTrackbarPos('Bin_threshold2',self.window_name)

        # 色块处理
        self.img1=copy.deepcopy(self.img)
        self.reverse=self.__reverse_color(self.img1)
        # if self.win_show!=False:
        #     cvwin.imshow(self.name+'reverse',self.reverse)

        self.Gauss = cv2.GaussianBlur(self.reverse, (5, 5), 0)                  #高斯滤波
        # if self.win_show!=False:
        #     cvwin.imshow(self.name+'Gauss',self.Gauss)
 
        self.gray = cv2.cvtColor(self.Gauss,cv2.COLOR_BGR2GRAY)             #图像灰度化
        ret, self.Binary_image = cv2.threshold(self.gray ,self.Bin_Thr1 ,self.Bin_Thr2 ,cv2.THRESH_BINARY_INV)  #图像二值化
        self.dst = cv2.medianBlur(self.Binary_image, 5)                     #滤波消除噪点，第二参数为奇数，越大消除的噪点越大
        if self.win_show!=False:
            cvwin.imshow(self.name+'Binary',self.dst)

        self.canny = cv2.Canny(self.dst, 80, 150, apertureSize = 3)          #检测边缘
        self.canny  ,self.contours ,self.hierarchy = cv2.findContours(self.canny ,cv2.RETR_EXTERNAL ,3) #查找轮廓.方法3点最少
        # print(self.contours)
        # if self.win_show!=False:
        #     cvwin.imshow(self.name+'canny',self.canny)
        '''
        self.contours: [array,array] ; array:([[[坐标点]],[[坐标点]],[[坐标点]],....,[[value]],type)
        self.plate_cons:[[self.contours[i],value],[self.contours[i],value],....]
        ls_len:轮廓长度列表
        '''
        self.plate_cons=[]
        # self.img_1=copy.deepcopy(self.img)
        ls_len=[]
        for i in range(len(self.contours)):     
            value = cv2.matchShapes(self.template,self.contours[i],cv2.CONTOURS_MATCH_I1,0)             # 将识别的所有轮廓和模板轮廓进行匹配
            if value < min_value:
                data=(self.contours[i],value)                   
                ls_len.append(len(self.contours[i]))            # 满足要求的轮廓长度列表
                self.plate_cons.append(copy.deepcopy(data))     # 存储这些轮廓

        if len(self.plate_cons)>0:
            idx=ls_len.index(max(ls_len))                       # 取最长轮廓的索引
            point_ls=[]
            for i in self.plate_cons[idx][0]:
                point_ls.append([int(i[0][0]),int(i[0][1])])    # 取最长轮廓中的点
            self.point1,self.point2,self.point3,self.point4=self.__find_point(point_ls,check,err)       # 根据公差查找出合适的对角点
            self.__draw_all_point(self.img, self.point1,self.point2,self.point3,self.point4)            # 画出这四个对角点
            if self.winmain_show==True: 
                cvwin.imshow(self.window_name,self.img)
            if self.point1!=None and self.point2!=None and self.point3!=None and self.point4!=None:
                # return [self.point1,self.point2,self.point3,self.point4]  #返回四个点
                if self.point1[0]>self.point2[0]:               # 始终返回左上角和右下角的点
                    return [self.point3,self.point4]
                else:
                    return [self.point1,self.point2]
            else:
                return None
        else:
            if self.winmain_show==True: 
                cvwin.imshow(self.window_name,self.img)
            return None

    def close_win(self):
        if self.__win_is_open(self.window_name)==True:
            if self.win_show!=False:
                cvwin.destroyWindow(self.name+'Binary')
                # cvwin.destroyWindow(self.name+'Gauss')
                # cvwin.destroyWindow(self.name+'canny')
            
            if self.winmain_show==True:
                cvwin.destroyWindow(self.window_name) 
                self.open_wins.remove(self.window_name)
                # print("close")


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    # win=[98,420,103,560]
    win=[]
    plate_det=Plate_det(win,win_show=True,winmain_show=True,undistort=True)
    while True:
        ret, img = cap.read()       #从摄像头读取,img就是每一帧的图像，是个三维矩阵
        if ret is None:
            break  
        # plate_det.prepare_template(img)         #获取模板和拍照点
        # plate_det.get_template(debug=True)
        point=plate_det.plate_det(img,check=True,err=0.03)      #返回检测到的定位板
        print(point)

