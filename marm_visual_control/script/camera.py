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
from color_detection import Color_Rec
from plate_detection import Plate_det
from collections import OrderedDict
import yaml
this = sys.modules[__name__]
this.config_path="/home/zonesion/catkin_ws/src/marm_visual_control/config/config.yaml"

this.dir_f = os.path.abspath(os.path.dirname(__file__))
#loc_plate 640*480像素下 定位板框 左上角 和方框像素宽度、高度

with open(config_path, "r") as f:
    config = yaml.load(f.read())

this.color_param=config['color_param']
this.bin_param=config['bin_param']

class AiCamera(object):
    def __init__(self,color,win=[],loc_plate=[141,192,465,386],loc_plate_act=[0.147,0.173,0.092],
    loc_plate_act_origin=[0,0],loc_x_off_mx=25,loc_x_off_mi=9,loc_y_off_mx=15,color_par=None,bin_param=None):
        '''
        win=(240,450,145,500)  (y, y_max, x, x_max)
        loc_plate        单位 pix 640*480像素 定位板框 左上角x,y 和 右下角x,y
        loc_plate_act        单位 m 定位板框实际长度 上底 下底 高[0.147,0.182,0.082]
        loc_x_off_mx         单位 pix 相机倾斜视角X像素偏移 最大值
        loc_y_off_mx         单位 pix 相机倾斜视角y像素偏移 最大值
        loc_plate_origin     单位 pix cv2下像素原点
        loc_plate_act_origin 单位 m  实际定位原点 相对base_link
        ------------- 下底
         -         -
          ---------  上底
        '''
        self.window_name='camera'
        self.win=win
        self.open_wins=[]
        #初始化颜色检测和底板检测
        self.rec_cla_dict=OrderedDict()                                 #颜色识别类,含多个
        if color_par==None:                                             #本文件取颜色识别参数
            __color_par=this.color_param
            for i in color:
                if i in __color_par.keys():
                    det={
                        "class":Color_Rec(i,win,win_show=False,winmain_show=False,color_par=__color_par),
                        "pos":None
                    }
                    self.rec_cla_dict[i]=copy.deepcopy(det)
                else:
                    print("%s has no color recognition parameters"%i)
                    sys.exit()
        else:                                                           #外部获取颜色识别参数
            __color_par=color_par
            for i in color:
                if i in __color_par.keys():
                    det={
                        "class":Color_Rec(i,win,win_show=False,winmain_show=False,color_par=__color_par),
                        "pos":None
                    }
                    self.rec_cla_dict[i]=copy.deepcopy(det)
                else:
                    print("%s has no color recognition parameters"%i)
                    sys.exit()
        if bin_param==None:                                              #底板检测类
            self.plate_det=Plate_det(win,win_show=False,winmain_show=False,par=this.bin_param)   
        else:
            self.plate_det=Plate_det(win,win_show=False,winmain_show=False,par=bin_param)   
        #检测摄像头
        cam=self.__camera_check__()
        
        if cam!=-1:
            self.cap = cv2.VideoCapture(cam)
            print("set cam number %d"%cam)
            self.__switch=False
            self.frame=np.array([])
            self.point1=None
            self.point2=None
            self.block=[]
            self.success_tag=[]
            #定位板参数_实际
            self.loc_plate_act=loc_plate_act
            self.loc_plate_act_origin=loc_plate_act_origin
            #定位板参数_像素
            self.loc_x_off_mx=loc_x_off_mx
            self.loc_x_off_mi=loc_x_off_mi
            self.loc_y_off_mx=loc_y_off_mx
            self.__update_plate_par(loc_plate)

            t = threading.Thread(target=self.__pollcam)
            t.setDaemon(True)
            t.start()
        
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

    def __undistort_new(self,src):   #矫正
        DIM=(640, 480)
        K=np.array([[435.14474444957017, 0.0, 283.9309989638742], [0.0, 436.1231208629601, 254.63554543105116], [0.0, 0.0, 1.0]])
        D=np.array([[-0.05408862574600821], [-0.14665026566850226], [0.3014964438943832], [-0.20689527582320547]])
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
        __img = cv2.remap(src, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        return __img

    def __pollcam(self):
        while True:
            if self.__switch ==True:
                while True:
                    success, frame = self.cap.read()
                    if not success:
                        time.sleep(0.1)
                        continue
                    img=copy.deepcopy(frame)
                    img=self.__undistort(img)
                    if len(self.win)!=0:
                        img = img[self.win[0]:self.win[1], self.win[2]:self.win[3]] #裁剪图像   
                    if self.point1!=None and self.point2!=None :
                        point1=copy.deepcopy(self.point1)
                        point2=copy.deepcopy(self.point2)
                        self.__drawpoint(img,self.point1)
                        self.__drawpoint(img,self.point2)

                    for i in range(len(self.block)):
                        for m in self.block[i]:
                            if m !=None:
                                point=[m[0],m[1]]
                                self.__drawpoint(img,point)

                    self.__open_win(img)
                    self.frame=frame
                    time.sleep(0.01)
                    if self.__switch !=True:
                        if time.time()-st>5:
                            break
                    else:
                        st=time.time()
            else: 
                self.frame=np.array([])
                self.__close_win()
                time.sleep(0.5)

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
 
    def __update_plate_par(self,loc_plate):
        self.loc_plate=loc_plate
        self.loc_plate_width=self.loc_plate[2]-self.loc_plate[0]
        self.loc_plate_height=self.loc_plate[3]-self.loc_plate[1]
        # if len(self.win)!=0:
        #     self.loc_plate=[loc_plate[0]-self.win[2],loc_plate[1]-self.win[0],loc_plate[2]-self.win[2],loc_plate[3]-self.win[0]]
        self.loc_plate_origin=[self.loc_plate[2]-(self.loc_plate[2]-self.loc_plate[0])/2,self.loc_plate[3]]

    def cam_ctrl(self,switch):
        self.__switch=switch
    
    def __check(self,a,b,err=5,err_a=3.0):
        if type(a)==int and type(b)==int :
            if abs(a-b)<err:                        #像素
                return True
            else:
                return False
        if type(a)==float and type(b)==float :      #角度
            if abs(a-b)<err_a:
                return True
            else:
                return False
    
    def __drawpoint(self,img,point):
        point=(point[0],point[1])
        cv2.circle(img,point, 0, (255, 255, 0), 3)   #画点
   
    def block_loc(self):
        # {"red":[pos],"green":pos}
        # {"red":pos,"green":pos}
        self.cam_ctrl(True) 
        time.sleep(0.5)         # 等待摄像头刷新
        self.block=[]  
        self.success_tag=[]
        self.point1=None
        self.point2=None
        while(np.size(self.frame)==0):
            if np.size(self.frame)!=0:
                break
            time.sleep(0.1)
  
        #检测底板
        point=None
        __la_pos=[]             #清除数据列表
        num=0
        palte_det_time=config['palte_det_time']
        palte_det_err=config['palte_det_err']
        palte_det_st=time.time()
        while(point==None):
            point=self.plate_det.plate_det(self.frame,check=True,err=palte_det_err)       #检测定位板,误差0.03(参数)
            time.sleep(0.1)
            if time.time()-palte_det_st>palte_det_time:
                print("plate detetion timeout! retry")
                return -1
        times_of_palte_det=config['times_of_palte_det']
        while True:
            if len(__la_pos) == 0:
                __la_pos = [point]
                continue
            point=self.plate_det.plate_det(self.frame,check=True,err=palte_det_err)       #检测定位板,误差0.03(参数)
            if point!=None:
                if self.__check(point[0][0],__la_pos[-1][0][0])==True and self.__check(point[0][1],__la_pos[-1][0][1])==True \
                and self.__check(point[1][0],__la_pos[-1][1][0])==True and self.__check(point[1][1],__la_pos[-1][1][1])==True  :
                    __la_pos.append(point)
                else:
                    __la_pos = [point]
            if len(__la_pos)==times_of_palte_det:
                break
            if time.time()-palte_det_st>palte_det_time:
                print("plate detetion timeout! retry")
                return -1
        plate_det_use_time=time.time()-palte_det_st
        sumx = sumy = sumx1 = sumy1 = 0
        for m in __la_pos:
            sumx += m[0][0]
            sumy += m[0][1]
            sumx1 += m[1][0]
            sumy1 += m[1][1]
        x = sumx / len(__la_pos)
        y = sumy / len(__la_pos)
        x1 = sumx1 / len(__la_pos)
        y1 = sumy1 / len(__la_pos)
        data=[x,y,x1,y1]
        point1=[x,y]
        point2=[x1,y1]
        print("locating plate ---> %s use time: %.2f"%(data,plate_det_use_time))
        self.__update_plate_par(data)               #更新定位板的位置
        self.point1=copy.deepcopy(point1)           #左上角
        self.point2=copy.deepcopy(point2)           #右下角

        #检测颜色物体
        for i in self.rec_cla_dict.keys():
            self.rec_cla_dict[i]["pos"]=None                # 清理位置数据 
            pos=None
            la_pos=[]
            block_loc_st=time.time()                        # 记录木块定位的起始时间
            def color_det(color,st_time):                           # 初步判断此颜色目标是否存在
                __pos=None
                num=0
                while True:
                    __pos=self.rec_cla_dict[color]["class"].find_pos(self.frame)    #[[同色物块1][同色物块2][同色物块3][同色物块4]....]
                    pos_ls=[]
                    if __pos!=None:
                        for i in __pos:
                            if self.point1!=None and self.point2!=None:         #检测识别颜色木块是否在定位框内
                                if i[0]<self.point2[0] and i[0]>self.point1[0] and \
                                i[1]<self.point2[1] and i[1]> self.point1[1]:
                                    pos_ls.append([i])          #在定位框内添加
                                    num+=1              # 有效数量
                                else:
                                    pos_ls.append([])           #在定位框内添加空
                            else:
                                return -1,pos_ls,num    # 没有定位点
                        if num>0:                       # 定位框内有木块
                            return 0,pos_ls,num         
                        else:
                            return -1,pos_ls,num        # 超出范围
                    time.sleep(0.1)
                    if time.time()-st_time>config['color_det_time']:
                        return -1,__pos,num 
            color_det_st=time.time()
            sta,pos,num=color_det(i,color_det_st)                    # 返回初次探测到的该类颜色的所有检测对象
            color_det_use_time=time.time()-color_det_st
            # (初次探测下的)检测状态位,位置列表,检测在定位框内的有效数量
            # print("pos",pos)                          # [[[同色物块1]], [[同色物块2]], [[同色物块3]]]
            if sta==0:
                print("%s Detected"%i)
            else:
                print("%s Not Detected"%i)
                continue

            def color_det_ex(color,init_pos,valid_num,st_time):         # 识别颜色木块位置
                __la_pos=[] 
                color_loc_times=config['color_loc_times']       # 参数:重复检测次数
                success_idx=[]
                def clear():        # 清理该颜色所有的检测结果
                    for i in range(len(__la_pos)):
                        __la_pos[i]=[]
                while True:              
                    if len(__la_pos) == 0:
                        __la_pos = init_pos            # [[] [[同色物块2]] [[同色物块3]] [[同色物块4]]....]  经过区域筛选
                        continue
                    # print(__la_pos)
                    pos=self.rec_cla_dict[color]["class"].find_pos(self.frame)  #[[同色物块1][同色物块2][同色物块3][同色物块4]....]
                    if pos!=None:
                        if len(__la_pos)!=len(pos):     
                            clear()
                            return -1,__la_pos          # 初次探测和检测数量不一致
                        for i in range(len(pos)):
                            if i in success_idx:                        #满足要求的不再继续采集
                                continue
                            if len(__la_pos[i])==color_loc_times:       #采集数量满足要求
                                success_idx.append(i)
                                continue
                            if self.point1!=None and self.point2!=None:
                                if pos[i][0]<self.point2[0] and pos[i][0]>self.point1[0] and \
                                pos[i][1]< self.point2[1] and pos[i][1]> self.point1[1]:
                                    if __la_pos[i]:
                                        if self.__check(pos[i][0],__la_pos[i][-1][0])==True and self.__check(pos[i][1],__la_pos[i][-1][1])==True \
                                        and self.__check(pos[i][2],__la_pos[i][-1][2])==True :
                                            __la_pos[i].append(pos[i])      #[[] [[同色物块2][同色物块2]] [[同色物块3]] [[同色物块4]]....]
                                        else:
                                            __la_pos[i] = [pos[i]]          #重新取数据
                                else:
                                    __la_pos[i]=[]      # 超出范围
                            else:
                                return -1,__la_pos      # 没有定位点 
                    def check():                        # 将没有采集完全的木块数据全部清除
                        for i in range(len(pos)):
                            if i not in success_idx:
                                __la_pos[i]=[]
                    if len(success_idx)==valid_num:     # 所有同色木块数据都满足数量
                        check()
                        return 0,__la_pos
                    # time.sleep(0.1)
                    if time.time()-st_time>config['color_loc_time']:
                        if len(success_idx)==0:
                            clear()
                            return -1,__la_pos          # 定位超时     
                        check()
                        return 0,__la_pos               # 返回有效检测的一部分
            color_det_ex_st=time.time()
            sta,la_pos=color_det_ex(i,pos,num,color_det_ex_st)
            color_det_ex_use_time=time.time()-color_det_ex_st
            # print("finaly",sta,la_pos)
            # [[[同色物块1], [同色物块1], [同色物块1], [同色物块1], [同色物块1]]])
            if sta==0:                              #定位木块成功
                #[[同色物块1,同色物块1,同色物块1..] [同色物块2,同色物块2..] [同色物块3,同色物块3..] ....]
                print("%s Block positioning succeeded! det use time: %.2f loc use time: %.2f"%(i,color_det_use_time,color_det_ex_use_time))
                data=[]
                for m in la_pos:
                    if m:
                        sumx = sumy = sumw = 0
                        for n in m:
                            sumx += n[0]
                            sumy += n[1]
                            sumw += n[2]
                        x = sumx / len(m)
                        y = sumy / len(m)
                        w = sumw / len(m)
                        z=[x,y,w]
                        data.append(z)
                self.rec_cla_dict[i]["pos"]=copy.deepcopy(data)         #将该颜色像素坐标和角度储存
                self.rec_cla_dict[i]["class"].close_win()
                # print(i,self.rec_cla_dict[i]["pos"])
                self.block.append(data)
                self.success_tag.append(i)
            else:                                                       #定位木块失败
                self.rec_cla_dict[i]["pos"]=None         
                self.rec_cla_dict[i]["class"].close_win()
                print("%s Block positioning failed"%i)
                continue
        self.cam_ctrl(False)                                            #关闭摄像头
        return 0

    def cv2_to_plate(self):
        '''
        CV2坐标系转换到与base_link同方向,单位PIX
                   ^  X
                   |
                   |
             ------------- 下底
              -    |    -
        <---------------  上底
                   ORIGIN
        '''
        # print(self.loc_plate)
        # print("loc_plate_origin:",self.loc_plate_origin)
        if self.block_loc()==0:
            for i in self.success_tag:
                for m in range(len(self.rec_cla_dict[i]['pos'])):
                    pix_x=self.rec_cla_dict[i]['pos'][m][1]         # 像素的x对应y方向
                    pix_y=self.rec_cla_dict[i]['pos'][m][0]
                    # 坐标转换
                    x_proportion=float(self.loc_plate_origin[1]-pix_x-(self.loc_x_off_mx+self.loc_x_off_mi)/2)/self.loc_plate_height
                    x_off=self.loc_x_off_mi+(self.loc_x_off_mx-self.loc_x_off_mi)*x_proportion
                    x=self.loc_plate_origin[1]-pix_x-int(x_off)
                    if pix_y>self.loc_plate_origin[0]:
                        y_off=pix_y-self.loc_plate_origin[0]
                        y=-(y_off-self.loc_y_off_mx*(y_off/(self.loc_plate_height/2)))
                    else:
                        y_off=self.loc_plate_origin[0]-pix_y
                        y=y_off-self.loc_y_off_mx*(y_off/(self.loc_plate_height/2))
                    self.rec_cla_dict[i]['pos'][m][0]=copy.deepcopy(x)
                    self.rec_cla_dict[i]['pos'][m][1]=copy.deepcopy(y)
                # print(i,self.rec_cla_dict[i]["pos"])
            return 0
        else:
            return -1

    def plate_to_base(self):
        if self.cv2_to_plate()==0:
            for i in self.success_tag:
                for m in range(len(self.rec_cla_dict[i]['pos'])):
                    plate_x=self.rec_cla_dict[i]['pos'][m][0]
                    plate_y=self.rec_cla_dict[i]['pos'][m][1]
                    # print(self.rec_cla_dict[i]["pos"][0]),self.loc_plate_height
                    x_proportion=float(plate_x)/self.loc_plate_height
                    y_proportion=float(2*plate_y)/self.loc_plate_width
                    x=float(self.loc_plate_act_origin[0])+self.loc_plate_act[2]*x_proportion
                    y=float(self.loc_plate_act_origin[1])+(self.loc_plate_act[0]/2+(self.loc_plate_act[1]-self.loc_plate_act[0])/2*x_proportion)*y_proportion
                    self.rec_cla_dict[i]['pos'][m][0]=copy.deepcopy(x)
                    self.rec_cla_dict[i]['pos'][m][1]=copy.deepcopy(y)
                # print i,"--->",self.rec_cla_dict[i]["pos"]
                print("%s ---> %s"%(i,self.rec_cla_dict[i]["pos"]))
            return 0
        else:
            return -1

if __name__ == '__main__':
    # win=[98,420,103,560]
    win=[]
    aicamer=AiCamera(("red","yellow","blue","green"),win)
    aicamer.cam_ctrl(True)
    # aicamer=AiCamera(("red",),win)
    while True:
        aicamer.plate_to_base()
        print('///////')
        time.sleep(1)
    #     break
    # time.sleep(20)
    # aicamer.plate_to_base()
    # time.sleep(20)
    
