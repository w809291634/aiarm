#!/usr/bin/python
# -*- coding: utf-8 -*-
import threading
import cv2 as cv
import sys
import time
import copy
from collections import OrderedDict

this = sys.modules[__name__]
this.showwins={}            #{"name":{"img":img,"update":True},}
this.destroywins=[]
this.namedWindows={}        #{"name":True}
this.createTrackbars=OrderedDict()     #{"winname":{"trackbarname":{"param":[trackbarname,winname,value,count,TrackbarCallback],"update":True,"value":0},},}
this.createTrackbars_win=OrderedDict() #{"winname":createTrackbars_win}
this.Trackbarvalues={}      #{"winname":{trackbarname:value},}

def namedWindow(name):
    this.namedWindows[name]=True        #ture：需要启动

def createTrackbar(trackbarname,winname,value,count,TrackbarCallback):
    Trackbar={
        "param":[trackbarname,winname,value,count,TrackbarCallback],
        "update":True,
        "value":value
    }
    this.createTrackbars_win[trackbarname]=Trackbar
    this.createTrackbars[winname]=copy.deepcopy(this.createTrackbars_win)

def getTrackbarPos(trackbarname, winname):
    return this.createTrackbars[winname][trackbarname]["value"]

def imshow(name,img):
    win={
        "img":img,
        "update":True
    }
    this.showwins[name]=win

def destroyWindow(name):
    this.destroywins.append(name)


def cvwin():
    while True:
        for i in this.namedWindows.keys():
            if this.namedWindows[i]==True:
                cv.namedWindow(i)
                this.namedWindows[i]=False
        
        for m in this.createTrackbars.keys():   #win
            if this.namedWindows[m]==False:    #check win is open
                for n in this.createTrackbars[m].keys(): #Trackbar
                    try :
                        t=cv.getWindowProperty(m,cv.WND_PROP_VISIBLE)
                    except:
                        pass
                    if this.createTrackbars[m][n]["update"]==True and t and this.namedWindows[m]==False:   #检查窗口打开:
                        data=this.createTrackbars[m][n]["param"]
                        cv.createTrackbar(data[0],data[1],data[2],data[3],data[4])
                        this.createTrackbars[m][n]["update"]=False

        for i in this.showwins.keys():
            win=this.showwins[i]
            if win["update"]==True:
                cv.imshow(i, win["img"])
                win["update"]=False

        while len(this.destroywins)>0:
            try:
                delwin=this.destroywins[0]
                if cv.getWindowProperty(delwin,cv.WND_PROP_VISIBLE):
                    del this.destroywins[0]
                    del this.showwins[delwin]
                    cv.destroyWindow(delwin)
            finally :
                break  
                
        for m in this.createTrackbars.keys():   #win
            try :
                if cv.getWindowProperty(m,cv.WND_PROP_VISIBLE):   #检查窗口打开
                    for n in this.createTrackbars[m].keys():
                        if this.createTrackbars[m][n]["param"]:
                            data=this.createTrackbars[m][n]["param"]
                            this.createTrackbars[m][n]["value"]=cv.getTrackbarPos(data[0],data[1])
                            # print(m,n,this.createTrackbars[m][n]["value"])
            except:
                break

        cv.waitKey(10)

t = threading.Thread(target=cvwin)
t.setDaemon(True)
t.start()


