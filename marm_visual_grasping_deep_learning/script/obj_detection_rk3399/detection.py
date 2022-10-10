import cv2
import numpy as np
import os
from copy import deepcopy

# c_dir = os.path.split(os.path.realpath(__file__))[0]
#obj_net = cv2.dnn.readNetFromTensorflow(c_dir+'/pill_detection_20200717.pb', c_dir+'/pill_detection_20200717.pbtxt')
obj_class_names = ['box1', 'box2', 'box3', 'box4']

def _check(img, net, class_name):
    #img = deepcopy(frame)
    im_width, im_height, img_channel = img.shape
    net.setInput(cv2.dnn.blobFromImage(img, size=(300, 300), swapRB=True, crop=False))
    cvOut = net.forward()
    
    rets = []
    types = []
    pp = []
    
    for detection in cvOut[0, 0, :, :]:
        score = float(detection[2])
        if score > 0.5:
            label = int(detection[1])
   
            left = int(detection[3] * im_height)
            top = int(detection[4] * im_width)
            right = int(detection[5] * im_height)
            bottom = int(detection[6] * im_width)
            
            _rect = img[left: right, top: bottom]
            cv2.rectangle(img, (int(left), int(top)), (int(right), int(bottom)), (0, 0, 255), thickness=2)
            cv2.putText(img, class_name[label - 1] + ': {:.2f}'.format(score), (int(left), int(top)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255), thickness=2)
            rets.append((left,top,right-left,bottom-top))
            types.append(class_name[label - 1])
            pp.append(score)
    return img, rets, types, pp

class ObjDetect:
    def __init__(self, model_name, class_name):
        c_dir = os.path.split(os.path.realpath(__file__))[0]
        model_name = (c_dir+'/models/'+ model_name+'.pb', c_dir+'/models/'+model_name+'.pbtxt')
        self.net = cv2.dnn.readNetFromTensorflow(model_name[0], model_name[1])
        self.class_names =  class_name
        
    def detect(self, frame):
        img, rect, types, pp = _check(frame, self.net, self.class_names)
        return img, rect, types, pp
    
    def getClassIdx(self, cla_name):
        if cla_name in self.class_names:
            return self.class_names.index(cla_name)
        return -1
        
if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    _, first_img = cap.read()
    rows, cols, _ = first_img.shape
    print(rows, cols)
    pilldetect=ObjDetect("pill_detection_20220426",obj_class_names)
    while True:
        ret, img = cap.read()
        if ret is None:
            break
        img, rect, types, pp  = pilldetect.detect(img)
        print rect
         
        cv2.imshow('img', img)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
