#!/usr/bin/python3
# -- coding: utf-8 --

import rospy 
import cv2

import torch
from sensor_msgs.msg import Image as Imagemsg
from cv_bridge import CvBridge, CvBridgeError
# from SSD_detector import *
from segmentation_inference import *

show_video = True
save_video = False

#Load segmentation inference
si = SegmentationInference(2)

class ObjectDetector: 
    def __init__(self):
        print('asd')
        self.bridge = CvBridge()
        # self.camera_sub  = rospy.Subscriber('/turtlebot/realsense_d435i/color/image_raw',Imagemsg,self.callback)
        self.camera_sub  = rospy.Subscriber('/camera/color/image_raw',Imagemsg,self.callback)
    
    def callback(self,data):
        print('MSG RECEIVED')
        try: 
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print("CvBridge could not convert images from realsense to opencv")
        height,width, channels = image.shape

        # annotated = inference(image)

        frame = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # cv2.imshow("Image window", annotated)
        # Sent frame to segmentation inferece and get result
        prediction_np, mask, result = si.process(frame)
        #Count FPS

        # time_stop = time.time()
        # fps = 1.0 / (time_stop - time_start)
        result = (result * 255).astype(numpy.uint8)


        # Print FPS
        
        im_bgr = cv2.cvtColor(result, cv2.COLOR_RGB2BGR)
        # print_video(im_bgr, text)

        cv2.imshow('frame', im_bgr)

        cv2.waitKey(3)
        
    print('Object detector running')


if __name__ == '__main__':
    rospy.init_node('segmentation')
    print('something')
    node = ObjectDetector()
    rospy.spin()
