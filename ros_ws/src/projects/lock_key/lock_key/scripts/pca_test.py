# -*- coding: utf-8 -*-
"""
Testing on PCA on raw data.
"""
import cv2
import numpy as np
from math import atan2, degrees
import image_utils

class Pic:
    '''Creates Pic object to view visualizations for color segmentation.'''
    def __init__(self, filepath, key_search_box, key_min_hsv, key_max_hsv):
        self.filepath=filepath
        self.key_search_box=key_search_box
        self.key_min_hsv=key_min_hsv
        self.key_max_hsv=key_max_hsv
        self.key_center=None
        self.key_or=None
               
    def update_img(self,img):
        '''Compute object centers and adds visuals to image.'''
        #Update img for key
        img,self.key_center,self.key_or=image_utils.color_change(img,
                                                                  self.key_search_box,
                                                                  self.key_min_hsv,
                                                                  self.key_max_hsv,
                                                                  new_pixel=[0,255,0], #Green
                                                                  centroid_pixel=[255,0,255]) #Pink
        
        #Add search box to img for key
        img=image_utils.add_quad(img,self.key_search_box,color=[255,0,255]) #Pink
        
        return img, self.key_center, self.key_or
    
    def visualize(self,show=False):
        '''Display live webcam video stream or recorded video'''
        raw_img = cv2.imread(self.filepath)
        img, key_center, key_or=self.update_img(raw_img)
        if show:
            cv2.imshow('Object Centers', img)
        print('Center: '+str(key_center),', Orien.: '+str(degrees(key_or)-90))

        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__=='__main__':
    #Define input parameters
    search_box={'min':{'x':int(60),'y':int(60)},
                'max':{'x':int(300),'y':int(200)}}
    hsv={'key':{'min':[0,int(0),int(95)],
                'max':[100,int(255),int(255)]}}
    # filepath=r"C:\Users\craig\Pictures\key1.JPG"
    filepath='/home/pandarobot/Desktop/key_image4.png'
    p=Pic(filepath, search_box, hsv['key']['min'], hsv['key']['max'])
    p.visualize(show=True)