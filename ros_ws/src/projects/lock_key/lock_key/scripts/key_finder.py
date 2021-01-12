# -*- coding: utf-8 -*-
"""
Preliminary key segmentation and centroid finding
"""
import cv2
import image_utils

class Stream:
    '''Creates Stream object to view live or recorded video with 
    visualizations for color segmentation.'''
    def __init__(self, filepath, single_image, 
                 key_search_box, key_min_hsv, key_max_hsv, 
                 lock_search_box, lock_min_hsv, lock_max_hsv):
        self.filepath=filepath
        self.single_image=single_image
        self.key_search_box=key_search_box
        self.key_min_hsv=key_min_hsv
        self.key_max_hsv=key_max_hsv
        self.key_center=None
        self.lock_search_box=lock_search_box
        self.lock_min_hsv=lock_min_hsv
        self.lock_max_hsv=lock_max_hsv
        self.lock_center=None
               
    def update_img(self,img):
        '''Compute object centers and adds visuals to image.'''
        #Update img for key
        img,self.key_center=image_utils.color_change(img,
                                                     self.key_search_box,
                                                     self.key_min_hsv,
                                                     self.key_max_hsv,
                                                     new_pixel=[255,0,0])
        #Update img for lock
        img,self.lock_center=image_utils.color_change(img,
                                                     self.lock_search_box,
                                                     self.lock_min_hsv,
                                                     self.lock_max_hsv,
                                                     new_pixel=[0,255,0])
        
        #Add search box to img for key
        img=image_utils.add_box(img,
                                self.key_search_box['x'],
                                self.key_search_box['y'],
                                self.key_search_box['width'],
                                self.key_search_box['height'],
                                color=[0,0,255])
        #Add search box to img for lock
        img=image_utils.add_box(img,
                                self.lock_search_box['x'],
                                self.lock_search_box['y'],
                                self.lock_search_box['width'],
                                self.lock_search_box['height'],
                                color=[0,200,200])
        
        return img, self.key_center, self.lock_center
    
    def visualize(self,show=False):
        '''Display live webcam video stream or recorded video'''
        
        if self.single_image:
            raw_img = cv2.imread(self.filepath)
            img, key_center, lock_center=self.update_img(raw_img)
            if show:
                cv2.imshow('Object Centers', img)
            print('Key: '+str(key_center)+' , Lock: '+str(lock_center))
        else:  
            if self.filepath=='camera':
                cam = cv2.VideoCapture(0)
            else:
                cam = cv2.VideoCapture(filepath)
            
            while True:
                _, raw_img = cam.read()
                img, key_center, lock_center=self.update_img(raw_img)
                if show:
                    cv2.imshow('Object Centers', img)
                print('Key: '+str(key_center)+' , Lock: '+str(lock_center))
        
                if cv2.waitKey(1) == 27: 
                    break  # esc to quit
            cv2.destroyAllWindows()        
    
if __name__=='__main__':
    #Define input parameters
    search_box={'lock':{'x':225,'y':100,'width':180,'height':180},
                'key':{'x':525,'y':100,'width':180,'height':180}}
    hsv={'lock':{'min':[40,int(0.3*255),int(0.6*255)],
                 'max':[150,int(0.9*255),int(0.8*255)]},
         'key':{'min':[95,int(0.18*255),int(0.6*255)],
                'max':[110,int(0.3*255),int(0.85*255)]}}
    filepath=r"C:\Users\craig\Desktop\MER_Shared\rosbags\eye_in_hand\lock_and_key\color_image_0.jpeg"
    single_image=True
    #Start stream
    bag_stream=Stream(filepath, single_image, 
                      search_box['key'], hsv['key']['min'], hsv['key']['max'], 
                      search_box['lock'], hsv['lock']['min'], 
                      hsv['lock']['max'])
    bag_stream.visualize(show=True)
    print('Key:',bag_stream.key_center,
          'Lock:',bag_stream.lock_center)