# -*- coding: utf-8 -*-
"""
Preliminary key segmentation and centroid finding
"""
import cv2
import image_utils

class Stream:
    '''Creates Stream object to view live or recorded video with 
    visualizations for color segmentation and pca.'''
    def __init__(self, filepath, single_image, search_box, min_hsv, max_hsv, 
                 pca=True):
        self.filepath=filepath
        self.single_image=single_image
        self.search_box=search_box
        self.min_hsv=min_hsv
        self.max_hsv=max_hsv
        self.pca=pca
        
    def perform_ops(self,img):
        '''Performs operations on image before viewing.'''
        # Change colors based on min/max bgr values within a search box
        # img=image_utils.add_blur(img)
        img=image_utils.color_change(img,self.search_box,
                                     self.min_hsv,self.max_hsv)

        #Perform watershed segmentation
        # img=image_utils.run_watershed(img,show_center=False)
        # #Convert gray image to RGB equivalent
        # img = cv2.cvtColor(img,cv2.COLOR_GRAY2RGB)
        
        # if self.pca:
        #     img=image_utils.perform_pca(img)
        
        # img=image_utils.draw_circles(img,self.search_box)

        img=image_utils.add_box(img,
                                self.search_box['x'],
                                self.search_box['y'],
                                self.search_box['width'],
                                self.search_box['height'])        
        return img
        
    def run(self):
        '''Display live webcam video stream or recorded video'''
        
        if self.single_image:
            raw_img = cv2.imread(self.filepath)
            cv2.imshow('Video', self.perform_ops(raw_img))
        else:  
            if self.filepath=='camera':
                cam = cv2.VideoCapture(0)
            else:
                cam = cv2.VideoCapture(filepath)
            
            while True:
                _, raw_img = cam.read()
                cv2.imshow('Video', self.perform_ops(raw_img))
        
                if cv2.waitKey(1) == 27: 
                    break  # esc to quit
            cv2.destroyAllWindows()        
    
if __name__=='__main__':
    #Define input parameters
    search_box={'x':320,'y':240,'width':500,'height':400}
    # search_box={'x':525,'y':100,'width':180,'height':180}
    hsv={'lock':{'min':[40,int(0.3*255),int(0.6*255)],
                 'max':[150,int(0.9*255),int(0.8*255)]},
         'key':{'min':[40,int(0.15*255),int(0.5*255)],
                'max':[120,int(0.3*255),int(0.75*255)]}}
    # filepath=r"E:\UbuntuDriveBackup\WPI\MER\Media\lockkey_ondesk.mp4"
    # filepath='camera'
    filepath=r"C:\Users\craig\Desktop\MER_Shared\rosbags\eye_in_hand\lock_and_key\color_image_0.jpeg"
    single_image=True
    item='lock'
    
    bag_stream=Stream(filepath, single_image,
                      search_box, hsv[item]['min'], hsv[item]['max'], pca=True)
    bag_stream.run()
