# -*- coding: utf-8 -*-
"""
Preliminary key segmentation and centroid finding
"""
import cv2
import image_utils

class Stream:
    '''Creates Stream object to view live or recorded video with 
    visualizations for color segmentation and pca.'''
    def __init__(self, filepath, search_box, min_bgr, max_bgr, 
                 pca=True):
        self.filepath=filepath
        self.search_box=search_box
        self.min_bgr=min_bgr
        self.max_bgr=max_bgr
        self.pca=pca
        
    def perform_ops(self,raw_img):
        '''Performs operations on image before viewing.'''
        img=image_utils.run_watershed(raw_img,show_center=False)
        img = cv2.cvtColor(img,cv2.COLOR_GRAY2RGB)
        
        # img=image_utils.color_change(img,search_box,min_bgr,max_bgr)
        
        if self.pca:
            img=image_utils.perform_pca(raw_img)
        
        # circle_img=image_utils.draw_circles(pca_img,search_box)

        # box_cc_img=image_utils.add_box(circle_img,
        #                                search_box['x'],
        #                                search_box['y'],
        #                                search_box['width'],
        #                                search_box['height'])        
        return img
        
    def run(self):
        '''Display live webcam video stream or recorded video'''
        if filepath=='camera':
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
    search_box={'x':320,'y':240,'width':320,'height':240}
    #Padlock Brass (from close-up): [90,85,100] [140,198,205]
    #Key: [40,40,40] [95,95,95]
    min_bgr=[40,40,40]
    max_bgr=[95,95,95]
    filepath=r"E:\UbuntuDriveBackup\WPI\MER\Media\lockkey_ondesk.mp4"
    # filepath='camera'
    
    bag_stream=Stream(filepath, search_box, min_bgr, max_bgr, pca=True)
    bag_stream.run()
