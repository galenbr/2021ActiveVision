# -*- coding: utf-8 -*-
"""
Webcam viewer with OpenCV
"""
import cv2
import numpy as np

def draw_circles(img): 
    '''Detects circles in image and plots them.'''     
    # Convert to grayscale. 
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
      
    # Blur using 3 * 3 kernel. 
    gray_blurred = cv2.blur(gray, (3, 3)) 
      
    # Apply Hough transform on the blurred image. 
    detected_circles = cv2.HoughCircles(gray_blurred,  
                       cv2.HOUGH_GRADIENT, 1, 20, param1 = 50, 
                   param2 = 30, minRadius = 1, maxRadius = 40) 
      
    # Draw circles that are detected. 
    if detected_circles is not None: 
      
        # Convert the circle parameters a, b and r to integers. 
        detected_circles = np.uint16(np.around(detected_circles)) 
      
        for pt in detected_circles[0, :]: 
            a, b, r = pt[0], pt[1], pt[2] 
      
            # Draw the circumference of the circle. 
            cv2.circle(img, (a, b), r, (0, 255, 0), 2) 
      
            # Draw a small circle (of radius 1) to show the center. 
            cv2.circle(img, (a, b), 1, (0, 0, 255), 3) 

    return img

def color_change(img,search_box,min_bgr=[150,0,0],max_bgr=[255,150,150],
                 new_pixel=[255,0,0]):
    '''Change pixels within a certain color range to a new color.'''
    #Convert input to arrays
    min_bgr=np.array(min_bgr)
    max_bgr=np.array(max_bgr)
    #Isolate desired pixel indices
    region=np.array(find_color(img,min_bgr,max_bgr))
    #Define bounds from search_box arg
    xmin=search_box['x']-int(search_box['width']/2)
    xmax=search_box['x']+int(search_box['width']/2)
    ymin=search_box['y']-int(search_box['height']/2)
    ymax=search_box['y']+int(search_box['height']/2)
    #Create boolean filter for indexing
    x_filt=np.array((region[1]>=xmin) & (region[1]<=xmax),dtype=bool)
    y_filt=np.array((region[0]>=ymin) & (region[0]<=ymax),dtype=bool)
    region_filter=x_filt & y_filt
    #Build final type of pixel indices
    final_region=(region[0][region_filter],
                  region[1][region_filter])
    #Change color of pixels
    img[final_region]=new_pixel
    return img

def find_color(img,min_bgr=[150,0,0],max_bgr=[255,150,150]):
    '''Identify pixels in image within a color range.'''
    #Isolate desired region. Retrieve row and column indices
    region=np.where((img[:,:,0]>min_bgr[0]) & (img[:,:,0]<max_bgr[0]) & 
                    (img[:,:,1]>min_bgr[2]) & (img[:,:,1]<max_bgr[1]) & 
                    (img[:,:,2]>min_bgr[2]) & (img[:,:,2]<max_bgr[2]))
    
    return region

def add_box(img,x,y,width,height):
    '''Adds a box box to an image'''
    img=cv2.rectangle(img,
                      (x-int(width/2),y-int(height/2)), #Start
                      (x+int(width/2),y+int(height/2)), #End
                      (0,0,255), #Font Color (BGR)
                      2) #thickness
    return img

def video_stream(mirror=True):
    '''Display live webcam video stream'''
    cam = cv2.VideoCapture(0)
    search_box={'x':320,'y':240,'width':30,'height':30}
    min_bgr=[0,0,0]
    max_bgr=[128,128,128]
    
    while True:
        ret_val, img = cam.read()
        if mirror: 
            img = cv2.flip(img, 1)
        
        cc_img=color_change(img,search_box,min_bgr,max_bgr)
        box_cc_img=add_box(cc_img,
                           search_box['x'],
                           search_box['y'],
                           search_box['width'],
                           search_box['height'])
            
        cv2.imshow('Webcam', box_cc_img)

        if cv2.waitKey(1) == 27: 
            break  # esc to quit
    cv2.destroyAllWindows()
    
if __name__=='__main__':
    video_stream(mirror=False)
