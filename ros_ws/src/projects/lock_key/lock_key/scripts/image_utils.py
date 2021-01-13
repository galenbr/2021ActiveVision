# -*- coding: utf-8 -*-
"""
Image processing utility functions
"""

import copy
import cv2
import numpy as np
from math import atan2, cos, sin, sqrt, pi
import matplotlib.pyplot as plt
from scipy.ndimage import label
from scipy.stats import trim_mean

def drawAxis(img, p_, q_, colour, scale):
    '''Draw Axis from PCA.'''
    p = list(p_)
    q = list(q_)
    
    angle = atan2(p[1] - q[1], p[0] - q[0]) # angle in radians
    hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))
    # Here we lengthen the arrow by a factor of scale
    q[0] = p[0] - scale * hypotenuse * cos(angle)
    q[1] = p[1] - scale * hypotenuse * sin(angle)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
    # create the arrow hooks
    p[0] = q[0] + 9 * cos(angle + pi / 4)
    p[1] = q[1] + 9 * sin(angle + pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
    p[0] = q[0] + 9 * cos(angle - pi / 4)
    p[1] = q[1] + 9 * sin(angle - pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
    
def getOrientation(pts, img):
    
    sz = len(pts)
    data_pts = np.empty((sz, 2), dtype=np.float64)
    for i in range(data_pts.shape[0]):
        data_pts[i,0] = pts[i,0,0]
        data_pts[i,1] = pts[i,0,1]
    # Perform PCA analysis
    mean = np.empty((0))
    mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)
    # Store the center of the object
    cntr = (int(mean[0,0]), int(mean[0,1]))
    
    cv2.circle(img, cntr, 3, (255, 0, 255), 2)
    p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
    p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])
    drawAxis(img, cntr, p1, (0, 255, 0), 1)
    drawAxis(img, cntr, p2, (255, 255, 0), 5)
    angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
    
    return angle

def perform_pca(img,gray=None):
    '''Identify contours of objects. Finds and plots axes of principal
    components.'''
    if gray is None:
         # Convert image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Convert image to binary
    _, bw = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    # countours_in_box=[]
    for i, c in enumerate(contours):
        # Calculate the area of each contour
        area = cv2.contourArea(c)
        # Ignore contours that are too small or too large
        if area < 1e2 or 1e5 < area: #Original: 1e2 - 1e5
            continue
   # for i, c in enumerate(contours_in_box):
   #       # Draw each contour only for visualisation purposes
   #      cv2.drawContours(img, contours, i, (0, 0, 255), 2)
   #      # Find the orientation of each shape
   #      getOrientation(c, img)
    return img

def draw_circles(img,search_box): 
    '''Detects circles in image and plots them.'''     
    # Convert to grayscale. 
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
      
    # Blur using 3 * 3 kernel. 
    gray_blurred = cv2.blur(gray, (3, 3)) 
      
    # Apply Hough transform on the blurred image. 
    detected_circles = cv2.HoughCircles(gray_blurred,  
                       cv2.HOUGH_GRADIENT, 1, 20, param1 = 50, 
                   param2 = 30, minRadius = 10, maxRadius = 25)
    
    #Define bounds from search_box arg
    xmin=search_box['x']-int(search_box['width']/2)
    xmax=search_box['x']+int(search_box['width']/2)
    ymin=search_box['y']-int(search_box['height']/2)
    ymax=search_box['y']+int(search_box['height']/2)
    
    # Draw circles that are detected. 
    if detected_circles is not None: 
      
        # Convert the circle parameters a, b and r to integers. 
        detected_circles = np.uint16(np.around(detected_circles)) 
      
        for pt in detected_circles[0, :]: 
            a, b, r = pt[0], pt[1], pt[2] 
            if (a>=xmin) & (a<=xmax) & (b>=ymin) & (b<=ymax):        
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
    new_img=copy.deepcopy(img)
    new_img[final_region]=new_pixel
    
    # Manual 'Centroid' finding
    try:
        # if len(final_region[0])>200:
        # Identify mean index and mark with circle
        # Mean is trimmed on both tails
        x_ave=int(trim_mean(final_region[1],0.4))
        y_ave=int(trim_mean(final_region[0],0.4))
        center=(x_ave, y_ave)
        cv2.circle(new_img, center, 1, (255, 0, 255), 6)
    except ValueError:
        center=(None,None)
        pass

    return new_img, center

def find_color(img,min_hsv,max_hsv):
    '''Identify pixels in image within a color range.'''
    #Convert img to HSV
    img=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    region=np.where((img[:,:,0]>=min_hsv[0]) & (img[:,:,0]<=max_hsv[0]) & 
                    (img[:,:,1]>=min_hsv[1]) & (img[:,:,1]<=max_hsv[1]) & 
                    (img[:,:,2]>=min_hsv[2]) & (img[:,:,2]<=max_hsv[2]))
    
    return region

def add_box(img,x,y,width,height,color=[0,0,255]):
    '''Adds a box box to an image'''
    img=cv2.rectangle(img,
                      (x-int(width/2),y-int(height/2)), #Start
                      (x+int(width/2),y+int(height/2)), #End
                      color, #Font Color (BGR)
                      2) #thickness
    return img

def detect_edges(img):
    '''Detects edges in image.'''
    edges = cv2.Canny(img,100,200)
    return edges

def depth(left_img,right_img):
    '''Calculate and view the disparity map from two images.'''
    stereo = cv2.StereoBM_create(numDisparities=16*5, blockSize=11) #16, 15
    disparity = stereo.compute(left_img,right_img)
    plt.imshow(disparity,'plasma')#'gray')
    plt.show()

def segment_on_dt(a, img):
    border = cv2.dilate(img, None, iterations=5)
    border = border - cv2.erode(border, None)
    dt = cv2.distanceTransform(img, 2, 3)
    dt = ((dt - dt.min()) / (dt.max() - dt.min()) * 255).astype(np.uint8)
    _, dt = cv2.threshold(dt, 180, 255, cv2.THRESH_BINARY)
    lbl, ncc = label(dt)
    lbl = lbl * (255 / (ncc + 1))
    # Completing the markers now.
    lbl[border == 255] = 255
    lbl = lbl.astype(np.int32)
    cv2.watershed(a, lbl)
    lbl[lbl == -1] = 0
    lbl = lbl.astype(np.uint8)
    return 255 - lbl

def run_watershed(img,show_center=False):
    # Pre-processing.
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)   
    _, img_bin = cv2.threshold(img_gray, 0, 255, cv2.THRESH_OTSU)
    img_bin = cv2.morphologyEx(img_bin, cv2.MORPH_OPEN,np.ones((3, 3), dtype=int))
    result = segment_on_dt(img, img_bin) 

    if show_center:
        region=np.where(result==255)
        #Identify center point
        x_idx,y_idx=int(trim_mean(region[1],0.4)),int(trim_mean(region[0],0.4))
        # Center coordinates
        center_coordinates = (x_idx,y_idx)
        # Radius of circle
        radius = 10
        # Blue color in BGR
        color = (255, 0, 0)
        # Line thickness of 2 px
        thickness = 2
        result = cv2.circle(result, center_coordinates, radius, color, thickness)
    return result

def add_blur(img):
    img = cv2.blur(img,(3,3))
    img=cv2.medianBlur(img,3)
    img= cv2.GaussianBlur(img,(5,5),0)
    img= cv2.bilateralFilter(img,9,5,5)
    return img