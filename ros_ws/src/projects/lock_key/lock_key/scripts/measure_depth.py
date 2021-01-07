## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.
''' Measure average depth in a user-specified bounding box from
    depth image from a Intel RealSense D435i Camera.'''

import pyrealsense2 as rs
import numpy as np
import cv2
from time import time
import csv

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
x_center, y_center = int(640/2), int(480/2)
box_size=10

def get_ave_depth(depth_frame,x_center,y_center,box_size):
    '''Get average pixel depth in bounding box from depth frame.'''
    depths=np.zeros((2*box_size,2*box_size),dtype=np.float)
    for ii in range(x_center-box_size,x_center+box_size+1):
        for jj in range(y_center-box_size,y_center+box_size+1):
            depths[ii-x_center-box_size,jj-y_center-box_size] = depth_frame.get_distance(ii, jj)
    
    ave_depth=round(np.average(depths),4)
    return ave_depth

def update_image(img,ave_depth,add_text=True,add_box=True):
    '''Add text and/or bounding box to img.'''
    if add_text:
        #Write text on image
        cv2.putText(img,f'Depth (m): {ave_depth}', 
                    (10,460), #Bottom left corner of text
                    cv2.FONT_HERSHEY_SIMPLEX, #Font
                    1.2, #Font Scale
                    (0,0,255), #Font Color (BGR)
                    4) #Line Type
    
    if add_box:
        #Add bounding box to image
        img=cv2.rectangle(img,
                          (x_center-box_size,y_center-box_size), #Start
                          (x_center+box_size,y_center+box_size), #End
                          (0,0,255), #Font Color (BGR)
                          2) #thickness
    return img
    
def main(show_image=True,show_depth=False,write_to_file=False,filename=None):
    # Start streaming
    pipeline.start(config)
    with open(filename,'w',newline='') as csvfile:
        csvwriter=csv.writer(csvfile)
        csvwriter.writerow(['Time (s)', 'Depth (m)'])
        while True:
            try:
                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue
             
                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                img = np.asanyarray(color_frame.get_data())
                #Get average depth in bounding box
                ave_depth=get_ave_depth(depth_frame,x_center,y_center,box_size)
                
                #Write timestamped data to file
                if write_to_file:
                    csvwriter.writerow([time(),ave_depth])
                
                #Show image if desired
                if show_image:                
                    if show_depth:
                        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                        img = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                
                    img=update_image(img,ave_depth,add_text=True,add_box=True)
            
                    # Show images
                    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                    cv2.imshow('RealSense', img)
                    cv2.waitKey(1)
                
            except KeyboardInterrupt:
                # Stop streaming
                pipeline.stop()
                break
        
if __name__=='__main__':
    #Build filename
    trial_id=1
    fname=rf'trial{trial_id}.csv'
    
    #Run test
    main(show_image=True,
         show_depth=True,
         write_to_file=False,
         filename=fname)