# -*- coding: utf-8 -*-
"""
ROS node for lock and key segmentation and centroid finding
"""
import cv2
import image_utils
import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

class FinderPub:
    '''Finds lock and key from RGBD data.'''
    def __init__(self, key_search_box, key_min_hsv, key_max_hsv, 
                 lock_search_box, lock_min_hsv, lock_max_hsv):
        self.key_search_box=key_search_box
        self.key_min_hsv=key_min_hsv
        self.key_max_hsv=key_max_hsv
        self.key_center=None
        self.lock_search_box=lock_search_box
        self.lock_min_hsv=lock_min_hsv
        self.lock_max_hsv=lock_max_hsv
        self.lock_center=None
        self.rgb_image=None
        self.rgb_received=False
        self.depth_image=None
        self.depth_received=False
        #ROS
        self.bridge=CvBridge()
        self.rgb_sub=rospy.Subscriber('/camera/color/image_raw',
                                      Image,self.rgb_callback)
        self.depth_sub=rospy.Subscriber('/camera/depth/image_rect_raw',
                                        Image,self.depth_callback)
        self.vis_pub = rospy.Publisher('finder_image', Image, queue_size=10)
        self.key_pub = rospy.Publisher('key_point', PointStamped, 
                                       queue_size=10)
        self.lock_pub = rospy.Publisher('lock_point', PointStamped, 
                                        queue_size=10)
    
    def calculate_positions(self):
        '''Calculates centers of lock and key.'''
        #Get lock and key centers in terms of pixels
        vis_img=self.get_centers()
        #*****Get displacement in terms of meters*******
        #*****Get corresponding depth*****
        
        #Publish centers and visualization image
        self.publish_points(lock_disp,key_disp)
        publish_vis(vis_img)
        
    def depth_callback(self,raw_depth_img)   
        #Convert to cv2 format and store image
        self.depth_img=self.bridge.imgmsg_to_cv2(raw_depth_img,"32FC1")
        self.depth_received=True

    def publish_points(self,lock_disp,key_disp):
        '''Publish lock and key positions in terms of camera_link.'''
        
    def publish_vis(self,vis_img):
        '''Define and publish visualization image'''
        h=Header()
        h.stamp=self.rgb_img.header.stamp
        h.frame_id='camera_link'
        vis_img=self.bridge.cv2_to_imgmsg(vis_img,"bgr8")
        vis_img.header=h
        self.vis_pub.publish(vis_img)        
        
    def rgb_callback(self,raw_rgb_img)   
        '''Convert to cv2 format and store image.'''
        self.rgb_img=self.bridge.imgmsg_to_cv2(raw_rgb_img,"bgr8")
        self.rgb_received=True
        self.calculate_positions()
        
    def get_centers(self):
        '''Compute object centers and adds visuals to image.'''
        img=self.current_rgb_image
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
        
        return img
        
def main():
    #Define input parameters
    search_box={'lock':{'x':225,'y':100,'width':180,'height':180},
                'key':{'x':525,'y':100,'width':180,'height':180}}
    hsv={'lock':{'min':[40,int(0.3*255),int(0.6*255)],
                 'max':[150,int(0.9*255),int(0.8*255)]},
         'key':{'min':[95,int(0.18*255),int(0.6*255)],
                'max':[110,int(0.3*255),int(0.85*255)]}}
    finder=FinderPub(search_box['key'], hsv['key']['min'], hsv['key']['max'], 
                     search_box['lock'], hsv['lock']['min'], 
                     hsv['lock']['max'])
    rospy.init_node('lock_key_finder_pub',anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting Down')
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()