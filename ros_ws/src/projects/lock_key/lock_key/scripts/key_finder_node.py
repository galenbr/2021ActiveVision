# -*- coding: utf-8 -*-
"""
ROS node for lock and key segmentation and centroid finding
"""
import cv2
import image_utils
import numpy as np
import rospy
import image_geometry
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError

class FinderPub:
    '''Finds lock and key from RGBD data.'''
    def __init__(self, key_min_hsv, key_max_hsv, lock_min_hsv, lock_max_hsv):
        self.header=None
        self.key_search_box=None
        self.key_min_hsv=key_min_hsv
        self.key_max_hsv=key_max_hsv
        self.key_center=None
        self.key_center_depth=None
        self.lock_search_box=None
        self.lock_min_hsv=lock_min_hsv
        self.lock_max_hsv=lock_max_hsv
        self.lock_center=None
        self.lock_center_depth=None
        self.rgb_camera=image_geometry.PinholeCameraModel()
        self.rgb_camera_info_received=False
        self.rgb_image=None
        self.rgb_received=False
        self.depth_array=None
        self.depth_received=False
        #ROS Subscribers and Publishers
        self.bridge=CvBridge()
        self.rgb_sub=rospy.Subscriber('/camera/color/image_raw',
                                      Image,self.rgb_callback)
        self.depth_sub=rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',
                                        #/camera/depth/image_rect_raw
                                        #/depth_registered/image_rect
                                        Image,self.depth_callback)
        self.rgb_info=rospy.Subscriber('/camera/color/camera_info',
                                       CameraInfo,self.rgb_info_callback)
        self.vis_pub = rospy.Publisher('finder_image', Image, queue_size=10)
        self.key_pub = rospy.Publisher('key_point', PointStamped, 
                                       queue_size=10)
        self.lock_pub = rospy.Publisher('lock_point', PointStamped, 
                                        queue_size=10)
        rospy.loginfo('Running lock and key finder node.')

    def calculate_positions(self,img):
        '''Calculates centers of lock and key.'''
        # Get lock and key centers (in RGB image) in terms of pixels
        # RGB and Depth are registered so pixel locations are the same
        vis_img=self.get_centers(img)
        #Lock
        try:
            # Get corresponding depth values (in meters)       
            self.lock_center_depth=self.depth_array[self.lock_center[1],
                                                    self.lock_center[0]]
            # Get normalized XYZ displacement (in camera_link)
            lock_disp=self.rgb_camera.projectPixelTo3dRay(self.lock_center)
            #Get scale from depth
            lock_scale=self.lock_center_depth/lock_disp[2]/1000.0
            #Convert to meters
            lock_disp=lock_scale*np.array(lock_disp)
            # Publish center
            self.publish_lock_point(lock_disp)
        except TypeError:
            rospy.loginfo('Lock Not Found in Frame')
        #Key
        try:
            self.key_center_depth=self.depth_array[self.key_center[1],
                                                   self.key_center[0]]
            # Get normalized XYZ displacement (in camera_link)
            key_disp=self.rgb_camera.projectPixelTo3dRay(self.key_center)
            #Get scale from depth
            key_scale=self.key_center_depth/key_disp[2]/1000.0
            #Convert to meters
            key_disp=key_scale*np.array(key_disp)
            # Publish center
            self.publish_key_point(key_disp)
        except TypeError:
            rospy.loginfo('Key Not Found in Frame')

        #Publish visualization image
        self.publish_vis(vis_img)

    def depth_callback(self,raw_depth_img):
        '''Converts depth image to np.array with actual depth.'''
        try:
            #Convert to cv2 format
            depth_img=self.bridge.imgmsg_to_cv2(raw_depth_img, 
                                                desired_encoding="passthrough")
            #Convert the depth image to a Numpy array (with actual depth values)
            self.depth_array = np.array(depth_img, dtype=np.float32)
            self.depth_received=True
        except CvBridgeError, e:
            print e

    def get_centers(self,img):
        '''Compute object centers and adds visuals to image.'''
        #Update img for key
        img,self.key_center=image_utils.color_change(img,
                                                     self.key_search_box,
                                                     self.key_min_hsv,
                                                     self.key_max_hsv,
                                                     new_pixel=[255,0,0], #Blue
                                                     centroid_pixel=[0,0,255]) #Red 
        #Update img for lock
        img,self.lock_center=image_utils.color_change(img,
                                                     self.lock_search_box,
                                                     self.lock_min_hsv,
                                                     self.lock_max_hsv,
                                                     new_pixel=[0,255,0], #Green
                                                     centroid_pixel=[255,0,255]) #Pink
        
        #Add search box to img for key
        img=image_utils.add_quad(img,self.key_search_box,color=[0,0,255]) #Red
        #Add search box to img for lock
        img=image_utils.add_quad(img,self.lock_search_box,color=[255,0,255]) #Pink
        
        return img

    def publish_key_point(self,key_disp):
        '''Publish key position in terms of camera_color_optical_frame.'''
        #Build key msg
        key_msg=PointStamped()
        key_msg.header=self.header
        key_msg.point.x=key_disp[0]
        key_msg.point.y=key_disp[1]
        key_msg.point.z=key_disp[2]
        #Publish message
        self.key_pub.publish(key_msg)

    def publish_lock_point(self,lock_disp):
        '''Publish lock position in terms of camera_color_optical_frame.'''
        #Build lock msg
        lock_msg=PointStamped()
        lock_msg.header=self.header
        lock_msg.point.x=lock_disp[0]
        lock_msg.point.y=lock_disp[1]
        lock_msg.point.z=lock_disp[2]
        #Publish message
        self.lock_pub.publish(lock_msg)

    def publish_vis(self,vis_img):
        '''Define and publish visualization image'''
        vis_img=self.bridge.cv2_to_imgmsg(vis_img,"bgr8")
        vis_img.header=self.header
        self.vis_pub.publish(vis_img)        

    def rgb_callback(self,raw_rgb_img):   
        '''Convert to cv2 format and store image.'''
        self.header=raw_rgb_img.header
        self.rgb_img=self.bridge.imgmsg_to_cv2(raw_rgb_img,desired_encoding="bgr8")
        if self.rgb_img is not None:
            self.rgb_received=True

        if (self.rgb_received and self.depth_received and self.rgb_camera_info_received) and \
           (self.rgb_img is not None) and (self.depth_array is not None):
            self.calculate_positions(self.rgb_img)

    def rgb_info_callback(self,rgb_cam_info):   
        '''Store camera info. in Pinhole Model.'''
        if not self.rgb_camera_info_received:
            self.rgb_camera.fromCameraInfo(rgb_cam_info)
            self.rgb_camera_info_received=True
            rospy.loginfo('RGB Camera Info. Received')

        # Convert object bounds from meters (in /camera_color_optical_frame)
        # to pixels using Pinhole model
        bounds=rospy.get_param('bounds')

        # This is for troublshooting
        # bounds={'key':{'min':{'x':0.1,'y':-0.08,'z':0.4},
        #                'max':{'x':0.2,'y':0.0,'z':0.45}},
        #        'lock':{'min':{'x':-0.12,'y':-0.08,'z':0.4},
        #                'max':{'x':-0.05,'y':0.0,'z':0.45}},
        #        'frame':'/camera_color_optical_frame'}

        #TODO: Incorporate point transformation here. Instantiate
        # PointStamped objects in init.

        key_min=(bounds['key']['min']['x'],
                 bounds['key']['min']['y'],
                 bounds['key']['min']['z'])
        key_max=(bounds['key']['max']['x'],
                 bounds['key']['max']['y'],
                 bounds['key']['max']['z'])
        lock_min=(bounds['lock']['min']['x'],
                  bounds['lock']['min']['y'],
                  bounds['lock']['min']['z'])
        lock_max=(bounds['lock']['max']['x'],
                  bounds['lock']['max']['y'],
                  bounds['lock']['max']['z'])
        key_min_pix=self.rgb_camera.project3dToPixel(key_min)
        key_max_pix=self.rgb_camera.project3dToPixel(key_max)
        lock_min_pix=self.rgb_camera.project3dToPixel(lock_min)
        lock_max_pix=self.rgb_camera.project3dToPixel(lock_max)

        self.key_search_box={'min':{'x':int(key_min_pix[0]),'y':int(key_min_pix[1])},
                             'max':{'x':int(key_max_pix[0]),'y':int(key_max_pix[1])}}

        self.lock_search_box={'min':{'x':int(lock_min_pix[0]),'y':int(lock_min_pix[1])},
                              'max':{'x':int(lock_max_pix[0]),'y':int(lock_max_pix[1])}}
        
def main():
    rospy.init_node('lock_key_finder_pub',anonymous=True)
    #Define input parameters
    hsv={'lock':{'min':[40,int(0.3*255),int(0.6*255)],
                 'max':[150,int(0.9*255),int(0.8*255)]},
         'key':{'min':[95,int(0.18*255),int(0.6*255)],
                'max':[110,int(0.3*255),int(0.85*255)]}}
    finder=FinderPub(hsv['key']['min'], hsv['key']['max'], 
                     hsv['lock']['min'], hsv['lock']['max'])
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting Down')
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()
