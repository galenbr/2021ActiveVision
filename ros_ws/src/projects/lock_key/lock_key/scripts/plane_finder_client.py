# -*- coding: utf-8 -*-
"""
ROS node for calling plane finder service.
"""
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
import lock_key_msgs.srv

class CloudClient:
    '''Calls Find Planes Service.'''
    def __init__(self):
        self.pc2_sub=rospy.Subscriber('/camera/depth/color/points',
                                      PointCloud2,self.pc2_callback)
        self.key_cloud_pub = rospy.Publisher('key_cloud', PointCloud2, 
                                            queue_size=10)
        self.lock_cloud_pub = rospy.Publisher('lock_cloud', PointCloud2, 
                                            queue_size=10)
        rospy.loginfo('Running plane finder client.')

    def call_find_planes(self,pc2_msg):
        rospy.loginfo('Waiting for FindPlanesServer.')
        rospy.wait_for_service('FindPlanesServer')
        rospy.loginfo('Found FindPlanesServer.')
        find_planes = rospy.ServiceProxy('FindPlanesServer', 
                                         lock_key_msgs.srv.FindPlanes)
        response = find_planes(pc2_msg)

        #Identify and print the min/maxes of XYZ
        key_xs=[]; key_ys=[]; key_zs=[]
        lock_xs=[]; lock_ys=[]; lock_zs=[]
        for p in pc2.read_points(response.key_cloud, field_names = ("x", "y", "z"), skip_nans=True):
            key_xs.append(p[0])
            key_ys.append(p[1])
            key_zs.append(p[2])
        for p in pc2.read_points(response.lock_cloud, field_names = ("x", "y", "z"), skip_nans=True):
            lock_xs.append(p[0])
            lock_ys.append(p[1])
            lock_zs.append(p[2])

        bounds={'key':{'min':{'x':min(key_xs),'y':min(key_ys),'z':min(key_zs)},
                       'max':{'x':max(key_xs),'y':max(key_ys),'z':max(key_zs)}},
                'lock':{'min':{'x':min(lock_xs),'y':min(lock_ys),'z':min(key_zs)},
                        'max':{'x':max(lock_xs),'y':max(lock_ys),'z':max(lock_zs)}},
                'frame':'/camera_color_optical_frame'}
        rospy.set_param('bounds',bounds)
        
        #Publish point clouds
        ii=0
        while ii<50:
            try:
                self.key_cloud_pub.publish(response.key_cloud)
                self.lock_cloud_pub.publish(response.lock_cloud)
            except:
                print 'Error publishing key or lock point cloud'
            rospy.sleep(0.2)
            ii+=1
        rospy.loginfo('End publishing of key and lock clouds.')

    def pc2_callback(self,pc2_msg):
    	'''PointCloud2 Message Callback passthrough function.'''
        # try:
        rospy.loginfo('Received pointcloud2 message.')
        self.call_find_planes(pc2_msg)
        # except:
        #     print 'Error calling find planes'
        
def main():
    rospy.init_node('plane_finder_client',anonymous=True)
    cloud_client=CloudClient()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting Down')

if __name__=='__main__':
    main()
