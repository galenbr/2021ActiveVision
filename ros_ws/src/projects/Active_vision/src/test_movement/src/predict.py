#! /usr/bin/python

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16
import numpy
import pickle
from sklearn.svm import SVC


direction_pub = rospy.Publisher('/predicted_direction', Int16, queue_size=10)

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print('data size is:')
    predict_list = []
    # for i in range(49):
    #     print(data.data[i])
    # 	predict_list[i] = data.data[i]
    predict_list = data.data
    print(predict_list)
    np_predict_list = numpy.array(predict_list)
    infile = open("/home/yash/testfoldwe/src/test_movement/src/svm_pickle2.pkl",'rb')
    clf = pickle.load(infile)
    direction = Int16()	
    print(np_predict_list.size)
    direction.data = clf.predict([np_predict_list])
    # print('direction to explore is: ',direction.data)
    direction_pub.publish(direction)	
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('predict', anonymous=True)

    rospy.Subscriber("concat_array_topic", Float32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()