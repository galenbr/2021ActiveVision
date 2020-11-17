#include "ros/ros.h"
#include <tf/tf.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32.h"

float x1[3], x2[3], x3[3]; //x2 is the end effector, x3 is the elbow marker
float Orientation_Z;
float Orientation_Reference_Z; 

void Callback1(const geometry_msgs::Pose &msg1)
{
	x1[0]=(msg1.position.x); // - 0.0057;  // Adding an offset to the marker1
	x1[1]=msg1.position.y;
	x1[2]=msg1.position.z;
	for(int i= 0;i<3;i++)
	{
		printf("pose1: ");
		printf("%f\n",x1[i]);
	}
	tf::Quaternion q(
        msg1.orientation.x,
        msg1.orientation.y,
        msg1.orientation.z,
        msg1.orientation.w);
	tf::Matrix3x3 m(q);
    	double roll, pitch, yaw;
    	m.getRPY(roll, pitch, yaw);
    	Orientation_Reference_Z=yaw;
    	
}

void Callback2(const geometry_msgs::Pose &msg2)
{
	x2[0]=msg2.position.x;
	x2[1]=msg2.position.y;
	x2[2]=msg2.position.z;
	for(int i= 0;i<3;i++)
	{
		printf("pose2: ");
		printf("%f\n", x2[i]);
	}
	tf::Quaternion q(
        msg2.orientation.x,
        msg2.orientation.y,
        msg2.orientation.z,
        msg2.orientation.w);
	tf::Matrix3x3 m(q);
    	double roll, pitch, yaw;
    	m.getRPY(roll, pitch, yaw);
    	Orientation_Z=yaw;
}

void Callback3(const geometry_msgs::Pose &msg3){
	x3[0]=msg3.position.x;
	x3[1]=msg3.position.y;
	x3[2]=msg3.position.z;
	for(int i= 0;i<3;i++)
	{
		printf("pose3: ");
		printf("%f\n", x3[i]);
	}
}

int main(int argc, char **argv)
{
	float x[3];
	float elbow_marker[3];
	ros::init(argc, argv, "listener");

	ros::NodeHandle n;

	ros::Subscriber sub1 = n.subscribe("/aruco_simple/pose", 1000, Callback1);
	ros::Subscriber sub2 = n.subscribe("/aruco_simple/pose2", 1000, Callback2);
	ros::Subscriber sub3 = n.subscribe("/aruco_simple/pose3", 1000, Callback3);
	ros::Publisher pub = n.advertise<geometry_msgs::Point>("/object_position", 1000);
	ros::Publisher pub1 = n.advertise<std_msgs::Float32>("/object_orientation", 1000);

	ros::Publisher elbow_pub = n.advertise<geometry_msgs::Point>("/elbow_position", 1000);
	ros::Rate r(20);
	while(ros::ok())
	{
		for (int i=0; i<3; i++)
		{
			x[i] = x2[i]-x1[i];
			elbow_marker[i] = x3[i] - x1[i];
		}

		geometry_msgs::Point msg;
		msg.x = -x[0];
		msg.y = x[1];
		msg.z = x[2];
		std_msgs::Float32 Block_orientation;
		Block_orientation.data= Orientation_Z - Orientation_Reference_Z;

		geometry_msgs::Point elbow_msg;
		msg.x = - elbow_marker[0];
		msg.y = elbow_marker[1];
		msg.z = elbow_marker[2];

		//ROS_INFO("publishing the pose difference");

		pub.publish(msg);
		elbow_pub.publish(elbow_msg);
		pub1.publish(Block_orientation);
		for(int i= 0;i<3;i++)
		{
			printf("***POSE***: ");
			printf("%f\n", x[i]);
			printf("%f\n",elbow_marker[i]);
		}

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
