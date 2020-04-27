#include <ros/ros.h>
// Services
#include "lock_key/imgCapture.h"
#include "lock_key/findKey.h"
#include "moveit_planner/MovePose.h"
#include "moveit_planner/MoveCart.h"
#include "franka_gripper_gazebo/GripMsg.h"
// Publish Marker
#include <visualization_msgs/Marker.h>
// Transform Listener
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

void wait_for_service(int32_t timeout){
    ros::service::waitForService("move_to_pose",timeout);
    ros::service::waitForService("imgCapture", timeout);
    ROS_INFO("SERVICES READY");
}

int main(int argc, char **argv){

    ros::init(argc, argv, "control");
    ros::NodeHandle n;

    //Waiting for services to be available
    int32_t timeout = 1000;
    wait_for_service(timeout);

    // Transform Listener

    tf::TransformListener listener;

    //Declaring Clients
    ros::ServiceClient imgCaptureClient = n.serviceClient<lock_key::imgCapture>("imgCaptureServer");
    ros::ServiceClient findKeyClient = n.serviceClient<lock_key::findKey>("findKeyServer");
    ros::ServiceClient movePoseClient = n.serviceClient<moveit_planner::MovePose>("move_to_pose");
    ros::ServiceClient moveCartClient = n.serviceClient<moveit_planner::MoveCart>("cartesian_move");
    ros::ServiceClient gripperClient = n.serviceClient<franka_gripper_gazebo::GripMsg>("gazebo_franka_grip");

    // Client Objects
    lock_key::imgCapture reqImg;
    lock_key::findKey keyImg;
    moveit_planner::MovePose pose;
    moveit_planner::MoveCart cart;
    franka_gripper_gazebo::GripMsg grip;

    //Client Calls
    imgCaptureClient.call(reqImg);
    keyImg.request.pc2 = reqImg.response.pc2;
    ROS_INFO("IMAGE RECEIVED");
    findKeyClient.call(keyImg);
    ROS_INFO("Key found");

    //Transforming Key position to robot frame
    geometry_msgs::PointStamped keypose;
    listener.transformPoint("panda_link0",keyImg.response.p,keypose);
    
    // Visualization and Debugging
    ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("debugPC2", 1);
    ros::Publisher centroid_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);


    visualization_msgs::Marker marker;

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = .005;
    marker.scale.y = .005;
    marker.scale.z = .005;
    marker.color.a = 1;
    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 1;
    marker.pose.orientation.w = 1;
    marker.header.frame_id="/panda_camera_optical_link";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.pose.position.x = keyImg.response.p.point.x;
    marker.pose.position.y = keyImg.response.p.point.y;
    marker.pose.position.z = keyImg.response.p.point.z;

    // ROS_INFO_STREAM(marker.pose);

    int count =0;
     while (count<50)
     {
        cloud_pub.publish(keyImg.response.pc2);
        centroid_pub.publish(marker);
        ROS_INFO("KEY Published");
        count += 1;
     }
    //Manipulating position so as to create pre-grasp pose
    keypose.point.x -= 0.05;
    keypose.point.y += 0.01;
    keypose.point.z -= 0.025;
    
    //Moving to pre-grasp pose
    pose.request.val.position = keypose.point;
    pose.request.val.orientation.x = 0.7071068; //-0.4082483;
    pose.request.val.orientation.y = 0.0; //0.8164966;
    pose.request.val.orientation.z = 0.0; //-0.4082483;
    pose.request.val.orientation.w = 0.7071068;
    pose.request.execute = true;
    grip.request.force = 1000.0;
    gripperClient.call(grip);
    movePoseClient.call(pose);

    //Grasp
    // Ideally we would first do cartesian move and then grasp, however the orientations are not working well w/o the hardware setup
    // geometry_msgs::Pose p;
    // p.position.y = -0.01;
    // p.orientation = pose.request.val.orientation;
    // cart.request.val.push_back(p);
    // cart.request.execute = true;
    // moveCartClient.call(cart);
    ros::Duration(2).sleep();
    ROS_INFO("Gripping");
    grip.request.force = -1000.0;
    gripperClient.call(grip);
    ROS_INFO("Gripped");

    
    ros::spin();

    return 0;
}