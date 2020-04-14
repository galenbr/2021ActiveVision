#include <ros/ros.h>
// Services
#include <lock_key/imgCapture.h>
#include <lock_key/findKey.h>
// Publish Marker
#include <visualization_msgs/Marker.h>

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

    ros::ServiceClient imgCaptureClient = n.serviceClient<lock_key::imgCapture>("imgCaptureServer");
    ros::ServiceClient findKeyClient = n.serviceClient<lock_key::findKey>("findKeyServer");

    lock_key::imgCapture reqImg;
    lock_key::findKey keyImg;

    imgCaptureClient.call(reqImg);
    keyImg.request.pc2 = reqImg.response.pc2;
    ROS_INFO("IMAGE RECEIVED");
    findKeyClient.call(keyImg);
    ROS_INFO("Key found");

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
    marker.pose.position.x = keyImg.response.p.x;
    marker.pose.position.y = keyImg.response.p.y;
    marker.pose.position.z = keyImg.response.p.z;

    ROS_INFO_STREAM(marker.pose);


    while (true)
    {
        cloud_pub.publish(keyImg.response.pc2);
        centroid_pub.publish(marker);
        //ROS_INFO("KEY Published");
    }
    
    ros::spin();
    





    return 0;
}