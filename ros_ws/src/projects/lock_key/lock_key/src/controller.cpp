#include <ros/ros.h>
#include <lock_key/imgCapture.h>
#include <lock_key/findKey.h>

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

    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("debugPC2", 5);
    while (true)
    {
        pub.publish(keyImg.response.pc2);
        ROS_INFO("KEY Published");
    }
    
    ros::spin();
    





    return 0;
}