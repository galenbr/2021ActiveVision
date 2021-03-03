#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/point_types.h"
#include "rrbot_pcl/cloudMsg.h"
#include "rrbot_pcl/processCloud.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "get_cloud");
    ros::NodeHandle n;

    ros::Publisher processedCloud__pub = n.advertise<sensor_msgs::PointCloud2>("debug_processed_cloud",1);

    ros::ServiceClient cloudClient = n.serviceClient<rrbot_pcl::cloudMsg>("get_cloud");
    ros::ServiceClient processClient = n.serviceClient<rrbot_pcl::processCloud>("process_cloud");

    ros::Rate r(10);
    while(ros::ok()){
        rrbot_pcl::cloudMsg cloud;
        rrbot_pcl::processCloud processedcloud;

        cloudClient.call(cloud);
    
        processedcloud.request.pc2 = cloud.response.pc2;
        if(processClient.call(processedcloud))
            processedCloud__pub.publish(processedcloud.response.pc2);

        ros::spinOnce();
        r.sleep();
    }






    ros::spin();
    return 0;
}