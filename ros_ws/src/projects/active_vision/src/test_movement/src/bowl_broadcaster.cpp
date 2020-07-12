#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
ros::init(argc, argv, "bowl_broadcaster");
while(ros::ok())
{
static tf::TransformBroadcaster br;
tf::Transform transform;
transform.setOrigin( tf::Vector3(1.0, 0.0, 1.1));
tf::Quaternion q;
q.setRPY(0, 0, 0);
transform.setRotation(q);
br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "world", "bowl_link"));
}

}