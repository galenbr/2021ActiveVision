#include <iostream>
#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <array>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ModelState.h>
#include "ros/ros.h"
#include <math.h>
// #include <move_camera.h>
#include <move_camera.cpp>
#include <testing_image_transport/image_stitching.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
///////////////////////////////////////////////////////////////
//Ransac Filtering
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>


#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/segmentation/region_growing_rgb.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include "testing_image_transport/image_stitching.h"
#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/box_clipper3D.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <Eigen/Dense>
#include <unistd.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/visualization/cloud_viewer.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/features/integral_image_normal.h>
#include <fstream>
#include <urdf/model.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// measure time
#include <chrono>



// #include <path to move_camera.cpp>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
geometry_msgs::Pose pose_to_go;
geometry_msgs::Pose saved_pose;
pcl::PointXYZ minPt, maxPt;

PointCloud temp;
PointCloud unexplored_temp;
PointCloud::Ptr exp_ptr (new PointCloud);
PointCloud::Ptr unexp_ptr (new PointCloud);

ros::NodeHandle *nhptr;
std::vector<double> graspVector;
int dir1;
int dir2;

// Some global variables
void spawn_object_model(std::string model_string1)
{
	geometry_msgs::Pose start_pose;

	start_pose.position.x = 0.35;
	start_pose.position.y = 0.0;
	start_pose.position.z = 1.0;
	// // float x = float(rand() % 100)/100;
	// // float y = sqrt(1-(x*x));
	// // Eigen::Vector3f initial, final,axis;
	// // float angle = 0;
	// // initial<<0,0,1;
	// // final<<x,y,0;
	// // axis = initial.cross(final);
	// // axis<<axis[0]/axis.norm(),axis[1]/axis.norm(),axis[2]/axis.norm();
	// // angle = acos(initial.dot(final) / (initial.norm() * final.norm()));

	// // Eigen::Quaterniond qOrientation(cos(angle/2), axis.x()*sin(angle/2), axis.y()*sin(angle/2), axis.z()*sin(angle/2));
	// // float roll = float(rand() % 100)*2*M_PI/100;

	// // For other orientation
	// float roll = 1.5707;
	// float angleDegrees;
	// std::random_device rd;  //Will be used to obtain a seed for the random number engine
	// std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
	// std::uniform_real_distribution<float> dis(0, 180);
	// angleDegrees = dis(gen);
	// angleDegrees += 45;
	// float pitch = float(angleDegrees)*2*M_PI/100;
	// float yaw = 0;
	// Eigen::Quaternionf qOrientation;
	// qOrientation = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
	// * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
	// * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
	// // For other orientation till here

	// Uncomment from below this line
	float angleDegrees;
	std::random_device rd;  //Will be used to obtain a seed for the random number engine
	std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
	std::uniform_real_distribution<float> dis(0, 90);
	angleDegrees = dis(gen);
	angleDegrees += rand()%180;
	float angle;
	angle = angleDegrees * M_PI / 180;
	angle = 175 * M_PI / 180;
	std::cout<<"spawn angle:"<<std::endl;
	std::cout<<angle<<std::endl;
	// Eigen::Quaternionf qOrientation;
	Eigen::Quaternionf qOrientation(cos(angle/2), 0, 0, sin(angle/2));
	qOrientation.normalize();	// std::cout << "Quaternion" << std::endl << q.coeffs() << std::endl;
	// Uncomment till this line

	// qOrientation.normalize();
	start_pose.orientation.x = qOrientation.x();
	start_pose.orientation.y = qOrientation.y();
	start_pose.orientation.z = qOrientation.z();
	start_pose.orientation.w = qOrientation.w();

	std::string name = "bowl";
	std::string frame = "world";
	std::string bot_namespace = "default";


	ros::ServiceClient spawn_client = nhptr->serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
	gazebo_msgs::SpawnModel spawnmodel;
	spawnmodel.request.model_name = name;
	spawnmodel.request.model_xml = model_string1;
	spawnmodel.request.robot_namespace = bot_namespace;
	spawnmodel.request.initial_pose = start_pose;
	spawnmodel.request.reference_frame = frame;

	bool A;
	std::string B;

	A,B = spawn_client.call(spawnmodel);
	// std::cout<<"bool returned"<<A<<std::endl;
	// std::cout<<"string returned"<<B<<std::endl;


}

void graspQuality_callback(const std_msgs::Float64::ConstPtr& graspQuality)
{
	//std::cout<<"graspQuality callback called"<<std::endl;
	graspVector.push_back(graspQuality->data);
	double quality;
	quality = graspQuality->data;
	//std::cout<<"graspQuality is: "<<quality<<std::endl;
}



void set_random_object_orientation(geometry_msgs::Pose pose_to_go_to){
	geometry_msgs::Pose new_pose;

	new_pose.position.x = pose_to_go_to.position.x;
	new_pose.position.y = pose_to_go_to.position.y;
	new_pose.position.z = pose_to_go_to.position.z + 1;
	new_pose.orientation.x = pose_to_go_to.orientation.x;//float(rand() % 100) * 2.0*M_PI/100.0;
	new_pose.orientation.y = pose_to_go_to.orientation.y;//float(rand() % 100) * 2.0*M_PI/100.0;
	new_pose.orientation.z = pose_to_go_to.orientation.z;//float(rand() % 100) * 2.0*M_PI/100.0 ;
	new_pose.orientation.w = pose_to_go_to.orientation.w;

	geometry_msgs::Twist new_twist;
	new_twist.linear.x = 0.0;
	new_twist.linear.y = 0.0;
	new_twist.linear.z = 0.0;
	new_twist.angular.x = 0.0;
	new_twist.angular.y = 0.0;
	new_twist.angular.z = 0.0;

	std::string name = "kinect";
	std::string frame = "world";
	gazebo_msgs::ModelState modelstate;
	modelstate.model_name = name;
	modelstate.reference_frame = frame;
	modelstate.pose = new_pose;
	modelstate.twist = new_twist;

	ros::ServiceClient state_client = nhptr->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	gazebo_msgs::SetModelState setmodelstate;
	setmodelstate.request.model_state = modelstate;
	bool A;
	std::string B;

	A,B = state_client.call(setmodelstate);
	//std::cout<<"bool returned"<<A<<std::endl;
	//std::cout<<"string returned"<<B<<std::endl;


}


void delete_object_model()
{
	std::string name = "bowl";

	ros::ServiceClient delete_client = nhptr->serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
	gazebo_msgs::DeleteModel deletemodel;
	deletemodel.request.model_name = name;

	bool A;
	std::string B;

	A,B = delete_client.call(deletemodel);
	//std::cout<<"Delete bool returned"<<A<<std::endl;
	//std::cout<<"Delete string returned"<<B<<std::endl;


}

int call_image_stitching(int index_1, int index_2){
    ros::ServiceClient client = nhptr->serviceClient<testing_image_transport::image_stitching>("image_stitching");

    testing_image_transport::image_stitching srv;
    srv.request.a = index_1;
    srv.request.c = index_2;


    if (client.call(srv))
      {
        //ROS_INFO("Successful call to service");
        return 0;
      }
      else
      {
        //ROS_ERROR("Failed to call service");
        return 1;
      }
}

float* highest_point (const PointCloud::Ptr temp, pcl::PointXYZ minimum, pcl::PointXYZ maximum )
{
  temp->points.resize (temp->width*temp->height);
  //float k = (maximum.x - minimum.x)/5;
  //float l = (maximum.y - minimum.y)/5;
	//float m = (maximum.z - minimum.z)/5;
  //float X[6] = {minimum.x, minimum.x + k , minimum.x + 2*k, minimum.x + 3*k, minimum.x + 4*k, maximum.x };
  //float Y[6] = {minimum.y, minimum.y + l , minimum.y + 2*l, minimum.y + 3*l, minimum.y + 4*l, maximum.y };
	float k = (maximum.x - minimum.x)/3;
  float l = (maximum.y - minimum.y)/3;
	float m = (maximum.z)/2;
  float X[4] = {minimum.x, minimum.x + k , minimum.x + 2*k, minimum.x + 3*k };
  float Y[4] = {minimum.y, minimum.y + l , minimum.y + 2*l, minimum.y + 3*l };
  float x0 = 0.0;
  float x1 = 0.0;
  float y0 = 0.0;
  float y1 = 0.0;
 	float highest_z = 0.0;
  float* explored_vector;
	explored_vector = (float*)malloc(9 * sizeof(float));
  int j = 0;
	for (int tx = 0; tx <9; tx++){
		explored_vector[j] = 0.0;
	}
  for (int x = 0; x < 3; x++)
  {
    for (int y= 0; y < 3; y++)
    {
      x0 = X[x];
      x1 = X[x+1];
      y0 = Y[y];
      y1 = Y[y+1];

      for (int i = 0; i< temp->points.size(); ++i)
      {
        if(temp->points[i].x >= x0 && temp->points[i].x <= x1 && temp->points[i].y >= y0 && temp->points[i].y <= y1)
        {
          if(temp->points[i].z > highest_z)
            {highest_z = temp->points[i].z;}
          explored_vector[j] = (int) lround((highest_z)/m);
        }
      }
      j++;
    }
  }

  // std::cout<<"The region 25x1 vector is"<<std::endl;
  // for(int i = 0 ; i<25 ; i++)
  // {std::cout<<explored_vector[i]<<std::endl;}

  return explored_vector;

}

void image_arr_callback(const PointCloud::Ptr& image_arr_msg)
{
	temp = *image_arr_msg;
	exp_ptr = image_arr_msg;
	//std::cout<<"Received a point cloud in callback of size:"<<exp_ptr->size()<<std::endl;
}

void unexplored_callback(const PointCloud::Ptr& image_arr_msg2)
{
	unexplored_temp = *image_arr_msg2;
	unexp_ptr = image_arr_msg2;
	//std::cout<<"Received a point cloud in callback of size:"<<unexp_ptr->size()<<std::endl;

}


void record_image_array(int which_cloud_to_stitch, int is_publish){
	int image_stitching_return;
	auto t1 = std::chrono::high_resolution_clock::now();
	image_stitching_return = call_image_stitching(which_cloud_to_stitch , is_publish);
	auto t2 = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
	//std::cout<<"record image array"<<std::endl;
	//std::cout << which_cloud_to_stitch << is_publish<<std::endl;;
	//std::cout<<"time"<<std::endl;
	//std::cout << duration<<std::endl;
}

float calc_grasp_quality(){
	return(graspVector.back());
}

void automated_training_procedure(std::string object_xml_parsed)
{

	int no_of_objects = 1;
	int no_of_data_per_object = 1;
	int no_search_dir = 8;
	int no_random_search = 5;
	int max_search_step_no = 5;
	float quality_threshold = 275; // Currently on a scale of 360 with 360 being the highest
	float grasp_quality = 0;
	std::vector<PointCloud> image_arr;
	float rec_grasp_quality[no_search_dir];
	int rec_step_no[no_search_dir];
	int dir_arr[no_of_data_per_object];
	float direction_data[400];
	int distribution;
	int exp_dir;
	dir1 = -5;
	dir2 = -5;
	int count = 0;
	float robot_theta_main;
	float robot_phi_main;
	float* explored_haf_pcd;
	//int* unexplored_haf_pcd;
	int q_dir;
	//std::cout<<"memory check"<<std::endl;
  //load trained q table
	float q_table[19683][8];
	for(int i = 0; i < 19683; i++){
		for(int j= 0; j < 8; j++){
			q_table[i][j] = 0;
		}
	}
	q_table[19681][0] = -1.19281;
	q_table[19681][1] = -1.00765;
	q_table[19681][2] = -0.183092;
	q_table[19681][3] = -0.217297;
	q_table[19681][6] = -0.196;
	q_table[19681][7] = -0.10793;

	q_table[19682][0] = -1.34051;
	q_table[19682][1] = -1.35505;
	q_table[19682][2] = -1.81913;
	q_table[19682][3] = -1.41073;
	q_table[19682][4] = -1.40149;
	q_table[19682][5] = -1.13331;
	q_table[19682][6] = -1.35411;
	q_table[19682][7] = -1.04705;

	q_table[6559][0] = 0.199;
	q_table[6559][6] = -0.295;

	q_table[6560][0] = 0.199;
	q_table[6560][3] = -0.295;
  //save file location
	move_camera camera;
	std::ofstream d_file;
	d_file.open("/home/quin/catkin_ws/result.txt");



	for(int i = 0; i < no_of_objects; i++)
	{
		for(int j = 0; j < no_of_data_per_object; j++)
		{

			d_file<<"times:"<<std::endl;
			d_file<<j<<std::endl;
			sleep(2.0);
			spawn_object_model(object_xml_parsed);
			// sleep(7.0);

	// 		move_camera robot;
			std::random_device rd;  //Will be used to obtain a seed for the random number engine
			std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
			std::uniform_int_distribution<> dis(1, 9);
			distribution = dis(gen);
      //random move 0-3 steps

			std::cout<<"chosen distribution:"<<std::endl;
			std::cout<<distribution<<std::endl;


			if (distribution<6)
			{
				camera.move_camera_to_initialpos();
				ros::AsyncSpinner spinner(1);
				//std::cout<<"------1------"<<std::endl;
				if (spinner.canStart() == true)
				{
					//std::cout<<"spinner can start has returned: "<< spinner.canStart()<<std::endl;
					spinner.start();
					record_image_array(1,1);
					sleep(7.0);
					grasp_quality = calc_grasp_quality();
					record_image_array(40,0);
				}

			}

			else if (distribution<8 && distribution > 5)
			{
				camera.move_camera_to_initialpos();
				ros::AsyncSpinner spinner(1);
				//std::cout<<"------1------"<<std::endl;
				if (spinner.canStart() == true)
				{
					//std::cout<<"spinner can start has returned: "<< spinner.canStart()<<std::endl;
					spinner.start();
					record_image_array(1,0);
					sleep(7.0);

					grasp_quality = calc_grasp_quality();
					record_image_array(40,0);
				}
				dir1 = camera.move_camera_to_random_dir(true,-1);
				if (spinner.canStart() == true)
				{
					//std::cout<<"spinner can start has returned: "<< spinner.canStart()<<std::endl;
					spinner.start();
					record_image_array(2,1);
					sleep(7.0);
					grasp_quality = calc_grasp_quality();
					record_image_array(3,0);
					record_image_array(40,0);
				}
			}


			else if (distribution<10 && distribution > 7)
			{
				camera.move_camera_to_initialpos();
				ros::AsyncSpinner spinner(1);
				//std::cout<<"------1------"<<std::endl;
				if (spinner.canStart() == true)
				{
					//std::cout<<"spinner can start has returned: "<< spinner.canStart()<<std::endl;
					spinner.start();
					record_image_array(1,0);
					sleep(7.0);
					grasp_quality = calc_grasp_quality();
					record_image_array(40,0);
				}
				dir1 = camera.move_camera_to_random_dir(true,-1);
				if (spinner.canStart() == true)
				{
					//std::cout<<"spinner can start has returned: "<< spinner.canStart()<<std::endl;
					spinner.start();
					record_image_array(2,0);
					sleep(7.0);
					grasp_quality = calc_grasp_quality();
					record_image_array(3,0);
					record_image_array(40,0);
				}
				dir2 = camera.move_camera_to_random_dir(true,-1);
				if (spinner.canStart() == true)
				{
					//std::cout<<"spinner can start has returned: "<< spinner.canStart()<<std::endl;
					spinner.start();
					record_image_array(41,1);
					sleep(7.0);
					grasp_quality = calc_grasp_quality();
					record_image_array(3,0);
					record_image_array(40,0);
				}

			}


			//std::cout<<"------2------"<<std::endl;

			int k = 0;
			grasp_quality = calc_grasp_quality();


			while (grasp_quality < quality_threshold && k < 15){

				k += 1;
				robot_theta_main = camera.return_robot_theta();
				robot_phi_main = camera.return_robot_phi();

				//function here need to determine the directions
				pcl::getMinMax3D (temp, minPt, maxPt);
		    explored_haf_pcd = highest_point(exp_ptr,minPt,maxPt);
				//pcl::getMinMax3D (unexplored_temp, minPt, maxPt);
				//unexplored_haf_pcd = highest_point(unexp_ptr, minPt,maxPt);
				int ctr_int = 0;
				int x_t = 0;
				int x_t_n = 0;
        //the probability to determine whether explore an random direction
				int exp_r = rand() % 100;
        //map the model to the vector and use the the vector to calculate the number of state
				while(ctr_int < 9){
					std::cout<<"vector check:"<<*(explored_haf_pcd + ctr_int)<<std::endl;
					if(*(explored_haf_pcd + ctr_int) > 2){
						*(explored_haf_pcd + ctr_int) = 2;
					}
					if(*(explored_haf_pcd + ctr_int) < 0){
						*(explored_haf_pcd + ctr_int) = 0;
					}
					x_t += *(explored_haf_pcd + ctr_int) * pow(3, ctr_int);
					//std::cout<<"vector check:"<<*(unexplored_haf_pcd + ctr_int)<<std::endl;
					ctr_int += 1;
				}
        //use the q_learning to find the desired dirction
				std::cout<<"vector id:"<<x_t<<std::endl;
				int q_max = -100000;
				for (int q_i = 0; q_i < 8; q_i++){
					if(q_max < q_table[x_t][q_i]){
						q_max = q_table[x_t][q_i];
						q_dir = q_i;
					}
				}

				//std::cout<<"------6------"<<std::endl;
				std::cout<<"chosen direction:"<<std::endl;
				std::cout<<q_dir<<std::endl;
				d_file<<"number of steps:"<<std::endl;
				d_file<<k<<std::endl;
				d_file<<"chosen direction:"<<std::endl;
				d_file<<q_dir<<std::endl;


				camera.move_camera_to_random_dir(false,q_dir);




				ros::AsyncSpinner spinner(1);
				while (spinner.canStart() == false)
				{

				}
				if (spinner.canStart() == true)
				{	spinner.start();
						//std::cout<<"spinner can start has returned: "<< spinner.canStart()<<std::endl;
					if (k == 0){
						record_image_array(41, 2);	// the 1 means save in temp
						sleep(7.0);
						grasp_quality = calc_grasp_quality();
					}
					else{
						record_image_array(42, 2);	// the 1 means save in temp
						sleep(7.0);
						grasp_quality = calc_grasp_quality();
					}
				}

				pcl::getMinMax3D (temp, minPt, maxPt);
		    explored_haf_pcd = highest_point(exp_ptr,minPt,maxPt);
				//pcl::getMinMax3D (unexplored_temp, minPt, maxPt);
				//unexplored_haf_pcd = highest_point(unexp_ptr, minPt,maxPt);
				ctr_int = 0;
				x_t_n = 0;
        //find the number of index of current state in the q table
				while(ctr_int < 9){
					//std::cout<<"vector check:"<<*(explored_haf_pcd + ctr_int)<<std::endl;
					if(*(explored_haf_pcd + ctr_int) > 2){
						*(explored_haf_pcd + ctr_int) = 2;
					}
					if(*(explored_haf_pcd + ctr_int) < 0){
						*(explored_haf_pcd + ctr_int) = 0;
					}
					x_t_n += *(explored_haf_pcd + ctr_int) * pow(3, ctr_int);
					//std::cout<<"vector check:"<<*(unexplored_haf_pcd + ctr_int)<<std::endl;
					ctr_int += 1;
				}
        //find the best potential reward
				float q_reward = 0;
				if (grasp_quality >= quality_threshold ){
					q_reward = 10;
				}
				else{
					q_reward = (grasp_quality - quality_threshold)/10 - 2;
				}
				q_max = -100000;
				for (int q_i = 0; q_i < 8; q_i++){
					if(q_max < q_table[x_t_n][q_i]){
						q_max = q_table[x_t_n][q_i];
					}
				}
        //update the q table
				q_table[x_t][q_dir] = q_table[x_t][q_dir] + 0.01*(q_reward + 0.1*q_max - q_table[x_t][q_dir]);

				std::cout<<"------7------Grasph Quality:"<<std::endl;
				std::cout<<grasp_quality<<std::endl;
				d_file<<"Grasph Quality:"<<std::endl;
				d_file<<grasp_quality<<std::endl;
				std::ofstream q_file;
        //save the q table to the file
				q_file.open("/home/quin/catkin_ws/q_save.txt");
				for(int x_t_h = 0; x_t_h < 19683; x_t_h++){
					for(int y_t_h= 0; y_t_h < 8; y_t_h++){
						q_file<< q_table[x_t_h][y_t_h]<<" ";
					}
					q_file<<std::endl;
				}
				q_file.close();
				free(explored_haf_pcd);

			}
			std::cout<<"--------------------8-----------------"<<std::endl;
			std::cout<<"one data collected step"<<std::endl;
			std::cout<<k<<std::endl;
			std::cout<<"times"<<std::endl;
			std::cout<<j<<std::endl;
			d_file<<"one data collected step: "<<std::endl;
			d_file<<k<<std::endl;


      //Uncomment the blow line of code if you want to see the pointcloud data
			//record_image_array(6,0);
			delete_object_model();
			sleep(6);

	}

}
	d_file.close();
}



int main(int argc, char** argv)
{
ros::init(argc, argv, "training_data_gen_node");
ros::NodeHandle nh;
nhptr = &nh;

ros::Subscriber image_arr_sub = nh.subscribe("/image_arr_topic", 1000, image_arr_callback);
ros::Subscriber unexplored_sub = nh.subscribe("/unexplored_topic", 1000, unexplored_callback);

ros::Subscriber grasp_quality_sub = nh.subscribe("/graspQuality_topic", 1000, graspQuality_callback);


// int image_stitching_return;

char c;
std::string model_string;
ifstream file("/home/quin/catkin_ws/src/object_models/models/urdf/sugar_box.urdf");
file>>noskipws;
while ( file >> c ) model_string += c;
file.close();

automated_training_procedure(model_string);

ros::shutdown();

}


/// index_1 = 0 :: Do nothing
/// index_1 = 1 :: Store point cloud in storedPCL
/// index_1 = 2 :: Use stored point cloud in storedPCL
/// index_1 = 3 :: Store point cloud in storedPCL_new
/// index_1 = 4 :: Use stored point cloud in storedPCL_new
/// index_1 = 5 :: Initialize new empty array

/// index_2 = 0 :: Dont add to image array
/// index_2 = 1 :: Add to image array
