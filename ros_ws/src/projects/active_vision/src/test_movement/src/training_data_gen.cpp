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
//#include <move_camera.h>
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


PointCloud temp;
PointCloud unexplored_temp;
ros::NodeHandle *nhptr;
std::vector<double> graspVector;

// Some global variables
void spawn_object_model(std::string model_string1)
{
	geometry_msgs::Pose start_pose;

	start_pose.position.x = 0.35;
	start_pose.position.y = 0.0;
	start_pose.position.z = 1.08;
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
	// float roll = 1.5707;
	// float pitch = float(rand() % 100)*2*M_PI/100;
	// float yaw = 0;
	// Eigen::Quaternionf qOrientation;
	// qOrientation = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
	// * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
	// * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

	float angle = M_PI;
	// Eigen::Quaternionf qOrientation;
	Eigen::Quaternionf qOrientation(cos(angle/2), 0, 0, sin(angle/2));
	qOrientation.normalize();	// std::cout << "Quaternion" << std::endl << q.coeffs() << std::endl;
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
	std::cout<<"graspQuality callback called"<<std::endl;
	graspVector.push_back(graspQuality->data);
	double quality;
	quality = graspQuality->data;
	std::cout<<"graspQuality is: "<<quality<<std::endl;
}



void set_random_object_orientation(){
	geometry_msgs::Pose new_pose;

	new_pose.position.x = 0.35;
	new_pose.position.y = 0.0;
	new_pose.position.z = 1.0;
	new_pose.orientation.x = float(rand() % 100) * 2.0*M_PI/100.0;
	new_pose.orientation.y = float(rand() % 100) * 2.0*M_PI/100.0;
	new_pose.orientation.z = float(rand() % 100) * 2.0*M_PI/100.0 ;
	new_pose.orientation.w = 0.0;

	geometry_msgs::Twist new_twist;
	new_twist.linear.x = 0.0;
	new_twist.linear.y = 0.0;
	new_twist.linear.z = 0.0;
	new_twist.angular.x = 0.0;
	new_twist.angular.y = 0.0;
	new_twist.angular.z = 0.0;

	std::string name = "bowl";
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
	std::cout<<"bool returned"<<A<<std::endl;
	std::cout<<"string returned"<<B<<std::endl;


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
	std::cout<<"Delete bool returned"<<A<<std::endl;
	std::cout<<"Delete string returned"<<B<<std::endl;


}

int call_image_stitching(int index_1, int index_2){
    ros::ServiceClient client = nhptr->serviceClient<testing_image_transport::image_stitching>("image_stitching");

    testing_image_transport::image_stitching srv;
    srv.request.a = index_1;
    srv.request.c = index_2;


    if (client.call(srv))
      {
        ROS_INFO("Successful call to service");
        return 0;
      }
      else
      {
        ROS_ERROR("Failed to call service");
        return 1;
      }
}

void image_arr_callback(const PointCloud::ConstPtr& image_arr_msg)
{
	temp = *image_arr_msg;
	std::cout<<"Received a point cloud in callback of size:"<<temp.size()<<std::endl;
}

void unexplored_callback(const PointCloud::ConstPtr& image_arr_msg2)
{
	unexplored_temp = *image_arr_msg2;
	std::cout<<"Received a point cloud in callback of size:"<<unexplored_temp.size()<<std::endl;

}

void record_image_array(int which_cloud_to_stitch, int is_publish){
	int image_stitching_return;
	image_stitching_return = call_image_stitching(which_cloud_to_stitch , is_publish);
}

float calc_grasp_quality(){
	// static int i = 0;
	// if (i == 0){
	// 	i++;
	// 	return 8;
	// }
	// else{
	// 	i = 0;
	// 	return 12;
	// }
	return(graspVector.back());
}

void automated_training_procedure(std::string object_xml_parsed)
{

	int no_of_objects = 1;
	int no_of_data_per_object = 12;
	int no_search_dir = 8;
	int no_random_search = 5;
	int max_search_step_no = 5;
	float quality_threshold = 230; // Currently on a scale of 360 with 360 being the highest
	float grasp_quality = 0;
	std::vector<PointCloud> image_arr;
	float rec_grasp_quality[no_search_dir];
	int rec_step_no[no_search_dir];
	int dir_arr[no_of_data_per_object];
	int exp_dir;

	move_camera robot;

	for(int i = 0; i < no_of_objects; i++)
	{
		for(int j = 0; j < no_of_data_per_object; j++)
		{
			spawn_object_model(object_xml_parsed);
			sleep(2.0);
			delete_object_model();
			sleep(2.0);
			spawn_object_model(object_xml_parsed);

			//set_random_object_orientation();
			robot.move_camera_to_initialpos();
			sleep(4.0);
			ros::AsyncSpinner spinner(1);
			std::cout<<"------1------"<<std::endl;
  			if (spinner.canStart() == true)
  			{
  				std::cout<<"spinner can start has returned: "<< spinner.canStart()<<std::endl;
  				spinner.start();
				auto t1 = std::chrono::high_resolution_clock::now();
				record_image_array(1,1);
				auto t2 = std::chrono::high_resolution_clock::now();
				auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
				std::cout<<"record image array"<<std::endl;
    		std::cout << duration;
				sleep(4.0);
				std::cout<<"before calling calc_grasp_quality function"<<std::endl;
				auto t1 = std::chrono::high_resolution_clock::now();
				grasp_quality = calc_grasp_quality();
				auto t2 = std::chrono::high_resolution_clock::now();
				auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
				std::cout<<"calc grasp quality"<<std::endl;
				std::cout << duration;
  			}
			std::cout<<"------2------"<<std::endl;

			if(grasp_quality > quality_threshold)
			{
				j -= 1;
				auto t1 = std::chrono::high_resolution_clock::now();
				record_image_array(5,0);
				auto t2 = std::chrono::high_resolution_clock::now();
				auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
				std::cout<<"larger than threshold"<<std::endl;
    		std::cout << duration;
				continue;
			}
			else
			{	std::cout<<"img arr pushback before"<<std::endl;
				image_arr.push_back(temp);
				std::string s1 = std::to_string(j);
				std::string s2 = "Explored_PointCloud";
				std::string s3 = "Unexplored_PointCloud";
				std::string s6 = ".pcd";
				std::string s4 = s2+s1+s6;
				std::string s5 = s3+s1+s6;
				auto t1 = std::chrono::high_resolution_clock::now();
				pcl::io::savePCDFileASCII (s4, temp);
				pcl::io::savePCDFileASCII (s5, unexplored_temp);
				auto t2 = std::chrono::high_resolution_clock::now();
				auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
				std::cout<<"save PDF file"<<std::endl;
    		std::cout << duration;
				std::cout<<"img arr pushback after"<<std::endl;

			}
			std::cout<<"------3------"<<std::endl;


			for (int k=0; k<no_search_dir ; k++)
			{
				// robot.move_camera_to_random_dir(false,4);
				// sleep(10.0);
				// robot.move_camera_to_random_dir(false,2);
				// sleep(10.0);
				// robot.move_camera_to_random_dir(false,2);
				// sleep(10.0);
				// robot.move_camera_to_random_dir(false,2);
				// sleep(10.0);
				// robot.move_camera_to_random_dir(false,1);
				// sleep(10.0);
				// robot.move_camera_to_random_dir(false,3);
				// sleep(10.0);

				// std::cout<<"doneeeee "<<std::endl;

				auto t1 = std::chrono::high_resolution_clock::now();
				robot.move_camera_to_random_dir(false,k);
				auto t2 = std::chrono::high_resolution_clock::now();
				auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
				std::cout<<"move camera random"<<std::endl;
    		std::cout << duration;
				sleep(4.0);
				std::cout<<"------4------"<<std::endl;
				std::cout<<"k is: "<<k<<std::endl;
				while (spinner.canStart() == false)
				{

				}
				if (spinner.canStart() == true)
				{
					std::cout<<"spinner can start has returned: "<< spinner.canStart()<<std::endl;
					spinner.start();
					auto t1 = std::chrono::high_resolution_clock::now();
					record_image_array(2,0);//// Use the initial_pos point cloud and update that
					auto t2 = std::chrono::high_resolution_clock::now();
					auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
					std::cout<<"record 6"<<std::endl;
	    		std::cout << duration;
					sleep(4.0);

				}

				std::cout<<"------6------"<<std::endl;

				while (spinner.canStart() == false)
				{

				}
				if (spinner.canStart() == true)
				{	spinner.start();
	  			std::cout<<"spinner can start has returned: "<< spinner.canStart()<<std::endl;
					auto t1 = std::chrono::high_resolution_clock::now();
					record_image_array(3, 0); //// Camera hasn't moved anywhere. So the updated point cloud would be the same as the last step. This step just stores the Pointcloud in storedPcl_new.
					auto t2 = std::chrono::high_resolution_clock::now();
					auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
					std::cout<<"record 7"<<std::endl;
	    		std::cout << duration;
	 				sleep(3.0);
					auto t1 = std::chrono::high_resolution_clock::now();
	 				grasp_quality = calc_grasp_quality();
					auto t2 = std::chrono::high_resolution_clock::now();
					auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
					std::cout<<"calc_grasp_quality 7"<<std::endl;
	    		std::cout << duration;
				}
				std::cout<<"------7------"<<std::endl;

				if(grasp_quality > quality_threshold)
				{
					rec_grasp_quality[k] = grasp_quality;
					rec_step_no[k] = 1;
					auto t1 = std::chrono::high_resolution_clock::now();
					robot.move_camera_to_initialpos();
					auto t2 = std::chrono::high_resolution_clock::now();
					auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
					std::cout<<"move initial"<<std::endl;
	    		std::cout << duration;
					auto t1 = std::chrono::high_resolution_clock::now();
					grasp_quality = calc_grasp_quality();
					auto t2 = std::chrono::high_resolution_clock::now();
					auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
					std::cout<<"move initial calc"<<std::endl;
	    		std::cout << duration;
					continue;
				}
				std::cout<<"------8------"<<std::endl;


				for(int m = 0 ; m < no_random_search; m++)
				{
					int u = 0;
					// reset point cloud to previous direction
					for (int n =0; n < max_search_step_no; n++)
					{
						std::cout<<"m is: "<<m<<std::endl;
						std::cout<<"u is: "<<u<<std::endl;
						std::cout<<"k is: "<<k<<std::endl;


						std::cout<<"------9------"<<std::endl;

						u++;
						// if(rec_step_no[k]<n)
						// 	{break;}
						int isPossible;
						isPossible = robot.move_camera_to_random_dir(true, -1);
						if (isPossible == 0)
							{u--;
							n--;
							continue;}
						sleep(4.0);
						std::cout<<"------10------"<<std::endl;

						while (spinner.canStart() == false)
						{

						}
						if (spinner.canStart() == true)
						{	spinner.start();
							if(u <= 1)
							{
								record_image_array(41, 0);	// the 1 means save in temp
								sleep(4.0);
								grasp_quality = calc_grasp_quality();
							}
							if(u > 1)
							{
								record_image_array(42, 0);	// the 1 means save in temp
								sleep(5.0);
								grasp_quality = calc_grasp_quality();
							}

						}
						std::cout<<"------11------"<<std::endl;

						if(grasp_quality > quality_threshold || u == max_search_step_no)
						{
							rec_grasp_quality[k] = grasp_quality;
							rec_step_no[k] = u;
							cout << "u = " << u << endl;
							break;
							std::cout<<"grasp threshold reached / u maxed out"<<std::endl;

						}


						// while (spinner.canStart() == false)
						// {

						// }
						// if (spinner.canStart() == true)
						// {	spinner.start();
						// 	record_image_array(6,0); /// Initialize new empty array for the next data
						// 	sleep(250);
						// }

					}

					robot.move_camera_to_initialpos();
					sleep(4.0);
					robot.move_camera_to_random_dir(false, k);  /// Camera needs to be at that particular direction before starting exploration
					sleep(4.0);
					std::cout<<"------12------"<<std::endl;

				}

				robot.move_camera_to_initialpos();
				std::cout<<"------13------"<<std::endl;


			}

			/* Initialize min step no to max search step no */
			int min_step_no = max_search_step_no;
			float max_grasp_quality = 0;

			for(int k =0; k < no_search_dir ; k++)
			{
				if ((rec_step_no[k] < min_step_no) || ( rec_step_no[k] == min_step_no && rec_grasp_quality[k] > max_grasp_quality))
				{
					min_step_no = rec_step_no[k];
					max_grasp_quality = rec_grasp_quality[k];
					exp_dir = k;
					std::cout<<"best direction for exploration is: "<<exp_dir<<std::endl;
					std::cout<<"J is: "<<j<<std::endl;
				}
			}

			dir_arr[j] = exp_dir;

			for (int m = 0; m<j+1 ; m++)
			{
				std::cout<<"Listing all the directions:: "<<dir_arr[m]<<std::endl;
			}

			// while (spinner.canStart() == false)
			// {

			// }
			// if (spinner.canStart() == true)
			// {	spinner.start();
			// 	record_image_array(6,0); /// Initialize new empty array for the next data
			// 	sleep(6500);
			// }

			delete_object_model();
			sleep(3.0);

		}

		ofstream myfile ("direction_data.txt");
		if (myfile.is_open())
		{
			myfile << "This is a line.\n";
			for(int count = 0; count < 10; count ++){
				myfile << dir_arr[count] << " " ;
			}
			myfile.close();
		}

	}

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
ifstream file("/home/yash/testfoldwe/src/object_models/models/urdf/sugar_box.urdf");
file>>noskipws;
while ( file >> c ) model_string += c;
file.close();

automated_training_procedure(model_string);


// ros::Subscriber image_arr_sub = nh.subscribe("/image_arr_topic", 1000, image_arr_callback);

// spawn_object_model(nh, model_string);

// sleep(10.0);

// delete_object_model(nh);


// move_camera robot;
// robot.move_camera_to_initialpos();
// image_stitching_return = call_image_stitching(nh,0,0);
// robot.move_camera_to_random_dir(0,4);
// image_stitching_return = call_image_stitching(nh,1,1);
// robot.move_camera_to_random_dir(0,6);
// image_stitching_return = call_image_stitching(nh,0);
// robot.move_camera_to_random_dir(0,2);
// image_stitching_return = call_image_stitching(nh,5);


//  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
// p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

//set_new_object_state(nh,5);

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