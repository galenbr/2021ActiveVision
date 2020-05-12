#include <iostream>
#include <stdio.h>
#include <path to move_camera.cpp>

using namespace std;

// Some global variables

ros::NodeHandle nh;

void spawn_object_model(int &i){
	geometry_msgs::Pose start_pose;

	start_pose.position.x = 1.0;
	start_pose.position.y = 0.0;
	start_pose.position.z = 1.1;
	start_pose.orientation.x = 0.0;
	start_pose.orientation.y = 0.0;
	start_pose.orientation.z = float(rand() % 100) * 2.0*M_Pi/100.0 ;
	start_pose.orientation.w = 0.0;

	geometry_msgs::Twist start_twist;
	start_twist.linear.x = 0.0;
	start_twist.linear.y = 0.0;
	start_twist.linear.z = 0.0;
	start_twist.angular.x = 0.0;
	start_twist.angular.y = 0.0;
	start_twist.angular.z = 0.0;

	gazebo::ModelState modelstate;
	modelstate.model_name = (std::string) "my_robot";
	modelstate.reference_frame = (std::string) "world";
	modelstate.pose = start_pose;
	modelstate.twist = start_twist;

	ros::ServiceClient client = nh.serviceClient<gazebo::SetModelState>("/gazebo/set_model_state");
	gazebo::SetModelState setmodelstate;
	setmodelstate.request.model_state = modelstate;
	client.call(setmodelstate);

}

void automated_training_procedure()
{

	int no_of_objects;
	int no_of_data_per_object;
	int no_search_dir;
	int no_random_search;
	int max_search_step_no;
	float quality_threshold;
	float image_arr[];
	float rec_grasp_quality[];
	int rec_step_no[];

	move_camera robot;

	for(int i = 0; i < no_of_objects; i++)
	{
		for(int j = 0; j < no_of_data_per_object; j++)
		{
			spawn_object_model(i);
			robot.move_camera_to_initialpos();
			temp = record_image_array();
			int grasp_quality = calc_grasp_quality();
			if(grasp_quality > quality_threshold)
			{
				j -= 1;
				continue;
			}
			else
			{
				image_arr[j] = temp;
			}

			for (int k=0; k<no_search_dir ; k++)
			{
				move_camera_to_direction_k();
				int grasp_quality = calc_grasp_quality();
				if(grasp_quality > quality_threshold)
				{
					rec_grasp_quality[k] = grasp_quality;
					rec_step_no[k] = 1;
					continue;
				}

				int u = 0;
				for(int m = 0 ; m < no_random_search; m++)
				{
					for (int n =0; n < max_search_step_no; n++)
					{	
						u++;
						if(rec_step_no[k]<n)
							{break;}
						robot.move_camera_to_random_dir();
						temp = record_image_array();
						int grasp_quality = calc_grasp_quality();
						if(grasp_quality > quality_threshold || u == max_search_step_no)
						{
							rec_grasp_quality[k] = grasp_quality;
							rec_step_no[k] = u;
							break;
						}
					}
				}
			}
			
			/* Initialize min step no to max search step no */
			int min_step_no = max_search_step_no;


			float max_grasp_quality = 0;

			for(int k =0; k < no_search_dir ; k++)
			{
				if ((rec_step_no[k] < min_step_no) || ( rec_step_no[k] == min_step_no && rec_grasp_quality[k] > max_grasp_quality))
				{
					min_step_no = rec_grasp_quality[k];
					max_grasp_quality = rec_grasp_quality[k];
					int exp_dir = k;
				}
			}

			dir_arr[j] = exp_dir;
		}
	}
}



int main()
{

automated_training_procedure();

}