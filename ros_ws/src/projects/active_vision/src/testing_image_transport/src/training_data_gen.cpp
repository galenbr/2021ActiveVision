#include <iostream>
#include <stdio.h>

using namespace std;

// Some global variables


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

for(int i = 0; i < no_of_objects; i++)
{
	for(int j = 0; j < no_of_data_per_object; j++)
	{
		spawn_object_model(i);
		move_camera_to_initialpos();
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
					move_camera_to_random_dir();
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
			if ((rec_step_no[k] < min_step_no) || ( rec_step_no[k] == min_step_no && rec_grasp_quality[k] > max_grasp_quality)
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