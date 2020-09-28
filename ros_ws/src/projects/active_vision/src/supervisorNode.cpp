#include <ros/ros.h>
#include <iostream>
#include "active_vision/testingModel.h"

#define TIMEOUT 10

void modifyTargetPosition(float *pos){
	pos[0]+=0.1;
	pos[1]+=0.1;
}

int main (int argc, char** argv){
	ros::init (argc, argv, "Supervisor_Node");
 	ros::NodeHandle nh;

 	environment kinectControl = environment(&nh);

 	int stepsTaken = 0;
 	float target_position[6] = {0.25, 0.0, 1.75, 0.0, 0.55, 0.0};

 	while(stepsTaken < TIMEOUT){

 		//Get new data
 		testKinectRead(kinectControl);

 		//Fuse with old data

 		//Grasp synthesis

 		//Grasp checking

 			//Exit if grasp good/timeout exceeded

 		//Build model

 		//Get instruction from optimization policy
 		modifyTargetPosition(target_position);

 		//Move camera
 		kinectControl.moveKinect(target_position);
 		

 		std::cout << "Spinning... " << stepsTaken << std::endl;
 		stepsTaken++;
 		sleep(1);
 	}
 }