#include <ros/ros.h>
#include <iostream>
#include "active_vision/testingModel.h"

#define TIMEOUT 10

int main (int argc, char** argv){
	ros::init (argc, argv, "Supervisor_Node");
 	ros::NodeHandle nh;

 	environment supervisor = environment(&nh);

 	int stepsTaken = 0;

 	while(stepsTaken < TIMEOUT){

 		//Get new data
 		testKinectRead(supervisor);

 		//Fuse with old data

 		//Grasp synthesis

 		//Grasp checking

 			//Exit if grasp good/timeout exceeded

 		//Build model

 		//Get instruction from optimization policy

 		//Move camera
 		testKinectMovement(supervisor);

 		std::cout << "Spinning... " << stepsTaken << std::endl;
 		stepsTaken++;
 		sleep(1);
 	}
 }