#include "active_vision/testingModel.h"
#include "active_vision/dataHandling.h"

#define TIMEOUT 10
#define MIN_ANGLE 20

void modifyTargetPosition(std::vector<double> &pos){
	pos[0]+=0.1;
	pos[1]+=0.1;
}

int main (int argc, char** argv){
	ros::init (argc, argv, "Supervisor_Node");
 	ros::NodeHandle nh;

 	environment kinectControl(&nh);
	sleep(1);

 	//int stepsTaken = 0;
	//std::vector<double> home_position = {0.25, 0.0, 1.75, 0.0, 0.55, 0.0};
 	//std::vector<double> target_position = {0.25, 0.0, 1.75, 0.0, 0.55, 0.0};

 	test();

	//kinectControl.moveKinectCartesian(home_position);
	for (int polarAngle = 0; polarAngle < 360; polarAngle+=MIN_ANGLE){
		for (int azimuthalAngle = 0; azimuthalAngle < 90; azimuthalAngle+=MIN_ANGLE){
				//kinectControl.moveKinectViewsphere()
				std::cout << "Angles: " << polarAngle << azimuthalAngle << std::endl;
			//Move kinect to position
			//Check grasp
			//If bad, algorithm.
		}
	}
	/*

 	while(stepsTaken < TIMEOUT){

 		//Get new data
 		// kinectControl.readKinect();

 		//Fuse with old data
		// kinectControl.fuseLastData();

		//Extract the object and table
		// kinectControl.dataExtract();

		//Generate the unexplored point cloud (Only the first time)
		// if (stepsTaken == 0) {
		// 	kinectControl.genUnexploredPtCld();
		// }

		//Update the unexplored point cloud
		// kinectControl.updateUnexploredPtCld();

 		//Grasp synthesis

 		//Grasp checking

 			//Exit if grasp good/timeout exceeded

 		//Build model

 		//Get instruction from optimization policy
 		modifyTargetPosition(target_position);

 		//Move camera
 		kinectControl.moveKinectCartesian(target_position);

 		std::cout << "Spinning... " << stepsTaken << std::endl;
 		stepsTaken++;
 		sleep(1);
 	}*/
 }
