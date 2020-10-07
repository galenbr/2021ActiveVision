#include "active_vision/testingModel.h"

#define TIMEOUT 10

void modifyTargetPosition(std::vector<float> &pos){
	pos[0]+=0.1;
	pos[1]+=0.1;
}

int main (int argc, char** argv){
	ros::init (argc, argv, "Supervisor_Node");
 	ros::NodeHandle nh;

 	environment kinectControl(&nh);
	sleep(1);

 	int stepsTaken = 0;
	std::vector<float> home_position = {0.25, 0.0, 1.75, 0.0, 0.55, 0.0};
 	std::vector<float> target_position = {0.25, 0.0, 1.75, 0.0, 0.55, 0.0};

	kinectControl.moveKinect(home_position);

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
 		kinectControl.moveKinect(target_position);

 		std::cout << "Spinning... " << stepsTaken << std::endl;
 		stepsTaken++;
 		sleep(1);
 	}
 }