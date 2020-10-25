#include "active_vision/testingModel_v1.h"
#include "active_vision/dataHandling.h"

#define TIMEOUT 10
#define MIN_ANGLE 20
#define MIN_ANGLE_RAD MIN_ANGLE*(M_PI/180.0)

bool visualize = false;

void modifyTargetPosition(std::vector<double> &pos){
	pos[0]+=0.1;
	pos[1]+=0.1;
}

ptCldVis::Ptr initRGBViewer(){
	ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
  	viewer->initCameraParameters();
    int vp(0);
    viewer->createViewPort(0.0,0.0,1.0,1.0,vp);
    viewer->addCoordinateSystem(1.0);
    viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);
    return viewer;
}

//Check that the azimuthal angle is less than 90.
bool checkValidPose(std::vector<double> pose){
	return (pose[2] < (M_PI/2+.1));
}

void addValidPose(std::vector<std::vector<double>> *out, std::vector<double> startPose, double polarOffset, double azimuthalOffset){
	std::vector<double> potentialPose = {startPose[0], startPose[1]+polarOffset, startPose[2]+azimuthalOffset};
	if(checkValidPose(potentialPose)){
		(*out).insert((*out).end(), potentialPose);
	}
}

std::vector<std::vector<double>> gen8Explorations(std::vector<double> startPose){
	std::vector<std::vector<double>> out;
	addValidPose(&out, startPose, 0, -MIN_ANGLE_RAD); //N
	addValidPose(&out, startPose, MIN_ANGLE_RAD, -MIN_ANGLE_RAD); //NE
	addValidPose(&out, startPose, MIN_ANGLE_RAD, 0); //E
	addValidPose(&out, startPose, MIN_ANGLE_RAD, MIN_ANGLE_RAD); //SE
	addValidPose(&out, startPose, 0, MIN_ANGLE_RAD); //S
	addValidPose(&out, startPose, -MIN_ANGLE_RAD, MIN_ANGLE_RAD); //SW
	addValidPose(&out, startPose, -MIN_ANGLE_RAD, 0); //W
	addValidPose(&out, startPose, -MIN_ANGLE_RAD, -MIN_ANGLE_RAD); //NW
	return out;
}

void printVecofVec(std::vector<std::vector<double>> input){
	for(int i=0; i < input.size(); i++){
		double a, b, c;
		a = input[i][0];
		b = input[i][1];
		c = input[i][2];
		printf("(%1.2f, %1.2f, %1.2f)\n", a, b , c);
	}
}

int main (int argc, char** argv){
	ros::init (argc, argv, "Supervisor_Node");
 	ros::NodeHandle nh;

 	environment kinectControl(&nh);
	sleep(1);

 	kinectControl.spawnObject(0,0,0,0);
	kinectControl.loadGripper();

 	ptCldVis::Ptr viewer = initRGBViewer();

 	std::vector<double> targetPose;

	for (int polarAngle = 0; polarAngle < 360; polarAngle+=MIN_ANGLE){
		for (int azimuthalAngle = 10; azimuthalAngle < 90; azimuthalAngle+=MIN_ANGLE){
				std::cout << "Polar Angle : " << polarAngle << " Azhimuthal Angle : " << azimuthalAngle << std::endl;
				//Move kinect to position
				targetPose = {1.4, polarAngle*(M_PI/180.0), azimuthalAngle*(M_PI/180.0)};

				kinectControl.reset();
				singlePass(kinectControl, targetPose, true);	// Initial Pass

				// if(0 == polarAngle and 10 == azimuthalAngle){
				// 	kinectControl.reset();
				// 	singlePass(kinectControl, targetPose, true); // Initial Pass
				// } else {
				// 	singlePass(kinectControl, targetPose, false);
				// }

				if(kinectControl.selectedGrasp != -1){
					std::cout << "Selected Grasp ID : " << kinectControl.selectedGrasp << std::endl;
					std::cout << "Selected Grasp Quality : " << kinectControl.graspsPossible[kinectControl.selectedGrasp].quality << std::endl;
				}else{
					std::cout << "No grasp found." << std::endl;
				}

				// Visualization Section Start
				if (visualize == true){
					for (int i = 0; i < kinectControl.ptrPtCldObject->size(); i++) {
						kinectControl.ptrPtCldObject->points[i].r = 0;
						kinectControl.ptrPtCldObject->points[i].b = 200;
						kinectControl.ptrPtCldObject->points[i].g = 0;
					}
					rbgVis(viewer,kinectControl.ptrPtCldObject,"Object",0);

					for (int i = 0; i < kinectControl.ptrPtCldUnexp->size(); i++) {
						kinectControl.ptrPtCldUnexp->points[i].r = 200;
						kinectControl.ptrPtCldUnexp->points[i].b = 0;
						kinectControl.ptrPtCldUnexp->points[i].g = 0;
					}
					rbgVis(viewer,kinectControl.ptrPtCldUnexp,"Unexplored",0);

					if(kinectControl.selectedGrasp != -1){
						kinectControl.updateGripper(kinectControl.selectedGrasp,0);    // Only for visulization purpose
						rbgVis(viewer,kinectControl.ptrPtCldGripper,"Gripper",0);
					}
					viewer->spinOnce(100);
					viewer->removeAllPointClouds(0);
					sleep(2);
				}
				// Visualization Section End
		}
	}

  kinectControl.deleteObject(0);
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
