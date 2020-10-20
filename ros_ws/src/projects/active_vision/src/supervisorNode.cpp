#include "active_vision/testingModel.h"
#include "active_vision/dataHandling.h"

#define TIMEOUT 10
#define MIN_ANGLE 20

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

int main (int argc, char** argv){
	ros::init (argc, argv, "Supervisor_Node");
 	ros::NodeHandle nh;

 	environment kinectControl(&nh);
	sleep(1);

	ptCldColor::Ptr curTotal(new ptCldColor);
	ptCldColor::ConstPtr cCurTotal(curTotal);
 	ptCldColor::Ptr curObj(new ptCldColor);
 	Eigen::Affine3f tfGazWorld;
 	kinectControl.spawnObject(0,0,0,0);

 	ptCldVis::Ptr viewer = initRGBViewer();

 	std::vector<double> targetPose;

	for (int polarAngle = 0; polarAngle < 360; polarAngle+=MIN_ANGLE){
		for (int azimuthalAngle = 0; azimuthalAngle < 90; azimuthalAngle+=MIN_ANGLE){
				//Move kinect to position
				targetPose = {1.4, polarAngle*(M_PI/180.0), azimuthalAngle*(M_PI/180.0)};
				kinectControl.moveKinectViewsphere(targetPose);
				kinectControl.readKinect();
    			kinectControl.fuseLastData(curTotal);
    			kinectControl.dataExtract(cCurTotal, curObj);
    			kinectControl.graspSynthesis(curObj);
    			std::cout << "Min grasp quality threshold is " << kinectControl.minGraspQuality << " Grasps found: " << kinectControl.graspsPossible.size() << std::endl;
    			rbgVis(viewer,curObj,"Raw Data",0);
    			if (kinectControl.graspsPossible.size() > 0){
      				for (int i = 0; i < std::min(3, (int)kinectControl.graspsPossible.size()); i++){
      					viewer->removeShape("GP_"+std::to_string(i)+"_A",0);
      					viewer->removeShape("GP_"+std::to_string(i)+"_B",0);
        				viewer->addSphere<pcl::PointXYZRGB>(kinectControl.graspsPossible[i].p1,0.0050,0.0,0.0,(i+1.0)/3.0,"GP_"+std::to_string(i)+"_A",0);
        				viewer->addSphere<pcl::PointXYZRGB>(kinectControl.graspsPossible[i].p2,0.0050,0.0,0.0,(i+1.0)/3.0,"GP_"+std::to_string(i)+"_B",0);
      				}
    			}
    			viewer->spinOnce(100);
				sleep(1);
				curTotal->clear();
				curObj->clear();
			//If bad, algorithm.
		}
	}
	while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
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
