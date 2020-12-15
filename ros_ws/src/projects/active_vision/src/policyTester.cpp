#include "active_vision/environment.h"
#include "active_vision/toolDataHandling.h"
#include "active_vision/toolViewPointCalc.h"
#include "active_vision/toolVisualization.h"
#include <active_vision/toolStateVector.h>
#include "active_vision/heuristicPolicySRV.h"
#include "active_vision/trainedPolicySRV.h"

int mode;
int runMode;
int testAll;

void updateRouteData(environment &env, RouteData &data, bool save ,std::string name){
	data.success = (env.selectedGrasp != -1);
	if(save == true){
		data.obj = *env.ptrPtCldObject;
		data.unexp = *env.ptrPtCldUnexp;
		data.childID = env.saveConfiguration(name);
	}
	if(data.success == true){
		if(save == true) data.goodInitialGrasp = true;
		data.graspQuality = env.graspsPossible[env.selectedGrasp].quality;
		env.updateGripper(env.selectedGrasp,0);
		data.env = *env.ptrPtCldEnv + *env.ptrPtCldGripper;
	}
	else{
		if(save == true) data.goodInitialGrasp = false;
		data.env = *env.ptrPtCldEnv;
	}
}

std::vector<int> nearbyDirections(int dir){
	std::vector<int> dirs = {1,2,3,4,5,6,7,8};
	dir--;
	dirs[0] = (8+dir)%8+1;
	dirs[1] = (8+dir+1)%8+1; dirs[2] = (8+dir-1)%8+1;
	dirs[3] = (8+dir+2)%8+1; dirs[4] = (8+dir-1)%8+1;
	dirs[5] = (8+dir+3)%8+1; dirs[5] = (8+dir-1)%8+1;
	dirs[7] = (8+dir+4)%8+1;
	return dirs;
}
// Function the find the grasp
void findGrasp(environment &kinectControl, int object, int objPoseCode, int objYaw, std::string dir, std::string saveLocation, ptCldVis::Ptr viewer, ros::ServiceClient &policy){
	int maxSteps = 10;
	std::fstream fout;
 	fout.open(dir+saveLocation, std::ios::out | std::ios::app);

  kinectControl.spawnObject(object,0,0);

 	std::vector<double> startPose = {kinectControl.viewsphereRad, 180*(M_PI/180.0), 45*(M_PI/180.0)};
	std::vector<double> nextPose;
	RouteData home, current;

	printf("*** Obj #%d : Pose #%d with Yaw %d ***\n",object,objPoseCode,objYaw);

	kinectControl.moveObject(object,objPoseCode,objYaw*(M_PI/180.0));

	// Home Pose
	home.reset();
	home.path = {startPose};
	home.type = 1;
	home.direction = 0;
	home.nSteps = 0;
	home.objType = kinectControl.objectDict[object][1];
	home.objPose = {kinectControl.objectPosesDict[object][objPoseCode][1],
								  kinectControl.objectPosesDict[object][objPoseCode][2],
									objYaw*(M_PI/180.0)};
	printf("Starting first pass.\n");
	singlePass(kinectControl, startPose, true, true);
	updateRouteData(kinectControl,home,true,"Home");
	current = home;

	std::vector<int> vp;
	setupViewer(viewer, 2, vp);
	viewer->setPosition(550,20);
	keyboardEvent keyPress(viewer,2); keyPress.help();

	ptCldColor::Ptr tempPtCld{new ptCldColor};
	ptCldColor::Ptr tempPtCldObj{new ptCldColor};
	ptCldColor::Ptr tempPtCldUnexp{new ptCldColor};
	pcl::PointXYZ table,a1,a2;
	table.x = 1.5; table.y = 0; table.z = 1;
	std::vector<int> nUnexp;
	printf("Starting while loop.\n");

	while(keyPress.ok){
		viewer->resetStoppedFlag();
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		addRGB(viewer,kinectControl.ptrPtCldObject,"Obj",vp[0]);
		addRGB(viewer,kinectControl.ptrPtCldUnexp,"Unexp",vp[0]);
		*tempPtCld = current.env; addRGB(viewer,tempPtCld,"Env",vp[1]);

    addViewsphere(viewer,vp.back(),table,current.path[0][0],false);
		a1 = sphericalToCartesian(current.path[0],table);
		viewer->addSphere(a1,0.04,1,0,0,"Start",vp[1]);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,"Start");

    for(int i = 0; i < current.path.size()-1; i++){
			a1 = sphericalToCartesian(current.path[i],table);
			a2 = sphericalToCartesian(current.path[i+1],table);
			viewer->addArrow(a2,a1,0,1,0,false,std::to_string(i),vp[1]);
		}

		setCamView(viewer,current.path.back(),table);
		int maxIndex = 0;
		if(current.success == true){
			current.filename = getCurTime();
			saveData(current, fout, dir);
			viewer->addText("GRASP FOUND. Saving and exiting.",4,5,25,1,0,0,"Dir1",vp[0]);
		}else{
      if(::runMode == 1){
        static active_vision::heuristicPolicySRV srv;
        pcl::toROSMsg(*kinectControl.ptrPtCldObject,srv.request.object);
        pcl::toROSMsg(*kinectControl.ptrPtCldUnexp,srv.request.unexplored);
        srv.request.path.data.clear();
        for(int i = 0; i < current.path.size(); i++){
          srv.request.path.data.push_back(current.path[i][0]);
          srv.request.path.data.push_back(current.path[i][1]);
          srv.request.path.data.push_back(current.path[i][2]);
        }
        srv.request.mode = ::mode;
        policy.call(srv);
  			maxIndex = srv.response.direction;
  			viewer->addText("Best direction calculated : " + std::to_string(maxIndex+1) + "(" + dirLookup[maxIndex+1] + ").\nPress any key to continue.",5,30,25,1,0,0,"Dir1",vp[0]);
      }else if(::runMode == 2){
				static HAFStVec1 stVec;
				stVec.setGridDim(5);
		    stVec.setMaintainScale(true);
				*tempPtCldObj = *kinectControl.ptrPtCldObject;
				*tempPtCldUnexp = *kinectControl.ptrPtCldUnexp;
				stVec.setInput(tempPtCldObj,tempPtCldUnexp,current.path.back());
				stVec.calculate();
				static active_vision::trainedPolicySRV srv;
				srv.request.stateVec.data = stVec.getStateVec();
				policy.call(srv);
				maxIndex = srv.response.direction-1;
  			viewer->addText("Best direction calculated : " + std::to_string(maxIndex+1) + "(" + dirLookup[maxIndex+1] + ").\nPress any key to continue.",5,30,25,1,0,0,"Dir1",vp[0]);
			}else{
        viewer->addText("Manual Mode.\nEnter the direction.",5,30,25,1,0,0,"Dir1",vp[0]);
      }
		}

		if(::runMode == 0){
			keyPress.called = false;
			while(!viewer->wasStopped() && keyPress.called==false){
				viewer->spinOnce(100);
				boost::this_thread::sleep (boost::posix_time::microseconds(100000));
			}
		}
    else{
      keyPress.called = false;
			if(::testAll == 1){
				viewer->spinOnce(100);
				boost::this_thread::sleep (boost::posix_time::microseconds(100000));
			}else{
				while(!viewer->wasStopped() && keyPress.called==false){
					viewer->spinOnce(100);
					boost::this_thread::sleep (boost::posix_time::microseconds(100000));
				}
			}
  		keyPress.dir = maxIndex+1;
		}

		if(current.success == true){
			keyPress.dir = 0;
			keyPress.ok = false;
		}else{
			// Limiting the steps
			if(current.nSteps > maxSteps){
				current.filename = getCurTime();
				saveData(current, fout, dir);
				keyPress.dir = 0;
				keyPress.ok = false;
			}
		}

		// Reset to home if direction is 0
		if(keyPress.dir == 0){
			kinectControl.rollbackConfiguration(home.childID);
			current = home;
			current.nSteps = 0;
		}

		if(keyPress.dir >=1 && keyPress.dir <= 8){
			std::vector<int> dirPref = nearbyDirections(keyPress.dir);
			int prfID = 0;
      nextPose = calcExplorationPose(current.path.back(),dirPref[prfID],::mode);
			while(checkValidPose(nextPose) == false || checkIfNewPose(current.path,nextPose,::mode) == false){
				prfID++;
				nextPose = calcExplorationPose(current.path.back(),dirPref[prfID],::mode);
			}
			if(prfID!=0){
				printf("Direction modified from %d -> %d.\n", dirPref[0],dirPref[prfID]);
			}
			singlePass(kinectControl, nextPose, false, true);
			updateRouteData(kinectControl,current,false,"dummy");
			current.path.push_back(nextPose);
			current.nSteps++;
			if(current.nSteps == 1) home.direction = keyPress.dir;
    }
	}

	keyPress.ok = true;
	kinectControl.deleteObject(object);
	kinectControl.reset();
	fout.close();
}

void help(){
  std::cout << "******* Policy Tester Node Help *******" << std::endl;
  std::cout << "Arguments : [CSV filename] [Object] [MoveMode] [RunMode]" << std::endl;
  std::cout << "CSV filename : CSV file name (Eg:data.csv) (Use \"default.csv\" to use time as the file name)" << std::endl;
	std::cout << "Object : Object ID -> 0(Drill),1(Sq Prism),2(Rect Prism)" << std::endl;
	std::cout << "MoveMode : 1->Normal, 2->New" << std::endl;
	std::cout << "RunMode : 0->Manual, 1->Heuristic, 2->Trained Policy" << std::endl;
  std::cout << "*******" << std::endl;
}

int main(int argc, char** argv){
	if(argc != 5 && argc != 7){
    ROS_ERROR("Incorrect number of arguments.");
    help(); return(-1);
  }
	chdir(getenv("HOME"));

	ros::init(argc, argv, "Policy_Tester");
 	ros::NodeHandle nh;
  ros::ServiceClient policy;
 	environment kinectControl(&nh); sleep(1);
	kinectControl.setPtCldNoise(0.5);
	kinectControl.viewsphereRad = 1.0;
  kinectControl.loadGripper();

	std::string dir;
	nh.getParam("/active_vision/data_dir", dir);
	std::string time = getCurTime();
	std::string tempName(argv[1]);
	std::string csvName;
	if(tempName == "default.csv") csvName = time+"_dataRec.csv";
	else	csvName = argv[1];

	if(csvName.substr(csvName.size() - 4) != ".csv"){
    ROS_ERROR("Incorrect file name.");
    help(); return(-1);
	}

	int objID = std::atoi(argv[2]);
	if(objID < 0 && objID > 7) objID = 0;
	::mode = std::atoi(argv[3]);
	if(::mode < 1 && ::mode > 2) ::mode = 1;
  ::runMode = std::atoi(argv[4]);
	if(::runMode < 0 && ::runMode > 2) ::runMode = 0;

  if(::runMode == 0){
    ROS_INFO("Manual Mode Selected.");
  }
  else if(::runMode == 1){
    policy = nh.serviceClient<active_vision::heuristicPolicySRV>("/active_vision/heuristic_policy");
    ROS_INFO("Heuristic Policy Selected.");
  }else if(::runMode == 2){
		policy = nh.serviceClient<active_vision::trainedPolicySRV>("/active_vision/trained_policy");
    ROS_INFO("Trained Policy Selected.");
	}

	ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));

	if(::runMode != 0){
		printf("Do you want to test for all possible poses (0->No, 1->Yes) : ");
		std::cin >> ::testAll;
		if(::testAll < 0 && ::testAll > 1) ::testAll = 0;
	}else{
		::testAll = 0;
	}

	if(testAll == 0){
		int objPoseCode, objYaw;
		printf("Enter the object pose code (0-%d) : ", int(kinectControl.objectPosesDict[objID].size()-1));
		std::cin >> objPoseCode;
		if(objPoseCode < 0 || objPoseCode > kinectControl.objectPosesDict[objID].size()-1) objPoseCode = 0;
		printf("Enter the object yaw (deg) (0-360) : ");
		std::cin >> objYaw;

		findGrasp(kinectControl, objID, objPoseCode, objYaw, dir, csvName, viewer, policy);
	}else{
		for(int objPoseCode = 0; objPoseCode < kinectControl.objectPosesDict[objID].size(); objPoseCode+=1){
			for(int objYaw = kinectControl.objectPosesYawLimits[objID][0]; objYaw < kinectControl.objectPosesYawLimits[objID][1]; objYaw+=10){
				findGrasp(kinectControl, objID, objPoseCode, objYaw, dir, csvName, viewer, policy);
			}
		}
	}

	printf("Data saved to : %s\n",csvName.c_str());
 }
