#include "active_vision/testingModel_v1.h"
#include "active_vision/dataHandling.h"

#define TIMEOUT 10
#define MIN_ANGLE 20
#define MIN_ANGLE_RAD MIN_ANGLE*(M_PI/180.0)

bool visualize = false;

struct explorer{
  int parentID{-1};
	int childID{-1};
	int direction{-1};
	std::vector<std::vector<double>> path;
	// Result collection (Used for visualization)
	double graspQuality{-1};
  ptCldColor ptCldRes;

  void reset(){
    parentID = -1;
    childID = -1;
    direction = -1;
    path.clear();
    graspQuality = -1;
    ptCldRes.clear();
  }
};

std::map<int, std::string> objLookup{{0, "Drill"}, {1, "Square Prism"}, {2, "Rectangular Prism"}, {3, "Bowl"}, {4, "Big Bowl"}, {5, "Cup"}};

void printVecofVec(std::vector<std::vector<double>> input){
	for(int i=0; i < input.size(); i++){
		double a, b, c;
		a = input[i][0];
		b = input[i][1];
		c = input[i][2];
		printf("(%1.2f, %1.2f, %1.2f)\n", a, b , c);
	}
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
bool checkValidPose(std::vector<double> &pose){
	return (round(pose[2]*180/M_PI) <= 90);
}

bool checkIfNewPose(std::vector<std::vector<double>> &oldPoses, std::vector<double> &pose){
  for(int i = 0; i < oldPoses.size(); i++){
    if(oldPoses[i] == pose) return false;
  }
  return true;
}

std::vector<double> calcExplorationPose(std::vector<double> startPose, int dir){
	double azimuthalOffset, polarOffset;
	switch(dir){
		case 1: azimuthalOffset = 0; 							polarOffset = -MIN_ANGLE_RAD; break;	//N
		case 2: azimuthalOffset = MIN_ANGLE_RAD; 	polarOffset = -MIN_ANGLE_RAD; break;	//NE
		case 3: azimuthalOffset = MIN_ANGLE_RAD; 	polarOffset = 0; 							break;	//E
		case 4: azimuthalOffset = MIN_ANGLE_RAD; 	polarOffset = MIN_ANGLE_RAD;  break;	//SE
		case 5: azimuthalOffset = 0; 							polarOffset = MIN_ANGLE_RAD;  break;	//S
		case 6: azimuthalOffset = -MIN_ANGLE_RAD; polarOffset = MIN_ANGLE_RAD;  break;	//SW
		case 7: azimuthalOffset = -MIN_ANGLE_RAD; polarOffset = 0; 							break;	//W
		case 8: azimuthalOffset = -MIN_ANGLE_RAD; polarOffset = -MIN_ANGLE_RAD; break;	//NW
		default: printf("ERROR in calcExplorationPose\n");
	}

	std::vector<double> potentialPose = {startPose[0], startPose[1]+azimuthalOffset, startPose[2]+polarOffset};

	// Addressing the NW & NE scenario when polar angle goes from *ve to -ve
	if(potentialPose[2] < 0 && startPose[2] > 0) potentialPose[1] = startPose[1]-azimuthalOffset;

	// Polar angle should be +ve
	if(potentialPose[2] < 0){
		potentialPose[2] = -1*potentialPose[2];
		potentialPose[1] = potentialPose[1] + M_PI;
	}

	// Azhimuthal angle 0 to 360 degree
	potentialPose[1] = fmod(potentialPose[1],2*M_PI);
	if(potentialPose[1] < 0) potentialPose[1] += 2*M_PI;

	// std::cout << dir << " " << startPose[1]*180/M_PI << "," << startPose[2]*180/M_PI << "->" <<
	//                            potentialPose[1]*180/M_PI << "," << potentialPose[2]*180/M_PI << std::endl;

	return(potentialPose);
}

/*
bool singleExploration(environment &kinectControl, std::vector<double> targetPose, ptCldVis::Ptr viewer, bool first){
	singlePass(kinectControl, targetPose, first);

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
	}
	return (kinectControl.selectedGrasp != -1);
}

std::vector<double> genRandomExploration(std::vector<double> currentPose, int &currentDir){
	std::vector<std::vector<double>> potentialPoses = gen8Explorations(currentPose);
	int selection = rand() % potentialPoses.size();
	while(selection == currentDir){
		selection = rand() % potentialPoses.size();
	}
	currentDir = selection;
	return potentialPoses.at(currentDir);
}

void addRandomPath(std::vector<std::vector<double>> &poses, std::vector<std::vector<std::vector<double>>> &histories, std::vector<int> &directions, int maxDepth){
	std::vector<double> currentPose = poses.back();
	std::vector<double> nextPose;
	int currentDir = directions.back();
	std::vector<std::vector<double>> oldPose = histories.back();
	for(int i=0; i<maxDepth; i++){
		nextPose = genRandomExploration(currentPose, currentDir);
		poses.insert(poses.begin(), nextPose);
		directions.insert(directions.begin(), currentDir);
		oldPose.push_back(nextPose);
		histories.insert(histories.begin(), oldPose);
		currentPose.swap(nextPose);
		nextPose.clear();
	}
}

std::vector<std::vector<double>> exploreRandomPoses(environment &kinectControl, std::vector<double> startPose, ptCldVis::Ptr viewer, int maxDepth, int &opDir){
	int root = kinectControl.saveConfiguration("Base");
	std::vector<std::vector<double>> poses = gen8Explorations(startPose);
	std::vector<int> parents, direction;
	std::vector<std::vector<std::vector<double>>> histories;
	for(int i=0; i<poses.size(); i++){
		parents.push_back(root);
		direction.push_back(i+1);
		histories.push_back({startPose, poses[i]});
	}
	bool finished = false;
	int currentSave = 0;
	for (int i=0; i<8; i++){
		finished = singleExploration(kinectControl, poses.back(), viewer, false);
		if (finished){
			opDir = direction.back();
			return histories.back();
		}
		currentSave = kinectControl.saveConfiguration(std::to_string(parents.back()));
		addRandomPath(poses, histories, direction, maxDepth);
		parents.insert(parents.begin(), currentSave);
		kinectControl.rollbackConfiguration(parents.back());
		poses.pop_back();
		parents.pop_back();
		direction.pop_back();
		histories.pop_back();
	}
	int bestDir = 0;
	int shortestLength = 999;
	std::vector<std::vector<double>> ret = {{-1}};
	for (int i=0; i<8; i++){
		finished = false;
		kinectControl.rollbackConfiguration(parents.back());
		parents.pop_back();
		for(int j=0; j<maxDepth; j++){
			if(!finished){
				finished = singleExploration(kinectControl, poses.back(), viewer, false);
				if (finished and histories.back().size() < shortestLength){
					bestDir = direction.back();
					ret = histories.back();
					shortestLength = histories.back().size();
				}
			}
			poses.pop_back();
			direction.pop_back();
			histories.pop_back();
		}
	}
	opDir = bestDir;
	return ret;
}

std::vector<std::vector<double>> exploreChildPoses(environment &kinectControl, std::vector<double> startPose, ptCldVis::Ptr viewer, int maxDepth, int &opDir){
	int root = kinectControl.saveConfiguration("Base");
	std::vector<std::vector<double>> poses = gen8Explorations(startPose);
	std::vector<int> parents, direction;
	std::vector<std::vector<std::vector<double>>> histories;
	for(int i=0; i<poses.size(); i++){
		parents.push_back(root);
		direction.push_back(i+1);
		histories.push_back({startPose, poses[i]});
	}
	bool finished = false;
	int currentSave = 0;
	while(!finished and parents.size() > 0 and maxDepth > 0){
		finished = singleExploration(kinectControl, poses.back(), viewer, false);
		if (finished){
			opDir = direction.back();
			return histories.back();
		}
		currentSave = kinectControl.saveConfiguration(std::to_string(parents.back()));
		std::vector<std::vector<double>> nextPoses = gen8Explorations(poses.back());
		for(int j=0; j < nextPoses.size(); j++){
			poses.insert(poses.begin(), nextPoses[j]);
			parents.insert(parents.begin(), currentSave);
			direction.insert(direction.begin(), direction.back());
			std::vector<std::vector<double>> oldPose = histories.back();
			oldPose.push_back(nextPoses[j]);
			histories.insert(histories.begin(), oldPose);
		}
		kinectControl.rollbackConfiguration(parents.back());
		poses.pop_back();
		parents.pop_back();
		direction.pop_back();
		histories.pop_back();
		maxDepth--;
	}
	return {{-1}};
}
*/

// Explore one step in all deirection from the home/parent position
// Configurations are saved
bool exploreOneStepAllDir(environment &env, explorer &home, std::vector<explorer> &opRecord){
	bool success = false;
	std::vector<double> curPose;
	explorer explorerTemp;					// Used to record the iteration details

	// Saving the home configuration
	explorerTemp.parentID = home.childID;
  if (explorerTemp.parentID == -1) {
    printf("ERROR exploreOneStepAllDir: Invalid parent ID.\n");
    return(success);
  }

	// Going to each direction for a step and saving their configurations
	for (int i = 1; i <= 8 && success == false; i++){
		// Rollback to the parent/home configuration
		env.rollbackConfiguration(explorerTemp.parentID);
		curPose = calcExplorationPose(env.lastKinectPoseViewsphere,i);
		// If kinect pose if above the table, then proceed in that direction
		if (checkValidPose(curPose) == true && checkIfNewPose(home.path,curPose) == true){
			// Saving the direction and path taken
			explorerTemp.direction = i;
			explorerTemp.path = home.path;
			explorerTemp.path.push_back(curPose);
      explorerTemp.graspQuality = -1;
      explorerTemp.ptCldRes.clear();

			// Doing a pass and saving the child
			singlePass(env, curPose, false);
			success = (env.selectedGrasp != -1);
			int child = env.saveConfiguration("P" + std::to_string(explorerTemp.parentID) + "D" + std::to_string(i));
			explorerTemp.childID = child;

			// Exit the loop and updating variables used for visualization
			if(success == true){
				explorerTemp.graspQuality = env.graspsPossible[env.selectedGrasp].quality;
				env.updateGripper(env.selectedGrasp,0);
				explorerTemp.ptCldRes = *env.ptrPtCldEnv + *env.ptrPtCldGripper;
			}

			opRecord.push_back(explorerTemp);
		}
	}
	return(success);
}

// Explore a max of "nSteps" random steps from the home/parent position
// Configurations not saved
bool exploreRdmStepOneDir(environment &env, explorer &home, int nSteps, explorer &opExp){
  bool success = false;
	std::vector<double> curPose;

  // Sanity check
  if (home.graspQuality != -1) printf("ERROR exploreOneStepAllDir: Starting with a successful grasp.\n");

	// Saving the home configuration
	opExp.parentID = home.childID;
  if (opExp.parentID == -1) {
    printf("ERROR exploreOneStepAllDir: Invalid parent ID.\n");
    return(success);
  }
  opExp.direction = home.direction;
  opExp.path = home.path;
  opExp.childID = -1;
  opExp.graspQuality = -1;
  opExp.ptCldRes.clear();

  // Rollback to the parent/home configuration
  env.rollbackConfiguration(opExp.parentID);

  srand (time(NULL)); // Randoming the seed used for rand generation

  // Going taking a max of nSteps from thoe parent position
	for (int i = 1; i <= nSteps && success == false; i++){
    // Do until the pose is valid
    do{
      int dir = rand() % ((8-1)+1) + 1;
      curPose = calcExplorationPose(env.lastKinectPoseViewsphere,dir);
    }while(checkValidPose(curPose) != true || checkIfNewPose(opExp.path,curPose) != true);

	  // Saving path taken
		opExp.path.push_back(curPose);

		// Doing a pass
		singlePass(env, curPose, false);
		success = (env.selectedGrasp != -1);

		// Updating variables used for visualization
		if(success == true){
			opExp.graspQuality = env.graspsPossible[env.selectedGrasp].quality;
			env.updateGripper(env.selectedGrasp,0);
			opExp.ptCldRes = *env.ptrPtCldEnv + *env.ptrPtCldGripper;
		}
	}
  return(success);
}

void generateData(environment &kinectControl, int object, std::string dir, std::string saveLocation){
	fstream fout;
 	fout.open(dir+saveLocation, ios::out | ios::app);

 	RouteData currentRoute;
 	currentRoute.objType= kinectControl.objectDict[object][1];

  kinectControl.spawnObject(object,0,0);

	ptCldVis::Ptr viewer;
	if(visualize) viewer = initRGBViewer();

 	std::vector<double> targetPose;
	bool success = false;
 	double azimuthalAngle = 180;
 	double polarAngle = 45;
	int configID;
	explorer expRes, expTemp1, expTemp2;
	std::vector<explorer> expRec;

  int maxRndSteps = 5;

	for (int currentPoseCode = 0; currentPoseCode < kinectControl.objectPosesDict[object].size(); currentPoseCode+=1){
		for (int currentYaw = kinectControl.objectPosesYawLimits[object][0]; currentYaw < kinectControl.objectPosesYawLimits[object][1]; currentYaw+=MIN_ANGLE){
        printf("*** Obj #%d : Pose #%d with Yaw %d ***\n",object+1,currentPoseCode+1,currentYaw);

        expRes.reset(); expTemp1.reset(); expTemp2.reset();
        expRec.clear();

				kinectControl.moveObject(object,currentPoseCode,currentYaw*(M_PI/180.0));

				//Move kinect to position
				targetPose = {1.4, azimuthalAngle*(M_PI/180.0), polarAngle*(M_PI/180.0)};
				// currentRoute.kinectPose = targetPose;
				currentRoute.objPose= {kinectControl.objectPosesDict[object][currentPoseCode][1],
															 kinectControl.objectPosesDict[object][currentPoseCode][2],
															 currentYaw*(M_PI/180.0)};

				expTemp1.direction = 0;
				expTemp1.path = {targetPose};

				// Do a pass for the starting position
				singlePass(kinectControl, targetPose, true);
				success = (kinectControl.selectedGrasp != -1);
				configID = kinectControl.saveConfiguration("Base");
				expTemp1.childID = configID;
				expTemp1.ptCldRes = *kinectControl.ptrPtCldEnv;

				*currentRoute.obj = *kinectControl.ptrPtCldObject;
				*currentRoute.unexp = *kinectControl.ptrPtCldUnexp;

        // If home position grasp is a success
        if(success == true){
          currentRoute.goodInitialGrasp = true;
          expTemp1.graspQuality = kinectControl.graspsPossible[kinectControl.selectedGrasp].quality;
    			kinectControl.updateGripper(kinectControl.selectedGrasp,0);
    			expTemp1.ptCldRes = *kinectControl.ptrPtCldEnv + *kinectControl.ptrPtCldGripper;
          expRes = expTemp1;
        }
				// If we do not find a grasp at home, then check for one step in each direction
				if(success == false){
					currentRoute.goodInitialGrasp = false;
					success = exploreOneStepAllDir(kinectControl,expTemp1,expRec);
					if(success == true) expRes = expRec.back();
				}
        // If we dont find a grasp after one step, then check using random searches
        int rdmSearchCtr = 1;
        while(success == false && rdmSearchCtr<=3){
          if (rdmSearchCtr > 1) printf("Doing %d set of random searches.\n",rdmSearchCtr);
          int nSteps = maxRndSteps*rdmSearchCtr;
          for(int i = 0; i < expRec.size(); i++) {
            bool successTemp;
            successTemp = exploreRdmStepOneDir(kinectControl,expRec[i],nSteps,expTemp2);
            if(successTemp == true){
              success = successTemp;
              // If its the first time entering this, then store this in expRes
              if(expRes.path.size() == 0){
                expRes = expTemp2;
              }
              // If not, check if current result is better than the previous result and update expRes
              else if((expTemp2.path.size() < expRes.path.size()) ||
                      (expTemp2.path.size() == expRes.path.size() && expTemp2.graspQuality > expRes.graspQuality)){
                expRes = expTemp2;
              }
              // Update nSteps to speed up the process
              nSteps = std::min(nSteps,int(expRes.path.size())-2);
              // If path length is 3 i.e 2 kinect movements then stop as its the best possible
              if(expRes.path.size() == 3) break;
            }
          }
          rdmSearchCtr++;
        }
        // If still no grasp found, probably need to try random search again
        if(success == false){
          printf("No grasp found still.\n");
          expRes = expTemp1;
        }

				currentRoute.direction = expRes.direction;
				currentRoute.stepVector = expRes.path;
				currentRoute.stepNumber = expRes.path.size();
				currentRoute.graspQuality = expRes.graspQuality;
				*currentRoute.env = expRes.ptCldRes;
				currentRoute.filename = getCurTime();
				saveData(currentRoute, fout, dir);

				kinectControl.reset();
			}
		}
  kinectControl.deleteObject(object);
	fout.close();
}

int main (int argc, char** argv){
	ros::init (argc, argv, "Supervisor_Node");
 	ros::NodeHandle nh;

 	environment kinectControl(&nh);
	sleep(1);
  kinectControl.loadGripper();

	std::string dir = "./DataRecAV/";
	std::string time = getCurTime();
	std::string csvName = "_dataRec.csv";
	std::cout << "Start Time : " << getCurTime() << std::endl;
  for (int i = 1; i < 2; i++) {
    generateData(kinectControl, i, dir, time+csvName);
    kinectControl.reset();
  }
	std::cout << "End Time : " << getCurTime() << std::endl;
	// generateData(kinectControl, 4, dir, time+csvName);  kinectControl.reset();
	// std::cout << "End Time : " << getCurTime() << std::endl;
 }
