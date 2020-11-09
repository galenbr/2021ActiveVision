#include "active_vision/testingModel_v1.h"
#include "active_vision/dataHandling.h"

#define TIMEOUT 10
#define MIN_ANGLE 20
#define MIN_ANGLE_RAD MIN_ANGLE*(M_PI/180.0)

bool visualize = false;

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
	singlePass(kinectControl, targetPose, first, true);

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
bool exploreOneStepAllDir(environment &env, RouteData &home, std::vector<RouteData> &opOneStep){

	std::vector<double> curPose;
	RouteData temp;                // Used to record the iteration details

  // Sanity check
  if(home.childID == -1 || home.success == true){
    printf("ERROR exploreOneStepAllDir: Invalid child ID / Grasp already found.\n");
    return(false);
  }

	temp.parentID = home.childID;
  temp.success = false;
	// Going to each direction for a step and saving their configurations
	for (int i = 1; i <= 8 && temp.success == false; i++){
		// Rollback to the parent/home configuration
		env.rollbackConfiguration(temp.parentID);
		curPose = calcExplorationPose(env.lastKinectPoseViewsphere,i);

		// If kinect pose if above the table, then proceed in that direction
		if (checkValidPose(curPose) == true && checkIfNewPose(home.path,curPose) == true){
			// Saving the direction and path taken
			temp.direction = i;
			temp.path = home.path;
			temp.path.push_back(curPose);

			// Doing a pass and saving the child
			singlePass(env, curPose, false, true);
			temp.success = (env.selectedGrasp != -1);
			temp.childID = env.saveConfiguration(env.configurations[temp.parentID].description+"_C"+std::to_string(i));

			// Exit the loop and updating variables used for visualization
			if(temp.success == true){
				temp.graspQuality = env.graspsPossible[env.selectedGrasp].quality;
				env.updateGripper(env.selectedGrasp,0);
				temp.env = *env.ptrPtCldEnv + *env.ptrPtCldGripper;
			}else{
        temp.graspQuality = -1;
        temp.env = *env.ptrPtCldEnv;
      }

			opOneStep.push_back(temp);
		}
	}
	return(temp.success);
}

// Explore a max of "nSteps" random steps from the home/parent position
// Configurations not saved
bool exploreRdmStep(environment &env, RouteData &home, int nSteps, RouteData &opData){

	std::vector<double> curPose;

  // Sanity check
  if(home.childID == -1 || home.success == true){
    printf("ERROR exploreRdmStep: Invalid child ID / Grasp already found.\n");
    return(false);
  }

	opData.parentID = home.childID;
  opData.direction = home.direction;
  opData.path = home.path;

  // Rollback to the parent/home configuration
  env.rollbackConfiguration(opData.parentID);

  srand(time(NULL)); // Randoming the seed used for rand generation

  opData.success = false;
  // Going taking a max of nSteps from thoe parent position
	for (int i = 1; i <= nSteps && opData.success == false; i++){
    // Do until the pose is valid
    do{
      curPose = calcExplorationPose(env.lastKinectPoseViewsphere,rand()%((8-1)+1)+1);
    }while(checkValidPose(curPose) != true || checkIfNewPose(opData.path,curPose) != true);

	  // Saving path taken
		opData.path.push_back(curPose);

		// Doing a pass
		singlePass(env, curPose, false, true);
		opData.success = (env.selectedGrasp != -1);

		// Updating variables used for visualization
		if(opData.success == true){
			opData.graspQuality = env.graspsPossible[env.selectedGrasp].quality;
			env.updateGripper(env.selectedGrasp,0);
			opData.env = *env.ptrPtCldEnv + *env.ptrPtCldGripper;
		}
	}
  return(opData.success);
}

// Generating the home poses from which we need to search
void genHomePoses(environment &env, std::vector<double> &stPose, int step, std::vector<RouteData> &opHomePoses){
  if (step < 1 || step >3){
    printf("WARNING genHomePoses: Invalid step. Step set to 1.\n");
    step = 1;
  }
  std::vector<double> newPose;
  RouteData temp; temp.reset();
  if(step == 1){
    temp.path = {stPose};
    temp.type = 1;
    singlePass(env, stPose, true, true);
    temp.success = (env.selectedGrasp != -1);
    temp.obj = *env.ptrPtCldObject;
    temp.unexp = *env.ptrPtCldUnexp;
    temp.childID = env.saveConfiguration("Home_0");
    if(temp.success == true){
      temp.goodInitialGrasp = true;
      temp.graspQuality = env.graspsPossible[env.selectedGrasp].quality;
      env.updateGripper(env.selectedGrasp,0);
      temp.env = *env.ptrPtCldEnv + *env.ptrPtCldGripper;
    }
    else{
      temp.goodInitialGrasp = false;
      temp.env = *env.ptrPtCldEnv;
    }
    opHomePoses.push_back(temp);
  }else if(step == 2){
    temp.type = 2;
    singlePass(env, stPose, true, false);
    int homeID = env.saveConfiguration("Home_0");
    for (int i = 1; i <= 8; i++){
  		newPose = calcExplorationPose(stPose,i);
      if(checkValidPose(newPose) == true){
        temp.path = {stPose,newPose};
        singlePass(env, newPose, false, true);
        temp.success = (env.selectedGrasp != -1);\
        temp.obj = *env.ptrPtCldObject;
        temp.unexp = *env.ptrPtCldUnexp;
        temp.childID = env.saveConfiguration("Home_0_"+std::to_string(i));
        if(temp.success == true){
          temp.goodInitialGrasp = true;
          temp.graspQuality = env.graspsPossible[env.selectedGrasp].quality;
          env.updateGripper(env.selectedGrasp,0);
          temp.env = *env.ptrPtCldEnv + *env.ptrPtCldGripper;
        }
        else{
          temp.goodInitialGrasp = false;
          temp.env = *env.ptrPtCldEnv;
        }
        opHomePoses.push_back(temp);
      }
      env.rollbackConfiguration(homeID);
    }
  }else if(step == 3){

  }
}

// Function the generate the daa for training
void generateData(environment &kinectControl, int object, int homeType, std::string dir, std::string saveLocation){

	fstream fout;
 	fout.open(dir+saveLocation, ios::out | ios::app);

  kinectControl.spawnObject(object,0,0);

 	std::vector<double> startPose = {1.4, 180*(M_PI/180.0), 45*(M_PI/180.0)};

  bool graspFound = false;
	RouteData dataFinal, dataFinalTemp;
	std::vector<RouteData> dataOneStep,dataHomePoses;

  int maxRndSteps = 5;
  int nHomeConfigs = 0;

	for (int currentPoseCode = 0; currentPoseCode < kinectControl.objectPosesDict[object].size(); currentPoseCode+=1){
		for (int currentYaw = kinectControl.objectPosesYawLimits[object][0]; currentYaw < kinectControl.objectPosesYawLimits[object][1]; currentYaw+=MIN_ANGLE){
        printf("*** Obj #%d, Step #%d : Pose #%d/%d with Yaw %d ***\n",object+1,homeType,currentPoseCode+1,int(kinectControl.objectPosesDict[object].size()),currentYaw);
				kinectControl.moveObject(object,currentPoseCode,currentYaw*(M_PI/180.0));

        // Get the homes poses to collect data for based on the step count
        dataHomePoses.clear();
        genHomePoses(kinectControl,startPose,homeType,dataHomePoses);
        nHomeConfigs = kinectControl.configurations.size();

        for(int poses = 0; poses < dataHomePoses.size(); poses++){
          dataFinal.reset();

          printf("    Home #%d/%d... ",poses+1,int(dataHomePoses.size())); std::cout << std::flush;
          graspFound = dataHomePoses[poses].success;
          // If home pose has a successful grasp
          if(graspFound == true) dataFinal = dataHomePoses[poses];

          // If we do not find a grasp at home, then check for one step in each direction
          if(graspFound == false){
            printf("8 Dir Search... "); std::cout << std::flush;
            dataOneStep.clear();
            graspFound = exploreOneStepAllDir(kinectControl,dataHomePoses[poses],dataOneStep);
            if(graspFound == true) dataFinal = dataOneStep.back();
          }

          // for (int j = 0; j < kinectControl.configurations.size(); j++){
          //   printf("%d,%s\n",j,kinectControl.configurations[j].description.c_str());
          // }

          // If we dont find a grasp after one step, then check using random searches
          for (int rdmSearchCtr = 1; rdmSearchCtr <= 3 && graspFound == false; rdmSearchCtr++){
            printf("Random Search #%d... ",rdmSearchCtr); std::cout << std::flush;
            int nSteps = maxRndSteps*rdmSearchCtr;
            for(int i = 0; i < dataOneStep.size(); i++){
              dataFinalTemp.reset();
              bool graspFoundTemp = exploreRdmStep(kinectControl,dataOneStep[i],nSteps,dataFinalTemp);
              if(graspFoundTemp == true){
                graspFound = true;
                // If its the first time entering this, then store this in expRes
                if(dataFinal.path.size() == 0){
                  dataFinal = dataFinalTemp;
                }
                // If not, check if current result is better than the previous result and update expRes
                else if((dataFinalTemp.path.size() < dataFinal.path.size()) ||
                        (dataFinalTemp.path.size() == dataFinal.path.size() &&
                         dataFinalTemp.graspQuality > dataFinal.graspQuality)){
                  dataFinal = dataFinalTemp;
                }
                // Update nSteps to speed up the process
                nSteps = std::min(nSteps,int(dataFinal.path.size()-dataOneStep[i].path.size()));
                // If nSteps goes to 1, then break as the shortest ahs been found
                if(nSteps == 1) break;
              }
            }
          }

          // If still no grasp found, good luck
          if(graspFound == false){
            printf("Not OK.\n");
            dataFinal = dataHomePoses[poses]; // Storing dummy data
          }

          dataFinal.objType = kinectControl.objectDict[object][1];
          dataFinal.objPose = {kinectControl.objectPosesDict[object][currentPoseCode][1],
                               kinectControl.objectPosesDict[object][currentPoseCode][2],
                               currentYaw*(M_PI/180.0)};
          dataFinal.type = dataHomePoses[poses].type;
          dataFinal.nSteps = dataFinal.path.size()-dataFinal.type;
          dataFinal.obj = dataHomePoses[poses].obj;
          dataFinal.unexp = dataHomePoses[poses].unexp;
          dataFinal.filename = getCurTime();

          if(graspFound == true) printf("OK. %d Steps\n",dataFinal.nSteps);

          saveData(dataFinal, fout, dir);

          kinectControl.configurations.resize(nHomeConfigs);
        }

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
    generateData(kinectControl, i, 2, dir, time+csvName);
    kinectControl.reset();
  }
	std::cout << "End Time : " << getCurTime() << std::endl;
	// generateData(kinectControl, 4, dir, time+csvName);  kinectControl.reset();
	// std::cout << "End Time : " << getCurTime() << std::endl;
 }
