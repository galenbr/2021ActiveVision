#include "active_vision/testingModel_v1.h"
#include "active_vision/dataHandling.h"

#define TIMEOUT 10
#define MIN_ANGLE 20
#define MIN_ANGLE_RAD MIN_ANGLE*(M_PI/180.0)

bool saveOnlyUsefulData = false;
bool visualize = false;
int mode = 1;

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

pcl::PointXYZ sphericalToCartesian(std::vector<double> &pose){
  pcl::PointXYZ res;
  res.x = pose[0]*sin(pose[2])*cos(pose[1]);
  res.y = pose[0]*sin(pose[2])*sin(pose[1]);
  res.z = pose[0]*cos(pose[2]);
  return(res);
}

std::vector<double> cartesianToSpherical(pcl::PointXYZ &point){
  std::vector<double> res = {0,0,0};
  res[0] = sqrt(pow(point.x,2)+pow(point.y,2)+pow(point.z,2));
  res[1] = atan2(point.y,point.x);
  res[2] = atan2(sqrt(pow(point.x,2)+pow(point.y,2)),point.z);
  return(res);
}

double disBtwSpherical(std::vector<double> &poseA,std::vector<double> &poseB){
	pcl::PointXYZ poseAcart = sphericalToCartesian(poseA);
	pcl::PointXYZ poseBcart = sphericalToCartesian(poseB);
	return(sqrt(pow(poseAcart.x-poseBcart.x,2)+pow(poseAcart.y-poseBcart.y,2)+pow(poseAcart.z-poseBcart.z,2)));
}

//Check that the azimuthal angle is less than 90.
bool checkValidPose(std::vector<double> &pose){
	return (round(pose[2]*180/M_PI) <= 90);
}

bool checkIfNewPose(std::vector<std::vector<double>> &oldPoses, std::vector<double> &pose){
  for(int i = 0; i < oldPoses.size(); i++){
		if(::mode == 1){
			if(oldPoses[i] == pose) return false;
		}else{
			if(disBtwSpherical(oldPoses[i],pose) <= 0.75*(pose[0])*(20*M_PI/180)) return false;
		}

  }
  return true;
}

std::vector<double> calcExplorationPoseB(std::vector<double> &startPose, int dir){

  Eigen::Vector3f xAxis,yAxis,zAxis,rotAxis,tempVec;
  Eigen::Vector3f xyPlane(0,0,1);
  Eigen::Matrix3f matA; matA << 1,0,0,0,1,0,0,0,1;
  Eigen::Matrix3f matB, matC, tempMat;
  // tf::Matrix3x3 rotMat;

  pcl::PointXYZ stPoint = sphericalToCartesian(startPose);
  pcl::PointXYZ endPoint;

  zAxis = stPoint.getVector3fMap(); zAxis.normalize();
  xAxis = zAxis.cross(xyPlane); xAxis.normalize();
  yAxis = zAxis.cross(xAxis);

	std::vector<double> ratio={0,0};
	switch(dir){
		case 1: ratio[0]=+1; ratio[1]=0;   break;
		case 2: ratio[0]=+1; ratio[1]=-1;  break;
		case 3: ratio[0]=0;  ratio[1]=-1;  break;
		case 4: ratio[0]=-1; ratio[1]=-1;  break;
		case 5: ratio[0]=-1; ratio[1]=0;   break;
		case 6: ratio[0]=-1; ratio[1]=+1;  break;
		case 7: ratio[0]=0;  ratio[1]=+1;  break;
		case 8: ratio[0]=+1; ratio[1]=+1;  break;
	}

  rotAxis = ratio[0]*xAxis + ratio[1]*yAxis; rotAxis.normalize();
  matB << 0,-rotAxis[2],rotAxis[1],rotAxis[2],0,-rotAxis[0],-rotAxis[1],rotAxis[0],0;
  matC = rotAxis*rotAxis.transpose();

  tempMat = cos(20*M_PI/180)*matA + sin(20*M_PI/180)*matB + (1-cos(20*M_PI/180))*matC;
  tempVec = tempMat*stPoint.getVector3fMap();
  endPoint.x = tempVec[0]; endPoint.y = tempVec[1]; endPoint.z = tempVec[2];

  std::vector<double> end = cartesianToSpherical(endPoint);

  // Polar angle 0 to 90 degree
  if(end[2] < 0){
    end[2] = -1*end[2];
    end[1] = end[1] + M_PI;
  }

  // Azhimuthal angle 0 to 360 degree
  end[1] = fmod(end[1],2*M_PI);
  if(end[1] < 0) end[1] += 2*M_PI;

	if(checkValidPose(end)) return end;
  else                    return startPose;
}

std::vector<double> calcExplorationPoseA(std::vector<double> &startPose, int dir){
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

std::vector<double> calcExplorationPose(std::vector<double> &startPose, int dir){
	if(::mode == 1) return calcExplorationPoseA(startPose,dir);
	else return calcExplorationPoseB(startPose,dir);
}
/*
bool singleExploration(environment &kinectControl, std::vector<double> targetPose, ptCldVis::Ptr viewer, bool first){
	singlePass(kinectControl, targetPose, first, true);

	// Visualization Section Start
	if (::visualize == true){
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
		std::cout << "." << std::flush;
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

void updateTemp(environment &env, RouteData &data, std::string &name){
	data.success = (env.selectedGrasp != -1);
	data.obj = *env.ptrPtCldObject;
	data.unexp = *env.ptrPtCldUnexp;
	data.childID = env.saveConfiguration(name);
	if(data.success == true){
		data.goodInitialGrasp = true;
		data.graspQuality = env.graspsPossible[env.selectedGrasp].quality;
		env.updateGripper(env.selectedGrasp,0);
		data.env = *env.ptrPtCldEnv + *env.ptrPtCldGripper;
	}
	else{
		data.goodInitialGrasp = false;
		data.env = *env.ptrPtCldEnv;
	}
}

// Generating the home poses from which we need to search
void genHomePoses(environment &env, std::vector<double> &stPose, int step, std::vector<RouteData> &opHomePoses){
  if (step < 1 || step >3){
    printf("WARNING genHomePoses: Invalid step. Step set to 1.\n");
    step = 1;
  }
  std::vector<double> newPose1,newPose2;
	std::string name;
  RouteData temp; temp.reset();
  if(step == 1){
    temp.path = {stPose};
    temp.type = 1;
    singlePass(env, stPose, true, true);
		name = "H0";
		updateTemp(env,temp,name);
    opHomePoses.push_back(temp);
		std::cout << "." << std::flush;
  }else if(step == 2){
    temp.type = 2;
    singlePass(env, stPose, true, false);
    int homeID = env.saveConfiguration("H0");
    for (int i = 1; i <= 8; i++){
  		newPose1 = calcExplorationPose(stPose,i);
      if(checkValidPose(newPose1) == true){
				name = "H0_"+std::to_string(i);
        temp.path = {stPose,newPose1};
        singlePass(env, newPose1, false, true);
				updateTemp(env,temp,name);
        opHomePoses.push_back(temp);
				std::cout << "." << std::flush;
      }
      env.rollbackConfiguration(homeID);
    }
  }else if(step == 3){
		temp.type = 3;
    singlePass(env, stPose, true, false);
		int homeID = env.saveConfiguration("H0");
		for(int i = 1; i <= 8; i++){
			newPose1 = calcExplorationPose(stPose,i);
			if(checkValidPose(newPose1) == true){
				for(int j = 1; j <= 8; j++){
					temp.path = {stPose,newPose1};
					newPose2 = calcExplorationPose(newPose1,j);
					if(checkValidPose(newPose2) == true && checkIfNewPose(temp.path,newPose2) == true){
						name = "H0_"+std::to_string(i)+"_"+std::to_string(j);
						temp.path = {stPose,newPose1,newPose2};
						singlePass(env, newPose1, false, false);
						singlePass(env, newPose2, false, true);
						updateTemp(env,temp,name);
						opHomePoses.push_back(temp);
						std::cout << "." << std::flush;
					}
					env.rollbackConfiguration(homeID);
				}
			}
		}
  }
}

// Function the generate the daa for training
std::vector<int> generateData(environment &kinectControl, int object, int homeType, std::string dir, std::string saveLocation){
	std::vector<int> nSaved = {0,0};
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
		for (int currentYaw = kinectControl.objectPosesYawLimits[object][0]; currentYaw < kinectControl.objectPosesYawLimits[object][1]; currentYaw+=40){
        printf("  *** Pose #%d/%d with Yaw %d ***\n",currentPoseCode+1,int(kinectControl.objectPosesDict[object].size()),currentYaw);
				kinectControl.moveObject(object,currentPoseCode,currentYaw*(M_PI/180.0));

        // Get the homes poses to collect data for based on the step count
        dataHomePoses.clear();
				printf("    * Generating Home Configs"); std::cout << std::flush;
        genHomePoses(kinectControl,startPose,homeType,dataHomePoses);
        nHomeConfigs = kinectControl.configurations.size();
				std::cout << " " << dataHomePoses.size() << " configs." << std::endl;

				// for (int j = 0; j < kinectControl.configurations.size(); j++){
				//   printf("%d,%s\n",j,kinectControl.configurations[j].description.c_str());
				// }

        for(int poses = 0; poses < dataHomePoses.size(); poses++){
          dataFinal.reset();

          printf("      Home #%d/%d : ",poses+1,int(dataHomePoses.size())); std::cout << std::flush;
          graspFound = dataHomePoses[poses].success;
          // If home pose has a successful grasp
          if(graspFound == true) dataFinal = dataHomePoses[poses];

          // If we do not find a grasp at home, then check for one step in each direction
          if(graspFound == false){
            printf("8 Dir Search"); std::cout << std::flush;
            dataOneStep.clear();
            graspFound = exploreOneStepAllDir(kinectControl,dataHomePoses[poses],dataOneStep);
            if(graspFound == true) dataFinal = dataOneStep.back();
          }

          // If we dont find a grasp after one step, then check using random searches
          for (int rdmSearchCtr = 1; rdmSearchCtr <= 3 && graspFound == false; rdmSearchCtr++){
            printf(" Random Search #%d",rdmSearchCtr); std::cout << std::flush;
            int nSteps = maxRndSteps*rdmSearchCtr;
            for(int i = 0; i < dataOneStep.size(); i++){
							std::cout << "." << std::flush;
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
            printf(" Not OK.\n");
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

          if(graspFound == true) printf(" OK. %d Steps\n",dataFinal.nSteps);

					if(dataFinal.nSteps > 0){
						saveData(dataFinal, fout, dir);
						nSaved[1]++;
					}else{
						nSaved[0]++;
						if(::saveOnlyUsefulData == false) saveData(dataFinal, fout, dir);
					}

          kinectControl.configurations.resize(nHomeConfigs);
        }

        kinectControl.reset();
			}
		}
  kinectControl.deleteObject(object);
	fout.close();
	return(nSaved);
}

void help(){
  std::cout << "******* Supervisor Node Help *******" << std::endl;
  std::cout << "Arguments : [CSV filename] [Object] [Step No] [Mode]" << std::endl;
  std::cout << "CSV filename : CSV file name (Eg:data.csv) (Use \"default.csv\" to use time as the file name)" << std::endl;
	std::cout << "Object : Object ID -> 0(Drill),1(Sq Prism),2(Rect Prism)" << std::endl;
	std::cout << "Step No : 1->Data for first step, 2->Second step, 3->Third Step" << std::endl;
	std::cout << "Mode : 1->Normal, 2->New" << std::endl;
  std::cout << "*******" << std::endl;
}

int main (int argc, char** argv){
	ros::init(argc, argv, "Supervisor_Node");

	if(argc != 5){
    std::cout << "ERROR. Incorrect number of arguments." << std::endl;
    help(); return(-1);
  }

 	ros::NodeHandle nh;
	std::chrono::high_resolution_clock::time_point start,end;

 	environment kinectControl(&nh);
	sleep(1);
  	kinectControl.loadGripper();

	std::string dir = "./DataRecAV/";
	std::string time = getCurTime();
	std::string tempName(argv[1]);
	std::string csvName;
	if(tempName == "default.csv") csvName = time+"_dataRec.csv";
	else	csvName = argv[1];

	if(csvName.substr(csvName.size() - 4) != ".csv"){
		std::cout << "ERROR. Incorrect file name." << std::endl;
    help(); return(-1);
	}

	int objID = std::atoi(argv[2]);
	if(objID < 0 && objID > 2) objID = 0;
	int stepType = std::atoi(argv[3]);
	if(stepType < 1 && stepType > 3) stepType = 1;
	::mode = std::atoi(argv[4]);
	if(::mode < 1 && ::mode > 2) ::mode = 1;

	std::cout << "Start Time : " << getCurTime() << std::endl;
	std::vector<int> ctrData;
	start = std::chrono::high_resolution_clock::now();

	printf("***** Object #%d, Data Type #%d *****\n",objID,stepType);
	ctrData = generateData(kinectControl, objID, stepType, dir, csvName);

	end = std::chrono::high_resolution_clock::now();
	std::cout << "End Time : " << getCurTime() << std::endl;
	double elapsed = (std::chrono::duration_cast<std::chrono::milliseconds>(end - start)).count()/1000;
	printf("Time Taken (sec) : %1.2f\n",elapsed);
	printf("Data Collected : %d(Total), %d(Useful)\n",ctrData[0]+ctrData[1],ctrData[1]);
	printf("Time per data collected (sec) : %1.2f\n",elapsed/(ctrData[0]+ctrData[1]));
	printf("Data saved to : %s\n",csvName.c_str());
 }
