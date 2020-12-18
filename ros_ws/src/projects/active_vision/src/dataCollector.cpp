#include "active_vision/environment.h"
#include "active_vision/toolDataHandling.h"
#include "active_vision/toolViewPointCalc.h"

bool saveOnlyUsefulData = true;
int mode = 1;

std::vector<int> randomDirection(){
	std::vector<int> dirs = {1,2,3,4,5,6,7,8};
	std::vector<int> ret;
	int randIndex;
	while(dirs.size() > 0){
		randIndex = rand() % (dirs.size());
		ret.push_back(dirs[randIndex]);
		dirs.erase(dirs.begin()+randIndex);
	}
	return ret;
}

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
  std::vector<int> dirs = randomDirection();
	for (int i = 1; i <= 8 && temp.success == false; i++){
		// Rollback to the parent/home configuration
		env.rollbackConfiguration(temp.parentID);
		curPose = calcExplorationPose(env.lastKinectPoseViewsphere,dirs[i-1],::mode);
		std::cout << "." << std::flush;
		// If kinect pose if above the table, then proceed in that direction
		if (checkValidPose(curPose) == true && checkIfNewPose(home.path,curPose,::mode) == true){
			// Saving the direction and path taken
			temp.direction = dirs[i-1];
			temp.path = home.path;
			temp.path.push_back(curPose);

			// Doing a pass and saving the child
			singlePass(env, curPose, false, true);
			temp.success = (env.selectedGrasp != -1);
			temp.childID = env.saveConfiguration(env.configurations[temp.parentID].description+"_C"+std::to_string(dirs[i-1]));

			// Exit the loop and updating variables used for visualization
			if(temp.success == true){
				temp.graspQuality = env.graspsPossible[env.selectedGrasp].quality;
				env.updateGripper(env.selectedGrasp,0);
				temp.env = *env.ptrPtCldEnv + *env.ptrPtCldGripper;
			}else{
        temp.graspQuality = -1;
        temp.env = *env.ptrPtCldEnv;
      }
			temp.env += *env.ptrPtCldUnexp;

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

  opData.success = false;
  // Going taking a max of nSteps from the parent position
	for (int i = 1; i <= nSteps && opData.success == false; i++){
    // Do until the pose is valid
		std::vector<int> dirs = randomDirection();
		bool foundNextPose = false;
    for(int j = 0; j < 7; j++) {
    	curPose = calcExplorationPose(env.lastKinectPoseViewsphere,dirs[j],::mode);
			if(checkValidPose(curPose) == true && checkIfNewPose(opData.path,curPose,::mode) == true){
				foundNextPose = true;
				break;
			}
    }

		if(foundNextPose == true){
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
				opData.env += *env.ptrPtCldUnexp;
			}
		}else{
			break;
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
	data.env += *env.ptrPtCldUnexp;
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
  		newPose1 = calcExplorationPose(stPose,i,::mode);
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
			newPose1 = calcExplorationPose(stPose,i,::mode);
			if(checkValidPose(newPose1) == true){
				for(int j = 1; j <= 8; j++){
					temp.path = {stPose,newPose1};
					newPose2 = calcExplorationPose(newPose1,j,::mode);
					if(checkValidPose(newPose2) == true && checkIfNewPose(temp.path,newPose2,::mode) == true){
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
	std::fstream fout;
 	fout.open(dir+saveLocation, std::ios::out | std::ios::app);

  kinectControl.spawnObject(object,0,0);

 	std::vector<double> startPose = {1.4, 180*(M_PI/180.0), 45*(M_PI/180.0)};

  bool graspFound = false;
	RouteData dataFinal, dataFinalTemp;
	std::vector<RouteData> dataOneStep,dataHomePoses;

  int maxRndSteps = 5;
  int nHomeConfigs = 0;

	for (int currentPoseCode = 0; currentPoseCode < kinectControl.objectPosesDict[object].size(); currentPoseCode+=1){
		for (int currentYaw = kinectControl.objectPosesYawLimits[object][0]; currentYaw < kinectControl.objectPosesYawLimits[object][1]; currentYaw+=10){
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
          dataFinal.filename = getCurTime()+"_"+std::to_string(nSaved[0]+nSaved[1]);

          if(graspFound == true) printf(" OK. %d Steps, Dir : %d.\n",dataFinal.nSteps,dataFinal.direction);

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

	std::string dir;
	nh.getParam("/active_vision/data_dir", dir);
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
