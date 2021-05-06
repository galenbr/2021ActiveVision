#include <active_vision/environment.h>
#include <active_vision/toolDataHandling.h>
#include <active_vision/toolViewPointCalc.h>
#include <active_vision/toolVisualization.h>
#include <active_vision/toolStateVector.h>
#include <active_vision/heuristicPolicySRV.h>
#include <active_vision/trainedPolicySRV.h>
#include <active_vision/controlSRV.h>
#include <active_vision/restartObjSRV.h>

int mode;
int HAFstVecGridSize;
environment *kC;
RouteData *home, *cur;
std::vector<double> startPose;
int cObj;

void updateRouteData(environment &env, RouteData &data, bool save ,std::string name){
	data.success = (env.graspID != -1);
	if(save == true){
		data.obj = *env.ptrPtCldObject;
		data.unexp = *env.ptrPtCldUnexp;
		data.childID = env.saveConfiguration(name);
	}
	if(data.success == true){
		if(save == true) data.goodInitialGrasp = true;
		data.graspQuality = env.graspData.quality;
		env.updateGripper(env.graspID,0);
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
	dirs[3] = (8+dir+2)%8+1; dirs[4] = (8+dir-2)%8+1;
	dirs[5] = (8+dir+3)%8+1; dirs[6] = (8+dir-3)%8+1;
	dirs[7] = (8+dir+4)%8+1;
	return dirs;
}

void resetEnv(){
	kC->spawnObject(cObj,0,0);
	startPose = {kC->viewsphereRad, 180*(M_PI/180.0), 45*(M_PI/180.0)};
	kC->reset();
	// Home Pose
	home = new RouteData;
	cur = new RouteData;
	home->reset();
	home->path = {startPose};
	home->type = 1;
	home->direction = 0;
	home->nSteps = 0;
	*cur = *home;
}

std::vector<float> readyStateVec(){
	static HAFStVec1 stVec;
	static ptCldColor::Ptr tempPtCldObj{new ptCldColor};
	static ptCldColor::Ptr tempPtCldUnexp{new ptCldColor};
	stVec.setGridDim(::HAFstVecGridSize);
	stVec.setMaintainScale(true);
	*tempPtCldObj = *kC->ptrPtCldObject;
	*tempPtCldUnexp = *kC->ptrPtCldUnexp;
	stVec.setInput(tempPtCldObj,tempPtCldUnexp,cur->path.back());
	stVec.calculate();
	return stVec.getStateVec();
}

bool checkDone(){
	return cur->success;
}

void print(std::vector<double> const &a){
	for(int i=0; i<a.size(); i++){
		std::cout << a.at(i) << " ";
	}
	std::cout << std::endl;
}

std::vector<double> move(int direction){
	std::vector<int> dirPref = nearbyDirections(direction);
	int prfID = 0;
	std::vector<double> nextPose = calcExplorationPose(cur->path.back(),dirPref[prfID],::mode);
	printf("Calculated next pose.\n");
	print(nextPose);
	while(checkValidPose(nextPose) == false || checkIfNewPose(cur->path,nextPose,::mode) == false){
		prfID++;
		nextPose = calcExplorationPose(cur->path.back(),dirPref[prfID],::mode);
		print(nextPose);
	}
	printf("Checked next pose.\n");
	if(prfID!=0){
		printf("Direction modified from %d -> %d.\n", dirPref[0],dirPref[prfID]);
	}
	std::cout << dirLookup[dirPref[prfID]] << ",";
	singlePass(*kC, nextPose, false, true, 2);
	printf("Finished single pass.\n");
	updateRouteData(*kC,*cur,false,"dummy");
	cur->path.push_back(nextPose);
	cur->nSteps++;
	if(cur->nSteps == 1){
		printf("Updating home direction.\n");
		home->direction = direction;
	}
	printf("Done with move.\n");
	return nextPose;
}

bool startObj(active_vision::restartObjSRV::Request  &req,
              active_vision::restartObjSRV::Response &res){
	printf("Starting reset...\n");
	kC->deleteObject(cObj);
	cObj = req.object;
	resetEnv();
	printf("Reset object.\n");
	kC->moveObject(cObj,req.objPoseCode,req.objYaw*(M_PI/180.0));
	home->objType = kC->objectDict[cObj].description;
	home->objPose = {kC->objectDict[cObj].poses[req.objPoseCode][1],
									kC->objectDict[cObj].poses[req.objPoseCode][2],
									req.objYaw*(M_PI/180.0)};
	printf("Reset home.\n");
	singlePass(*kC, startPose, true, true, 2);
	updateRouteData(*kC,*home,true,"Home");
	res.stateVec.data = readyStateVec();
	printf("Done with reset.\n");
	return true;
}

bool takeStep(active_vision::controlSRV::Request  &req,
              active_vision::controlSRV::Response &res){
	printf("Starting step...\n");
	move(req.direction);
	res.stateVec.data = readyStateVec();
	res.done = checkDone();
	res.steps = cur->nSteps;
	printf("Done with step.\n");
	return true;
}

int main(int argc, char** argv){
	chdir(getenv("HOME"));

	ros::init(argc, argv, "Policy_Tester");
 	ros::NodeHandle nh;
  	ros::ServiceClient policy;
 	environment av(&nh); 
 	kC = &av;
 	sleep(1);

	int objID;
	nh.getParam("/active_vision/policyTester/objID", objID);
	if(av.objectDict.count(objID) == 0){
		std::cout << "ERROR. Incorrect Object ID." << std::endl;
    return(-1);
	}
	cObj = objID;

	nh.getParam("/active_vision/cameraMode", ::mode);
	if(::mode < 1 && ::mode > 2) ::mode = 2;

	nh.getParam("/active_vision/policyTester/HAFstVecGridSize", ::HAFstVecGridSize);
	ros::ServiceServer service = nh.advertiseService("/active_vision/restartEnv", startObj);
	ros::ServiceServer service2 = nh.advertiseService("/active_vision/moveKinect", takeStep);
  	ROS_INFO("Camera control service ready.");
  	ros::spin();
  	return(0);
 }
