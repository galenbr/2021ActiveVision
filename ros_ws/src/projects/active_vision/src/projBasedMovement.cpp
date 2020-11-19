#include "active_vision/testingModel_v1.h"
#include "active_vision/dataHandling.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define MIN_ANGLE 20
#define MIN_ANGLE_RAD MIN_ANGLE*(M_PI/180.0)

int runMode;
int mode;
int dirChosen = 0;
bool called = false;
bool quit = false;

std::map<int, std::string> dirLookup{{0, "Reset"},
                                     {1, "N"}, {2, "NE"},
																		 {3, "E"}, {4, "SE"},
																		 {5, "S"}, {6, "SW"},
																	 	 {7, "W"}, {8, "NW"}};

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event){
  if(event.keyUp()){
    ::dirChosen = -1;
    if      (event.getKeySym() == "KP_5"){ ::dirChosen = 0; ::called = true;}
    else if (event.getKeySym() == "KP_1"){ ::dirChosen = 6; ::called = true;}
    else if (event.getKeySym() == "KP_2"){ ::dirChosen = 5; ::called = true;}
    else if (event.getKeySym() == "KP_3"){ ::dirChosen = 4; ::called = true;}
    else if (event.getKeySym() == "KP_4"){ ::dirChosen = 7; ::called = true;}
    else if (event.getKeySym() == "KP_6"){ ::dirChosen = 3; ::called = true;}
    else if (event.getKeySym() == "KP_7"){ ::dirChosen = 8; ::called = true;}
    else if (event.getKeySym() == "KP_8"){ ::dirChosen = 1; ::called = true;}
    else if (event.getKeySym() == "KP_9"){ ::dirChosen = 2; ::called = true;}
    else if (event.getKeySym() == "5"){ ::dirChosen = 0;    ::called = true;}
    else if (event.getKeySym() == "1"){ ::dirChosen = 6;    ::called = true;}
    else if (event.getKeySym() == "2"){ ::dirChosen = 5;    ::called = true;}
    else if (event.getKeySym() == "3"){ ::dirChosen = 4;    ::called = true;}
    else if (event.getKeySym() == "4"){ ::dirChosen = 7;    ::called = true;}
    else if (event.getKeySym() == "6"){ ::dirChosen = 3;    ::called = true;}
    else if (event.getKeySym() == "7"){ ::dirChosen = 8;    ::called = true;}
    else if (event.getKeySym() == "8"){ ::dirChosen = 1;    ::called = true;}
    else if (event.getKeySym() == "9"){ ::dirChosen = 2;    ::called = true;}
    else if (event.getKeySym() == "Escape"){ ::quit = true; ::called = true;}
  }
}

void initRGBViewer(ptCldVis::Ptr viewer, std::vector<int> &vp){

	viewer->createViewPort(0.0,0.0,0.5,1.0,vp[0]);
	viewer->createViewPort(0.5,0.0,1.0,1.0,vp[1]);
	viewer->addCoordinateSystem(1.0);
	viewer->setCameraPosition(-2,0,7,2,0,-1,1,0,2);
	viewer->registerKeyboardCallback(keyboardEventOccurred);
	viewer->setPosition(550,20);
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

Eigen::Affine3f tfKinect(std::vector<double> &pose){
  std::vector<double> cartesian = {0,0,0,0,0,0};
  cartesian[0] = 1.5+pose[0]*sin(pose[2])*cos(pose[1]);
  cartesian[1] = 0.0+pose[0]*sin(pose[2])*sin(pose[1]);
  cartesian[2] = 1.0+pose[0]*cos(pose[2]);
  cartesian[3] = 0;
  cartesian[4] = M_PI/2-pose[2];
  cartesian[5] = M_PI+pose[1];

  Eigen::Affine3f tfKinOptGaz,tfGazWorld;
  tfKinOptGaz = pcl::getTransformation(0,0,0,-M_PI/2,0,-M_PI/2);
  tfGazWorld = pcl::getTransformation(cartesian[0],cartesian[1],cartesian[2],\
                                      cartesian[3],cartesian[4],cartesian[5]);
  return(tfGazWorld * tfKinOptGaz);
}

void cleanUnexp(ptCldColor::Ptr obj, ptCldColor::Ptr unexp){
  pcl::PointXYZRGB minPtObj, maxPtObj;
  pcl::getMinMax3D(*obj, minPtObj, maxPtObj);

  // int scale = 3;
  // pcl::PointXYZRGB minPtUnexp, maxPtUnexp;
  // minPtUnexp.x = minPtObj.x - (scale-1)*(maxPtObj.x-minPtObj.x)/2;
  // minPtUnexp.y = minPtObj.y - (scale-1)*(maxPtObj.y-minPtObj.y)/2;
  // minPtUnexp.z = minPtObj.z;
  // maxPtUnexp.x = maxPtObj.x + (scale-1)*(maxPtObj.x-minPtObj.x)/2;
  // maxPtUnexp.y = maxPtObj.y + (scale-1)*(maxPtObj.y-minPtObj.y)/2;
  // maxPtUnexp.z = maxPtObj.z + 0.05;

  ptCldColor::ConstPtr cPtrUnexp{unexp};
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cPtrUnexp);
  // pass.setFilterFieldName("x"); pass.setFilterLimits(minPtUnexp.x,maxPtUnexp.x);
  // pass.filter(*unexp);
  // pass.setFilterFieldName("y"); pass.setFilterLimits(minPtUnexp.y,maxPtUnexp.y);
  // pass.filter(*unexp);
  pass.setFilterFieldName("z"); pass.setFilterLimits(minPtObj.z,maxPtObj.z + 0.05);
  pass.filter(*unexp);
}

int findVisibleUnexp(ptCldColor::Ptr obj, ptCldColor::Ptr unexp, std::vector<double> &pose, cv::Mat &res){
  Eigen::Affine3f tf = tfKinect(pose);
  ptCldColor::Ptr tempObj{new ptCldColor};
  ptCldColor::Ptr tempUnexp{new ptCldColor};
  pcl::transformPointCloud(*obj, *tempObj, homoMatTranspose(tf));
  pcl::transformPointCloud(*unexp, *tempUnexp, homoMatTranspose(tf));

  Eigen::MatrixXf projectionMat;
  projectionMat.resize(3,4);
  projectionMat << 554.254691191187, 0.0, 320.5, -0.0,
                   0.0, 554.254691191187, 240.5, 0.0,
                   0.0, 0.0, 1.0, 0.0;

  cv::Mat projObj = cv::Mat::zeros(480,640,CV_8UC1);
  cv::Mat projUnexp = cv::Mat::zeros(480,640,CV_8UC1);

  Eigen::Vector4f v4fTemp;
  Eigen::Vector3f proj;

  for(int i = 0; i < tempUnexp->width; i++){
    v4fTemp = tempUnexp->points[i].getVector4fMap();
    proj = projectionMat*v4fTemp;
    proj = proj/proj[2];
    proj[0] = (round(proj[0])-1);
    proj[1] = (round(proj[1])-1);
    if(proj[0] >= 0 && proj[0] < 640 && proj[1] >= 0 && proj[1] < 480){
      uchar &intensity = projUnexp.at<uchar>(proj[1],proj[0]);
      intensity = 255;
    }
  }
  for(int i = 0; i < tempObj->width; i++){
    v4fTemp = tempObj->points[i].getVector4fMap();
    proj = projectionMat*v4fTemp;
    proj = proj/proj[2];
    proj[0] = (round(proj[0])-1);
    proj[1] = (round(proj[1])-1);
    if(proj[0] >= 0 && proj[0] < 640 && proj[1] >= 0 && proj[1] < 480){
      uchar &intensity = projObj.at<uchar>(proj[1],proj[0]);
      intensity = 255;
    }
  }

  int sA = 3;
  cv::Mat element = cv::getStructuringElement(0, cv::Size(2*sA+1,2*sA+1), cv::Point(sA,sA));
  cv::morphologyEx(projObj,projObj,3,element);
  cv::morphologyEx(projUnexp,projUnexp,3,element);

  cv::Mat sub;
  cv::subtract(projUnexp, projObj, sub);

  double s = cv::sum(sub)[0]/255;

	cv::Rect roi;
  roi.x = 640/6;
  roi.y = 480/6;
  roi.width = 640 - (640/6*2);
  roi.height = 480 - (480/6*2);

  cv::hconcat(projObj(roi), projUnexp(roi), res);
  cv::hconcat(res, sub(roi), res);
  cv::resize(res, res, cv::Size(), 0.35, 0.35);
  cv::Scalar value(255);
  copyMakeBorder(res, res, 2, 2, 2, 2, cv::BORDER_CONSTANT, value);

  return(s);
}

std::vector<int> findVisibleUnexp8Dir(ptCldColor::Ptr obj, ptCldColor::Ptr unexp, std::vector<double> &pose){
	ptCldColor::Ptr tempObj{new ptCldColor};
  ptCldColor::Ptr tempUnexp{new ptCldColor};
	*tempObj = *obj; *tempUnexp = *unexp;
	cleanUnexp(tempObj,tempUnexp);
  std::vector<int> res;
  int temp;
  std::vector<double> newPose;
  cv::Mat single, final;
  for(int i = 1; i <= 8; i++){
    newPose = calcExplorationPose(pose,i);
    if(checkValidPose(newPose) == true){
      temp = findVisibleUnexp(tempObj,tempUnexp,newPose,single);
      cv::Size s = single.size();
      // std::cout << s.height << "," << s.width << std::endl;
      cv::putText(single, //target image
            dirLookup[i]+":"+std::to_string(temp), //text
            cv::Point(5, 20), //top-left position
            cv::FONT_HERSHEY_DUPLEX,
            0.5,
            cv::Scalar(255), //font color
            1);
    }else{
      temp = 0;
      single = cv::Mat::zeros(116,453,CV_8UC1);
    }
    if(i > 1) cv::vconcat(final, single, final);
    else final = single;

    res.push_back(temp);
  }
	cv::imshow("Projection",final);
	cv::moveWindow("Projection",20,20);
	cv::waitKey(200);
  return(res);
}

void updateRouteData(environment &env, RouteData &data, bool save ,std::string name){
	ptCldColor::Ptr tempUnexp{new ptCldColor};
	data.success = (env.selectedGrasp != -1);
	if(save == true){
		data.obj = *env.ptrPtCldObject;
		*tempUnexp = *env.ptrPtCldUnexp;
		cleanUnexp(env.ptrPtCldObject,tempUnexp);
		data.unexp = *tempUnexp;
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

void viewerSetCamPose(ptCldVis::Ptr viewer, std::vector<double> pose){
	pose[0]*=1.5;
	pcl::PointXYZ temp;
	temp = sphericalToCartesian(pose);
	temp.x+=1.5; temp.z+=1;
	viewer->setCameraPosition(temp.x,temp.y,temp.z,1.5,0,1,0,0,1);
}

// Function the find the grasp
void findGrasp(environment &kinectControl, int object, int objPoseCode, int objYaw, std::string dir, std::string saveLocation, ptCldVis::Ptr viewer){
	fstream fout;
 	fout.open(dir+saveLocation, ios::out | ios::app);

  kinectControl.spawnObject(object,0,0);

 	std::vector<double> startPose = {1.4, 180*(M_PI/180.0), 45*(M_PI/180.0)};
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
  singlePass(kinectControl, startPose, true, true);
	updateRouteData(kinectControl,home,true,"Home");
	current = home;

	std::vector<int> vp = {0,0,0};
	initRGBViewer(viewer,vp);

	ptCldColor::Ptr tempPtCld{new ptCldColor};
	pcl::PointXYZ table,a1,a2;
  table.x = 1.5; table.y = 0; table.z = 1;
	std::vector<int> nUnexp;

	while(::quit == false){
		viewer->resetStoppedFlag();
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		rbgVis(viewer,kinectControl.ptrPtCldObject,"Obj",vp[0]);
		*tempPtCld = *kinectControl.ptrPtCldUnexp;
		cleanUnexp(kinectControl.ptrPtCldObject,tempPtCld);
		rbgVis(viewer,tempPtCld,"Unexp",vp[0]);
		*tempPtCld = current.env; rbgVis(viewer,tempPtCld,"Env",vp[1]);

		viewer->addSphere(table,current.path[0][0],0,0,1,"Viewsphere",vp[1]);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,"Viewsphere");
		a1 = sphericalToCartesian(current.path[0]);   a1.x += table.x; a1.y += table.y; a1.z += table.z;
		viewer->addSphere(a1,0.04,1,0,0,"Start",vp[1]);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,"Start");
		for(int i = 0; i < current.path.size()-1; i++){
			a1 = sphericalToCartesian(current.path[i]);   a1.x += table.x; a1.y += table.y; a1.z += table.z;
			a2 = sphericalToCartesian(current.path[i+1]); a2.x += table.x; a2.y += table.y; a2.z += table.z;
			viewer->addArrow(a2,a1,0,1,0,false,std::to_string(i),vp[1]);
		}
		viewerSetCamPose(viewer,current.path.back());
		int maxIndex = 0;
		if(current.success == true){
			// std::cout << "GRASP FOUND. Saving and exiting." << std::endl;
			current.filename = getCurTime();
			saveData(current, fout, dir);
			viewer->addText("GRASP FOUND. Saving and exiting.",4,5,25,1,0,0,"Dir1",vp[1]);
		} else{
			nUnexp = findVisibleUnexp8Dir(kinectControl.ptrPtCldObject,kinectControl.ptrPtCldUnexp,current.path.back());
			maxIndex = std::max_element(nUnexp.begin(),nUnexp.end()) - nUnexp.begin();
			viewer->addText("Best direction calculated : " + dirLookup[maxIndex+1],4,5,25,1,0,0,"Dir1",vp[1]);
			// std::cout << "Best direction calculated : " << dirLookup[maxIndex+1] << "." << std::flush;
		}

		if(::runMode == 1){
			::called = false;
			while(!viewer->wasStopped() && ::called==false){
				viewer->spinOnce(100);
				boost::this_thread::sleep (boost::posix_time::microseconds(100000));
			}
		}else{
			viewer->spinOnce(100);
			boost::this_thread::sleep (boost::posix_time::microseconds(100000));
			::dirChosen = maxIndex+1;
		}

		if(current.success == true){
			::dirChosen = 0;
			::quit = true;
		}

		// Reset to home if direction is 0
		if(::dirChosen == 0){
			// if(current.success != true){
			// 	std::cout << " Direction Setected : " << dirLookup[::dirChosen] << "." << std::flush;
			// }
			kinectControl.rollbackConfiguration(home.childID);
			current = home;
			current.nSteps = 0;
		}

		if(::dirChosen >=1 && ::dirChosen <= 8){
			// std::cout << " Direction Setected : " << dirLookup[::dirChosen] << "." << std::flush;
      nextPose = calcExplorationPose(current.path.back(),::dirChosen);
			if(checkValidPose(nextPose) == true){
				singlePass(kinectControl, nextPose, false, true);
				updateRouteData(kinectControl,current,false,"dummy");
				current.path.push_back(nextPose);
				current.nSteps++;
				if(current.nSteps == 1) home.direction = ::dirChosen;
			}
    }

		// std::cout << std::endl;
	}

	::quit = false;
	kinectControl.deleteObject(object);
	kinectControl.reset();
	fout.close();
}

void help(){
  std::cout << "******* Supervisor Node Help *******" << std::endl;
  std::cout << "Arguments : [CSV filename] [Object] [MoveMode] [RunMode]" << std::endl;
  std::cout << "CSV filename : CSV file name (Eg:data.csv) (Use \"default.csv\" to use time as the file name)" << std::endl;
	std::cout << "Object : Object ID -> 0(Drill),1(Sq Prism),2(Rect Prism)" << std::endl;
	std::cout << "MoveMode : 1->Normal, 2->New" << std::endl;
	std::cout << "RunMode : 0->Auto, 1->Manual" << std::endl;
  std::cout << "*******" << std::endl;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "Projection_Based");

	if(argc != 5){
    std::cout << "ERROR. Incorrect number of arguments." << std::endl;
    help(); return(-1);
  }

 	ros::NodeHandle nh;
 	environment kinectControl(&nh); sleep(1);
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
	if(objID < 0 && objID > 7) objID = 0;
	::mode = std::atoi(argv[3]);
	if(::mode < 1 && ::mode > 2) ::mode = 1;
	if(std::atoi(argv[4]) == 0) ::runMode = 0;
	else ::runMode = 1;

	ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));

	int objPoseCode, objYaw;
	if(::runMode == 1){
		printf("Enter the object pose code (0-%d) : ", int(kinectControl.objectPosesDict[objID].size()-1));
		std::cin >> objPoseCode;
		if(objPoseCode < 0 || objPoseCode > kinectControl.objectPosesDict[objID].size()-1) objPoseCode = 0;
		printf("Enter the object yaw (deg) (0-360) : ");
		std::cin >> objYaw;

		findGrasp(kinectControl, objID, objPoseCode, objYaw, dir, csvName, viewer);

	}else{
		for(int objPoseCode = 0; objPoseCode < kinectControl.objectPosesDict[objID].size(); objPoseCode+=1){
			for(int objYaw = kinectControl.objectPosesYawLimits[objID][0]; objYaw < kinectControl.objectPosesYawLimits[objID][1]; objYaw+=40){
				findGrasp(kinectControl, objID, objPoseCode, objYaw, dir, csvName, viewer);
			}
		}
	}

	printf("Data saved to : %s\n",csvName.c_str());
 }
