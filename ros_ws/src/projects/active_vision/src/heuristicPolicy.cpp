#include "active_vision/environment.h"
#include "active_vision/toolDataHandling.h"
#include "active_vision/toolViewPointCalc.h"
#include "active_vision/toolVisualization.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int runMode;
int mode;

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

  ptCldColor::ConstPtr cPtrUnexp{unexp};
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cPtrUnexp);
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
    newPose = calcExplorationPose(pose,i,::mode);
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

// Function the find the grasp
void findGrasp(environment &kinectControl, int object, int objPoseCode, int objYaw, std::string dir, std::string saveLocation, ptCldVis::Ptr viewer){
	std::fstream fout;
 	fout.open(dir+saveLocation, std::ios::out | std::ios::app);

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

	std::vector<int> vp;
	setupViewer(viewer, 2, vp);
  viewer->setPosition(550,20);
  keyboardEvent keyPress(viewer,2); keyPress.help();

	ptCldColor::Ptr tempPtCld{new ptCldColor};
	pcl::PointXYZ table,a1,a2;
  table.x = 1.5; table.y = 0; table.z = 1;
	std::vector<int> nUnexp;

	while(keyPress.ok){
		viewer->resetStoppedFlag();
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		addRGB(viewer,kinectControl.ptrPtCldObject,"Obj",vp[0]);
		*tempPtCld = *kinectControl.ptrPtCldUnexp;
		cleanUnexp(kinectControl.ptrPtCldObject,tempPtCld);
		addRGB(viewer,tempPtCld,"Unexp",vp[0]);
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
			viewer->addText("GRASP FOUND. Saving and exiting.",4,5,25,1,0,0,"Dir1",vp[1]);
		} else{
			nUnexp = findVisibleUnexp8Dir(kinectControl.ptrPtCldObject,kinectControl.ptrPtCldUnexp,current.path.back());
			maxIndex = std::max_element(nUnexp.begin(),nUnexp.end()) - nUnexp.begin();
			viewer->addText("Best direction calculated : " + std::to_string(maxIndex+1) + "(" + dirLookup[maxIndex+1] + ")",4,5,25,1,0,0,"Dir1",vp[1]);
		}

		if(::runMode == 1){
			keyPress.called = false;
			while(!viewer->wasStopped() && keyPress.called==false){
				viewer->spinOnce(100);
				boost::this_thread::sleep (boost::posix_time::microseconds(100000));
			}
		}else{
			viewer->spinOnce(100);
			boost::this_thread::sleep (boost::posix_time::microseconds(100000));
			keyPress.dir = maxIndex+1;
		}

		if(current.success == true){
			keyPress.dir = 0;
			keyPress.ok = false;
		}

		// Reset to home if direction is 0
		if(keyPress.dir == 0){
			kinectControl.rollbackConfiguration(home.childID);
			current = home;
			current.nSteps = 0;
		}

		if(keyPress.dir >=1 && keyPress.dir <= 8){
      nextPose = calcExplorationPose(current.path.back(),keyPress.dir,::mode);
			if(checkValidPose(nextPose) == true){
				singlePass(kinectControl, nextPose, false, true);
				updateRouteData(kinectControl,current,false,"dummy");
				current.path.push_back(nextPose);
				current.nSteps++;
				if(current.nSteps == 1) home.direction = keyPress.dir;
			}
    }
	}

	keyPress.ok = true;
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
