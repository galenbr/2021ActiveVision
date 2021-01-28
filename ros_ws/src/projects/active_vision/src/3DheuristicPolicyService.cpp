#include <active_vision/environment.h>
#include <active_vision/toolViewPointCalc.h>
#include <active_vision/toolVisualization.h>
#include <active_vision/heuristicPolicySRV.h>
// #include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <set>

float voxelGridSize;

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

Eigen::Affine3f calcTfFromNormal(pcl::Normal normal, pcl::PointXYZRGB point){
  Eigen::Matrix3f rot;
  Eigen::Vector3f trans;

  Eigen::Matrix4f tfMat; tfMat.setIdentity();
  Eigen::Affine3f tf;

  Eigen::Vector3f xAxis,yAxis,zAxis,xyPlane(0,0,1);

  xAxis = {normal.normal_x,normal.normal_y,normal.normal_z}; xAxis.normalize();
  yAxis = xAxis.cross(xyPlane); yAxis.normalize();
  zAxis = xAxis.cross(yAxis);

  rot << xAxis[0], yAxis[0], zAxis[0],
         xAxis[1], yAxis[1], zAxis[1],
         xAxis[2], yAxis[2], zAxis[2];
  trans = point.getVector3fMap();

  tfMat.block<3,3>(0,0) = rot;
  tfMat.block<3,1>(0,3) = trans;
  tf.matrix() = tfMat;

  return tf;
}

void calcTfFromNormal(pcl::Normal normal, pcl::PointXYZRGB point, Eigen::Matrix3f &rot, Eigen::Vector3f &trans){
  Eigen::Matrix4f tfMat; tfMat.setIdentity();
  Eigen::Affine3f tf;

  Eigen::Vector3f xAxis,yAxis,zAxis,xyPlane(0,0,1);

  xAxis = {normal.normal_x,normal.normal_y,normal.normal_z}; xAxis.normalize();
  yAxis = xAxis.cross(xyPlane); yAxis.normalize();
  zAxis = xAxis.cross(yAxis);

  rot << xAxis[0], yAxis[0], zAxis[0],
         xAxis[1], yAxis[1], zAxis[1],
         xAxis[2], yAxis[2], zAxis[2];
  trans = point.getVector3fMap();
}

// For object point cloud
bool objCheckIfNotOccluded(pcl::PointXYZRGB home, pcl::PointXYZRGB check, pcl::Normal normal, float radius){

  // Plane
  Eigen::Vector3f n = {normal.normal_x,normal.normal_y,normal.normal_z};
  Eigen::Vector3f p0 = {check.x,check.y,check.z};
  // Line
  Eigen::Vector3f l = {-home.x,-home.y,-home.z};
  Eigen::Vector3f l0 = {home.x,home.y,home.z};

  float d = 999;
  if(l.dot(n) != 0)
    d = ((p0 - l0).dot(n)) / (l.dot(n));

  pcl::PointXYZRGB intersectionPt;
  intersectionPt.getVector3fMap() = l0 + l*d;
  float distance = pcl::euclideanDistance(check,intersectionPt);

  // std::cout << (distance > radius) << "," << d << "," << distance << std::endl;

  if(d <= 0 || distance > radius) return true;
  else                            return false;
}

// For unexplored point cloud
bool unexpCheckIfNotOccluded(pcl::PointXYZRGB home, pcl::PointXYZRGB check,float radius){

  float disEuc = pcl::euclideanDistance(home,check);
  float disPerpVP = 0;
  if(disEuc > 0){
    Eigen::Vector3f homeToVP,homeToCheck;
    homeToVP = {-home.x,-home.y,-home.z}; homeToVP.normalize();
    homeToCheck = {check.x-home.x,check.y-home.y,check.z-home.z}; homeToCheck.normalize();
    float angle = atan2(homeToVP.cross(homeToCheck).norm(), homeToVP.dot(homeToCheck));
    disPerpVP = disEuc*sin(angle);
  }

  if(disEuc > 0 && disPerpVP < radius*0.5 && disEuc > radius) return false;
  return true;
}

void extractUsefulUnexpPts(ptCldColor::Ptr obj, ptCldColor::Ptr unexp, ptCldColor::Ptr res){
  res->clear();

  // Calculating the normal vectors
  ptCldNormal::Ptr normal{new ptCldNormal};
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;  // Normal Estimation
  pcl::search::Search<pcl::PointXYZRGB>::Ptr KdTree{new pcl::search::KdTree<pcl::PointXYZRGB>};
  ptCldColor::ConstPtr cObj{obj};
  ne.setInputCloud(cObj);
  ne.setSearchMethod(KdTree);
  ne.setKSearch(10);
  ne.compute(*normal);

  pcl::PointXYZRGB minPtObj, maxPtObj;
  pcl::getMinMax3D(*obj, minPtObj, maxPtObj);

  pcl::PointXYZRGB min,max;
  min.x = -0.080; max.x = 0.080;
  min.y = -0.015; max.y = 0.015;
  min.z = -0.015; max.z = 0.015;

  Eigen::Affine3f tf;

  pcl::CropBox<pcl::PointXYZRGB> cpBox;
  cpBox.setInputCloud(unexp);
  cpBox.setMin(min.getVector4fMap());
  cpBox.setMax(max.getVector4fMap());

  std::vector<int> tempIndices;
  std::set<int> usefulIndices;

  for(int i = 0; i < normal->width; i++){
    tf = calcTfFromNormal(normal->points[i],obj->points[i]);

    cpBox.setRotation(getEuler(tf));
    cpBox.setTranslation(getTranslation(tf));
    cpBox.filter(tempIndices);

    for(int j = 0; j < tempIndices.size(); j++){
      usefulIndices.insert(tempIndices[j]);
    }
  }

  // Extracting the indices
  for(auto idx : usefulIndices){
    if(unexp->points[idx].z >= minPtObj.z*1.01){
      res->points.push_back(unexp->points[idx]);
      res->points.back().r = 0;
      res->points.back().g = 200;
      res->points.back().b = 0;
    }
  }
  res->width = res->points.size();
  res->height = 1;
}

std::set<int> findVisibleUsefulUnexp(ptCldColor::Ptr obj, ptCldColor::Ptr usefulUnexp, std::vector<double> &pose){
  Eigen::Affine3f tf = tfKinect(pose);
  ptCldColor::Ptr tempObj{new ptCldColor};          pcl::transformPointCloud(*obj, *tempObj, homoMatTranspose(tf));
  ptCldColor::Ptr tempUsefulUnexp{new ptCldColor};  pcl::transformPointCloud(*usefulUnexp, *tempUsefulUnexp, homoMatTranspose(tf));

  // Calculating the transoformed normal vectors
  ptCldNormal::Ptr tempNormal{new ptCldNormal};
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;  // Normal Estimation
  pcl::search::Search<pcl::PointXYZRGB>::Ptr KdTree{new pcl::search::KdTree<pcl::PointXYZRGB>};
  ptCldColor::ConstPtr ctempObj{tempObj};
  ne.setInputCloud(ctempObj);
  ne.setSearchMethod(KdTree);
  ne.setKSearch(10);
  ne.compute(*tempNormal);

  // float coneAngle = calcConeAngle(pose[1],pose[2]);

  float radius = ::voxelGridSize/sqrt(2)*1.1;
  pcl::PointXYZRGB min,max;
  min.y = -radius;  max.y = radius;
  min.z = -radius;  max.z = radius;

  Eigen::Affine3f tf1;
  pcl::CropBox<pcl::PointXYZRGB> cpBoxObj; cpBoxObj.setInputCloud(tempObj);

  pcl::PointXYZRGB cam; cam.x = 0; cam.y = 0; cam.z = 0;

  std::vector<int> tempIndices;
  std::set<int> objClearIndices; objClearIndices.clear();
  std::set<int> finalIndices; finalIndices.clear();

  for(int i = 0; i < tempUsefulUnexp->width; i++){
    pcl::Normal normal, line;
    normal.normal_x = -1*tempUsefulUnexp->points[i].x;
    normal.normal_y = -1*tempUsefulUnexp->points[i].y;
    normal.normal_z = -1*tempUsefulUnexp->points[i].z;
    tf1 = calcTfFromNormal(normal,tempUsefulUnexp->points[i]);
    bool notOccluded = true;

    // Checking for occlusion with object
    min.x = -0.01; max.x = sqrt(pow(tempUsefulUnexp->points[i].x,2)+pow(tempUsefulUnexp->points[i].y,2)+pow(tempUsefulUnexp->points[i].z,2));
    cpBoxObj.setMin(min.getVector4fMap()); cpBoxObj.setMax(max.getVector4fMap());
    cpBoxObj.setRotation(getEuler(tf1));   cpBoxObj.setTranslation(getTranslation(tf1));
    cpBoxObj.filter(tempIndices);

    for(int j = 0; j < tempIndices.size(); j++){
      notOccluded *= objCheckIfNotOccluded(tempUsefulUnexp->points[i],tempObj->points[tempIndices[j]],tempNormal->points[tempIndices[j]],radius);
    }

    if(notOccluded) objClearIndices.insert(i);
  }

  // Check of the object cleared useful unxplored occlude each other
  for(auto i : objClearIndices){
    bool notOccluded = true;
    for(auto j : objClearIndices){
      if(i == j) continue;
      float disI = pcl::euclideanDistance(cam,tempUsefulUnexp->points[i]);
      float disJ = pcl::euclideanDistance(cam,tempUsefulUnexp->points[j]);
      // J in in front of I
      if(disJ < disI)  notOccluded = unexpCheckIfNotOccluded(tempUsefulUnexp->points[i],tempUsefulUnexp->points[j],::voxelGridSize);
      if(!notOccluded) break;
    }
    if(notOccluded) finalIndices.insert(i);
  }

  return finalIndices;
}

// int findDirection(ptCldColor::Ptr obj, ptCldColor::Ptr unexp, std::vector<double> &pose, int mode, int minVis, ptCldVis::Ptr viewer){
int findDirection(ptCldColor::Ptr obj, ptCldColor::Ptr unexp, std::vector<double> &pose, int mode, int minVis){

  ptCldColor::Ptr usefulUnexp{new ptCldColor};
  extractUsefulUnexpPts(obj,unexp,usefulUnexp);

  int threshold = 75;
  std::set<int> visibleIndices;
  std::vector<std::set<int>> indices;
  std::vector<int> res = {0,0,0,0,0,0,0,0};
  std::vector<int> step = {0,0,0,0,0,0,0,0};
  int maxRes = 0;

  for(int i = 1 ; i <= 8; i++){
    std::vector<double> newPose = calcExplorationPose(pose,i,mode); step[i-1] = 20;

    if(checkValidPose(newPose)) visibleIndices = findVisibleUsefulUnexp(obj,usefulUnexp,newPose);
    else                        visibleIndices.clear();
    indices.push_back(visibleIndices);
  }
  for(int i = 0 ; i < 8; i++) res[i] = indices[i].size();
  maxRes = *std::max_element(res.begin(), res.end());

  if(maxRes < minVis){
    // std::cout << "Two Check" << std::endl;
    threshold = 80;
    for(int i = 1 ; i <= 8; i++){
      std::vector<double> newPose = calcExplorationPose(pose,i,mode,40*(M_PI/180.0)); step[i-1] = 40;
      if(!checkValidPose(newPose)){
        newPose = calcExplorationPose(pose,i,mode,20*(M_PI/180.0)); step[i-1] = 20;
      }

      if(checkValidPose(newPose)) visibleIndices = findVisibleUsefulUnexp(obj,usefulUnexp,newPose);
      else                        visibleIndices.clear();
      indices[i-1].insert(visibleIndices.begin(),visibleIndices.end());
    }
    for(int i = 0 ; i < 8; i++) res[i] = indices[i].size();
    maxRes = *std::max_element(res.begin(), res.end());
  }

  if(maxRes < minVis){
    // std::cout << "Three Check" << std::endl;
    threshold = 85;
    for(int i = 1 ; i <= 8; i++){
      std::vector<double> newPose = calcExplorationPose(pose,i,mode,60*(M_PI/180.0)); step[i-1] = 60;
      if(!checkValidPose(newPose)){
        newPose = calcExplorationPose(pose,i,mode,40*(M_PI/180.0)); step[i-1] = 40;
      }
      if(!checkValidPose(newPose)){
        newPose = calcExplorationPose(pose,i,mode,20*(M_PI/180.0)); step[i-1] = 20;
      }

      if(checkValidPose(newPose)) visibleIndices = findVisibleUsefulUnexp(obj,usefulUnexp,newPose);
      else                        visibleIndices.clear();
      indices[i-1].insert(visibleIndices.begin(),visibleIndices.end());
    }
    for(int i = 0 ; i < 8; i++) res[i] = indices[i].size();
    maxRes = *std::max_element(res.begin(), res.end());
  }

  for(int i = 0 ; i < 8; i++){
    if(maxRes != 0){
      // std::cout << obj->points.size() << "," << i+1 << "->" << res[i] << "," << round(res[i]*100.0/maxRes) << std::endl;
      res[i] = round(res[i]*100.0/maxRes);
    }
    else{
      res[i] = 100;
    }
  }

  /***/
  // std::vector<int> vp;
  // setupViewer(viewer, 9, vp);
  // pcl::PointXYZ table;
	// table.x = 1.5; table.y = 0; table.z = 1;
  // setCamView(viewer,{pose[0]/2,pose[1],pose[2]},table,vp[0]);
  // addRGB(viewer,obj,"obj0",vp[0]);
  // addRGB(viewer,unexp,"unexp",vp[0]);
  // addRGB(viewer,usefulUnexp,"usefulUnexp",vp[0]);
  // viewer->removeCoordinateSystem();
  //
  // ptCldColor::Ptr result{new ptCldColor};
  // for(int i = 1 ; i <= 8; i++){
  //   result->clear();
  //   for(auto idx : indices[i-1]){
  //     result->points.push_back(usefulUnexp->points[idx]);
  //     result->points.back().r = 255;
  //     result->points.back().g = 255;
  //     result->points.back().b = 255;
  //   }
  //   std::vector<double> newPose = calcExplorationPose(pose,i,mode,step[i-1]*(M_PI/180.0));
  //   setCamView(viewer,{newPose[0]/2,newPose[1],newPose[2]},table,vp[i]);
  //   addRGB(viewer,obj,"obj"+std::to_string(i),vp[i]);
  //   addRGB(viewer,result,"result"+std::to_string(i),vp[i]);
  //   viewer->addText(std::to_string(step[i-1])+","+std::to_string(res[i-1]),2,2,20,1,0,0,"data"+std::to_string(i),vp[i]);
  // }
  // while(!viewer->wasStopped()){
  //   viewer->spinOnce(100);
  //   boost::this_thread::sleep (boost::posix_time::microseconds(100000));
  // }
  // viewer->resetStoppedFlag();
  // viewer->removeAllPointClouds();
  // viewer->removeAllShapes();
  /***/

  return std::distance(res.begin(), std::max_element(res.begin(), res.end())) + 1;

  // DIrection pref order 1,(2,8),(3,7),(4,6),5
  // if(res[1-1] >= threshold){
  //   return 1;
  // }else if(res[2-1] >= threshold || res[8-1] >= threshold){
  //   if(res[2-1] > res[8-1]) return 2;
  //   else return 8;
  // }else if(res[3-1] >= threshold || res[7-1] >= threshold){
  //   if(res[3-1] > res[7-1]) return 3;
  //   else return 7;
  // }else if(res[4-1] >= threshold || res[6-1] >= threshold){
  //   if(res[4-1] > res[6-1]) return 4;
  //   else return 6;
  // }else if(res[5-1] >= threshold){
  //   return 5;
  // }else{
  //   std::cout << "ERROR in direction calculation" << std::endl;
  //   return 1;
  // }
}

bool heuristicPolicy(active_vision::heuristicPolicySRV::Request  &req,
                     active_vision::heuristicPolicySRV::Response &res){

  static ptCldColor::Ptr obj{new ptCldColor};
  static ptCldColor::Ptr unexp{new ptCldColor};
  static std::vector<std::vector<double>> path;
  static std::vector<double> temp;
  static int mode;
  static std::vector<int> nUnexp;

  pcl::fromROSMsg(req.object, *obj);
  pcl::fromROSMsg(req.unexplored, *unexp);
  temp = req.path.data; path.clear();
  for(int i = 0; i < temp.size(); i=i+3){
    path.push_back({temp[i],temp[i+1],temp[i+2]});
  }
  mode = req.mode;

  res.direction = findDirection(obj,unexp,path.back(),mode,req.minPtsVis)-1;

  ROS_INFO_STREAM("Heuristic Policy Service Called. Direction -> "<< dirLookup[res.direction+1]);

  return true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "HeuristicPolicyServer");

 	ros::NodeHandle nh;
  nh.getParam("/active_vision/environment/voxelGridSize", ::voxelGridSize);
  ros::ServiceServer service = nh.advertiseService("/active_vision/heuristic_policy", heuristicPolicy);
  ROS_INFO("3D Heuristic policy service ready.");
  ros::spin();
  return 0;
 }

// int main (int argc, char** argv){
//   // Initialize ROS
//   ros::init(argc, argv, "Test");
//   ros::NodeHandle nh;
//   nh.getParam("/active_vision/environment/voxelGridSize", ::voxelGridSize);
//   environment activeVision(&nh);
//
//   // 4 kinect position to capture and fuse
//   std::vector<std::vector<double>> kinectPoses = {{1.0,M_PI,M_PI/4},
//                                                   {1.0,M_PI/2,M_PI/4},
//                                                   {1.0,0,M_PI/4},
//                                                   {1.0,M_PI/2,M_PI/4}};
//
//   // Delay to ensure all publishers and subscribers are connected
//   boost::this_thread::sleep(boost::posix_time::milliseconds(500));
//
//   if(argc != 4){
//     std::cout << "ERROR. Incorrect number of arguments." << std::endl;
//     return(-1);
//   }
//
//   int objID = std::atoi(argv[1]);
//   int pose = std::atoi(argv[2]);
//   int yaw = std::atoi(argv[3]);
//   int dir;
//   std::vector<double> newPose = kinectPoses[0];
//   activeVision.spawnObject(objID,pose,yaw*M_PI/180);
//
//   singlePass(activeVision,newPose,true,true,2);
//   int minPtsVis = 0.025*(3*activeVision.ptrPtCldObject->points.size());
//
//   ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
//   int nSteps = 0;
//   while(activeVision.selectedGrasp == -1 && nSteps <= 5){
//     dir = findDirection(activeVision.ptrPtCldObject,activeVision.ptrPtCldUnexp,newPose,2,minPtsVis,viewer);
//     // dir = findDirection(activeVision.ptrPtCldObject,activeVision.ptrPtCldUnexp,newPose,2,minPtsVis);
//     std::cout << "Direction selected : " << dir << std::endl;
//
//     newPose = calcExplorationPose(newPose,dir,2);
//     singlePass(activeVision,newPose,false,true,2);
//     nSteps++;
//   }
//
//   activeVision.deleteObject(objID);
//   boost::this_thread::sleep(boost::posix_time::milliseconds(500));
// }
