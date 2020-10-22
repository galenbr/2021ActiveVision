#ifndef TESTING_MODEL
#define TESTING_MODEL

#include "active_vision/helperFunctions.h"

void rbgVis(ptCldVis::Ptr viewer, ptCldColor::ConstPtr cloud, std::string name,int vp);

void rbgNormalVis(ptCldVis::Ptr viewer, ptCldColor::ConstPtr cloud, ptCldNormal::ConstPtr normal, std::string name,int vp);

// Class to store data of environment and its processing
class environment{
private:
  ros::Rate r;                          // ROS sleep rate
  ros::Publisher pubKinectPose;         // Publisher : Kinect pose
  ros::Subscriber subKinectPtCld;       // Subscriber : Kinect pointcloud
  // NOT USED (JUST FOR REFERENCE)
  /*ros::Subscriber subKinectRGB;       // Subscriber : Kinect RGB
  ros::Subscriber subKinectDepth;       // Subscriber : Kinect DepthMap */
  ros::ServiceClient gazeboSpawnModel;  // Service : Spawn Model
  ros::ServiceClient gazeboDeleteModel; // Service : Delete Model

  pcl::PassThrough<pcl::PointXYZRGB> pass;         // Passthrough filter
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;      // VoxelGrid object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;      // Segmentation object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;   // Extracting object
  pcl::ConvexHull<pcl::PointXYZRGB> cvHull;        // Convex hull object
  pcl::CropHull<pcl::PointXYZRGB> cpHull;          // Crop hull object
  pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;   // Prism object
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;  // Normal Estimation

  Eigen::MatrixXf projectionMat;   // Camera projection matrix

  int readFlag[3];                 // Flag used to read data from kinect only when needed
  float fingerZOffset;             // Z axis offset between gripper hand and finger

  std::string path;                                     // Path the active vision package

public:
  // NOT USED (JUST FOR REFERENCE)
  /* cv_bridge::CvImageConstPtr ptrRgbLast{new cv_bridge::CvImage};    // RGB image from camera
  cv_bridge::CvImageConstPtr ptrDepthLast{new cv_bridge::CvImage};  // Depth map from camera */

  Eigen::Affine3f tfKinOptGaz;     // Transform : Kinect Optical Frame to Kinect Gazebo frame
  Eigen::Affine3f tfGazWorld;      // Transform : Kinect Gazebo Frame to Gazebo World frame
  Eigen::Affine3f tfGripper;       // Transform : For gripper based on grasp points found

  // PtCld: Last recorded viewpoint
  ptCldColor::Ptr ptrPtCldLast{new ptCldColor};      ptCldColor::ConstPtr cPtrPtCldLast{ptrPtCldLast};

  // PtCld: Environment after fusing multiple view points, extracted table, object and its normal
  ptCldColor::Ptr ptrPtCldEnv{new ptCldColor};       ptCldColor::ConstPtr cPtrPtCldEnv{ptrPtCldEnv};
  ptCldColor::Ptr ptrPtCldTable{new ptCldColor};     ptCldColor::ConstPtr cPtrPtCldTable{ptrPtCldTable};
  ptCldColor::Ptr ptrPtCldObject{new ptCldColor};    ptCldColor::ConstPtr cPtrPtCldObject{ptrPtCldObject};
  ptCldNormal::Ptr ptrObjNormal{new ptCldNormal};    ptCldNormal::ConstPtr cPtrObjNormal{ptrObjNormal};

  // PtCld: Sorting the convex hull generated
  ptCldColor::Ptr ptrPtCldHull{new ptCldColor};      ptCldColor::ConstPtr cPtrPtCldHull{ptrPtCldHull};

  // PtCld: Unexplored point cloud, point clould used for collision check (unexplored + table)
  ptCldColor::Ptr ptrPtCldUnexp{new ptCldColor};     ptCldColor::ConstPtr cPtrPtCldUnexp{ptrPtCldUnexp};
  ptCldColor::Ptr ptrPtCldCollCheck{new ptCldColor}; ptCldColor::ConstPtr cPtrPtCldCollCheck{ptrPtCldCollCheck};

  // PtCld: Gripper related
  ptCldColor::Ptr ptrPtCldGrpHnd{new ptCldColor};    ptCldColor::ConstPtr cPtrPtCldGrpHnd{ptrPtCldGrpHnd};
  ptCldColor::Ptr ptrPtCldGrpRfgr{new ptCldColor};   ptCldColor::ConstPtr cPtrPtCldGrpRfgr{ptrPtCldGrpRfgr};
  ptCldColor::Ptr ptrPtCldGrpLfgr{new ptCldColor};   ptCldColor::ConstPtr cPtrPtCldGrpLfgr{ptrPtCldGrpLfgr};
  ptCldColor::Ptr ptrPtCldGripper{new ptCldColor};   ptCldColor::ConstPtr cPtrPtCldGripper{ptrPtCldGripper};

  // PtCld: Points colliding with gripper
  ptCldColor::Ptr ptrPtCldCollided{new ptCldColor};  ptCldColor::ConstPtr cPtrPtCldCollided{ptrPtCldCollided};

  // PtCld: Temporary variable
  ptCldColor::Ptr ptrPtCldTemp{new ptCldColor};      ptCldColor::ConstPtr cPtrPtCldTemp{ptrPtCldTemp};

  // Variables used in table and object extraction
  pcl::ModelCoefficients::Ptr tableCoeff{new pcl::ModelCoefficients()};
  pcl::PointIndices::Ptr tableIndices{new pcl::PointIndices()};
  pcl::PointIndices::Ptr objectIndices{new pcl::PointIndices()};

  std::vector<pcl::Vertices> hullVertices;          // Used in convex hull during collision check
  std::vector<double> lastKinectPoseCartesian;      // Last Kinect pose where it was moved in cartesian co-ordiantes
  std::vector<double> lastKinectPoseViewsphere;     // Last Kinect pose where it was moved in viewsphere co-ordinates
  std::vector<double> minUnexp;                     // Min x,y,z of unexplored pointcloud generated
  std::vector<double> maxUnexp;                     // Max x,y,z of unexplored pointcloud generated
  std::vector<graspPoint> graspsPossible;           // List of possible grasps
  pcl::PointXYZRGB minPtObj, maxPtObj;              // Min and Max x,y,z co-ordinates of the object

  std::vector<std::vector<std::string>> objectDict; // List of objects which can be spawned
  double voxelGridSize;                             // Voxel Grid size for environment
  double voxelGridSizeUnexp;                        // Voxel Grid size for unexplored point cloud
  std::vector<double> tableCentre;                  // Co-ordinates of table centre
  int scale;                                        // Scale value for unexplored point cloud generation
  double maxGripperWidth;                           // Gripper max width (Actual is 8 cm)
  double minGraspQuality;                           // Min grasp quality threshold
  int selectedGrasp;                                // Index of the selected grasp
  int gridDim;                                      // Grid dimension for state vector
  std::vector<float> stateVec;                      // State Vector

  environment(ros::NodeHandle *nh);

  // 1A: Callback function to point cloud subscriber
  void cbPtCld (const ptCldColor::ConstPtr& msg);

  // 2: Spawning objects in gazebo on the table centre for a given RPY
  void spawnObject(int objectID, float R, float P, float Y);

  // 3: Deleting objects in gazebo
  void deleteObject(int objectID);

  // 4: Load Gripper Hand and Finger file
  void loadGripper();

  // 5: Update gripper
  void updateGripper(int index ,int choice);
  void updateGripper(int index ,int choice, std::vector<graspPoint> *graspsP);

  // 6A: Function to move the kinect. Args: Array of X,Y,Z,Roll,Pitch,Yaw
  void moveKinectCartesian(std::vector<double> pose);

  // 6B: Funtion to move the Kinect in a viewsphere which has the table cente as its centre
  void moveKinectViewsphere(std::vector<double> pose);

  // 7: Function to read the kinect data.
  void readKinect();

  // 8: Function to Fuse last data with existing data
  void fuseLastData(void);
  void fuseLastData(ptCldColor::Ptr retEnv);

  // 9: Extracting the major plane (Table) and object
  void dataExtract(void);
  void dataExtract(ptCldColor::ConstPtr cEnv, ptCldColor::Ptr target);

  // 10: Generating unexplored point cloud
  void genUnexploredPtCld();

  // 11: Updating the unexplored point cloud
  void updateUnexploredPtCld();

  // 12: Finding pairs of grasp points from object point cloud
  void graspSynthesis(void);
  void graspSynthesis(ptCldColor::Ptr object);

  // 13: Given a grasp point pair find the gripper orientation
  void findGripperPose(int index);

  // 14: Collision check for gripper and unexplored point cloud
  void collisionCheck();
};

#endif
