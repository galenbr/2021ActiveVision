#ifndef ENVIRONMENT
#define ENVIRONMENT

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <array>
#include <string>
#include <fstream>
#include <chrono>
#include <random>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/master.h>
#include <tf/transform_datatypes.h>
#include <active_vision/graspSRV.h>
#include <active_vision/graspVisSRV.h>
#include <active_vision/toolDataHandling.h>

// OpenCV Specific Includes
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/generate.h>
#include <pcl/common/random.h>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <pcl/registration/warp_point_rigid.h>

//Gazebo specific includes
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/DeleteModel.h>

//Moveit
#include <moveit_planner/GetPose.h>
#include <moveit_planner/MoveCart.h>
#include <moveit_planner/MovePose.h>
// #include <moveit_planner/MoveJoint.h>
#include <moveit_planner/Inv.h>
#include <moveit_planner/SetVelocity.h>
#include <moveit_planner/AddCollision.h>
#include <moveit_planner/SetConstraints.h>
#include <moveit_planner/MoveNamedState.h>
#include <moveit_planner/SetJointWithTime.h>
#include <std_srvs/Empty.h>

// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <actionlib/client/simple_action_client.h>
// #include <actionlib/client/terminal_state.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <franka_pos_grasping_gazebo/GripPos.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <tf/transform_listener.h>

// Typedef for convinience
typedef pcl::PointCloud<pcl::PointXYZRGB> ptCldColor;
typedef pcl::PointCloud<pcl::Normal> ptCldNormal;

bool ROSCheck(std::string type, std::string name);

// Structure to store one grasp related data
struct graspPoint{
  double quality;
  double gripperWidth;
  pcl::PointXYZRGB p1;
  pcl::PointXYZRGB p2;
  std::vector<double> pose;    // Note: This is not the final gripper pose
  double addnlPitch;
};

// Structure to store state for rollback
struct stateConfig{
  ptCldColor env;                   // Environment point cloud
  ptCldColor unexp;                 // Unexplored point cloud
  std::vector<double> cameraPose;   // Camera Pose in Viewsphere
  std::string description;          // Description of the configuration
  std::vector<double> unexpMin;
  std::vector<double> unexpMax;
};

// Funstion to transpose a homogenous matrix
Eigen::Affine3f homoMatTranspose(const Eigen::Affine3f& tf);

// Get Rotation Part of a Affine3f
Eigen::Vector3f getEuler(const Eigen::Affine3f& tf);

// Get Translational Part of a Affine3f
Eigen::Vector3f getTranslation(const Eigen::Affine3f& tf);

Eigen::Affine3f calcTfFromNormal(pcl::Normal normal, pcl::PointXYZRGB point);

void calcTfFromNormal(pcl::Normal normal, pcl::PointXYZRGB point, Eigen::Matrix3f &rot, Eigen::Vector3f &trans);

void RGBtoHSV(float fR, float fG, float fB, float& fH, float& fS, float& fV);

bool checkFrankReach(ros::ServiceClient &IKClient, geometry_msgs::Pose &p);

// Class to store data of environment and its processing
class environment{
private:
  ros::Rate r{10};                      // ROS sleep rate
  ros::Publisher pubObjPose;            // Publisher : Camera/Objects pose
  ros::Subscriber subCameraPtCld;       // Subscriber : Camera pointcloud
  ros::ServiceClient gazeboSpawnModel;  // Service : Spawn Model
  ros::ServiceClient gazeboCheckModel;  // Service : Check Model
  ros::ServiceClient gazeboDeleteModel; // Service : Delete Model

  pcl::PassThrough<pcl::PointXYZRGB> pass;                  // Passthrough filter
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;               // VoxelGrid object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;               // Segmentation object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;            // Extracting object
  pcl::ConvexHull<pcl::PointXYZRGB> cvHull;                 // Convex hull object
  pcl::CropBox<pcl::PointXYZRGB> cpBox;                     // Crop box object
  pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;   // Prism object
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;  // Normal Estimation

  Eigen::MatrixXf projectionMat;   // Camera projection matrix
  Eigen::Affine3f tfCamOptGaz;     // Transform : Camera Optical Frame to Camera Gazebo frame
  Eigen::Affine3f tfGazWorld;      // Transform : Camera Gazebo Frame to Gazebo World frame

  int readFlag[3];                 // Flag used to read data from Camera only when needed
  bool addNoise;
  float depthNoise;                // Depth in %
  std::default_random_engine generator;

  std::string path;                // Path the active vision package

public:

  ros::ServiceClient graspClient;
  ros::ServiceClient graspVisClient;

  ros::ServiceClient getPoseClient;
  ros::ServiceClient poseClient;
  ros::ServiceClient IKClient;
  ros::ServiceClient cartMoveClient;
  ros::ServiceClient velScalingClient;
  // ros::ServiceClient jointSpaceClient;
  ros::ServiceClient gripperPosClient;
  ros::ServiceClient collisionClient;
  ros::ServiceClient setConstClient;
  ros::ServiceClient clearConstClient;
  ros::ServiceClient namedStateClient;
  ros::ServiceClient oneJointWithTimeClient;
  // moveit_planner::SetVelocity velscale;
  tf::TransformListener listener;

  // PtCld: Last recorded viewpoint
  ptCldColor::Ptr ptrPtCldLast{new ptCldColor};      ptCldColor::ConstPtr cPtrPtCldLast{ptrPtCldLast};
  ptCldColor::Ptr ptrPtCldLastFill{new ptCldColor};  ptCldColor::ConstPtr cPtrPtCldLastFill{ptrPtCldLastFill};

  // PtCld: Environment after fusing multiple view points, extracted table, object and its normal
  ptCldColor::Ptr ptrPtCldEnv{new ptCldColor};       ptCldColor::ConstPtr cPtrPtCldEnv{ptrPtCldEnv};
  ptCldColor::Ptr ptrPtCldTable{new ptCldColor};     ptCldColor::ConstPtr cPtrPtCldTable{ptrPtCldTable};
  ptCldColor::Ptr ptrPtCldObject{new ptCldColor};    ptCldColor::ConstPtr cPtrPtCldObject{ptrPtCldObject};

  // PtCld: Sorting the convex hull generated
  ptCldColor::Ptr ptrPtCldHull{new ptCldColor};      ptCldColor::ConstPtr cPtrPtCldHull{ptrPtCldHull};

  // PtCld: Unexplored point cloud
  ptCldColor::Ptr ptrPtCldUnexp{new ptCldColor};     ptCldColor::ConstPtr cPtrPtCldUnexp{ptrPtCldUnexp};

  // PtCld: Temporary variable
  ptCldColor::Ptr ptrPtCldTemp{new ptCldColor};      ptCldColor::ConstPtr cPtrPtCldTemp{ptrPtCldTemp};

  // Variables used in table and object extraction
  pcl::ModelCoefficients::Ptr tableCoeff{new pcl::ModelCoefficients()};
  pcl::PointIndices::Ptr tableIndices{new pcl::PointIndices()};
  pcl::PointIndices::Ptr objectIndices{new pcl::PointIndices()};

  std::vector<pcl::Vertices> hullVertices;          // Used in convex hull during collision check
  std::vector<double> lastCameraPoseCartesian;      // Last Camera pose where it was moved in cartesian co-ordiantes
  std::vector<double> lastCameraPoseViewsphere;     // Last Camera pose where it was moved in viewsphere co-ordinates
  std::vector<double> minUnexp, maxUnexp;           // Min and Max x,y,z of unexplored pointcloud generated
  pcl::PointXYZRGB minPtObj, maxPtObj;              // Min and Max x,y,z co-ordinates of the object
  pcl::PointXYZRGB minTable, maxTable;              // Min and Max x,y,z co-ordinates of the object
  Eigen::Vector4f cenTable,cenObject;

  std::map<int,objectInfo> objectDict;              // Dictionary to store object info
  double voxelGridSize;                             // Voxel Grid size for environment
  double voxelGridSizeUnexp;                        // Voxel Grid size for unexplored point cloud
  std::vector<double> tableCentre;                  // Co-ordinates of table centre
  int scale;                                        // Scale value for unexplored point cloud generation
  
  std::vector<stateConfig> configurations;          // Vector to store states for rollback
  float viewsphereRad;                              // Viewsphere Radius in m
  std::string simulationMode;                       // simulationMode
  float fingerZOffset;                              // Z axis offset between gripper hand and finger
  Eigen::Matrix4f ICP_Tf;

  active_vision::graspSRV graspMsg;
  active_vision::graspVisSRV graspVisMsg;
  ptCldColor::Ptr ptrPtCldGripper{new ptCldColor};   ptCldColor::ConstPtr cPtrPtCldGripper{ptrPtCldGripper};
  graspPoint graspData;
  int nGrasps;
  int graspID;

  environment(ros::NodeHandle *nh);

  // Function to reset the environment
  void reset();

  // Store the configuration
  int saveConfiguration(std::string name);

  // Rollback to a configuration
  void rollbackConfiguration(int index);

  // 1A: Callback function to point cloud subscriber
  void cbPtCld(const ptCldColor::ConstPtr& msg);

  // Function to set noise variables
  void setPtCldNoise(float num);

  // 2A: Spawning objects in gazebo on the table centre for a given pose option and yaw
  void spawnObject(int objectID, int choice, float yaw);
  void spawnBoxOnTable();

  // 2B: Function to move the object. Same args as spawnObject
  void moveObject(int objectID, int choice, float yaw);

  // 3: Deleting objects in gazebo
  void deleteObject(int objectID);

  // 5: Update gripper (Only for visualization)
  void updateGripper(int index ,int choice);

  // 6A: Function to move the camera. Args: Array of X,Y,Z,Roll,Pitch,Yaw
  bool moveCameraCartesian(std::vector<double> pose, bool execute = true);

  // 6B: Funtion to move the Camera in a viewsphere which has the table cente as its centre
  // R (Radius)
  // Theta (Polar Angle) -> 0 to 2*PI
  // Phi (Azhimuthal angle) -> 0 to PI/2
  bool moveCameraViewsphere(std::vector<double> pose, bool execute = true);

  // Function to move franka
  bool moveFranka(Eigen::Matrix4f tfMat, std::string mode ,bool isCamera ,bool execute, geometry_msgs::Pose &p);
  void moveGripper(double Grasp_Width, bool grasp = false);
  void moveFrankaHome(bool gripper = true);
  void addVisibilityConstraint();
  void addOrientationConstraint(Eigen::Affine3f tf);
  void clearAllConstraints();

  // 7: Function to read the camera data.
  void readCamera();

  // 8: Function to Fuse last data with existing data
  void fuseLastData();
  void ICPRegistration();

  // 9: Extracting the major plane (Table) and object
  void dataExtract();
  void dataExtractPlaneSeg();
  void dataColorCorrection();

  // 10: Generating unexplored point cloud
  void genUnexploredPtCld();

  // 11: Updating the unexplored point cloud
  void updateUnexploredPtCld();

  // 12: Finding normals and pairs of grasp points from object point cloud
  void graspsynthesis();

  // 16: Modify moveit collision elements
  void editMoveItCollisions(std::string object, std::string mode);

  // 17: Check is pregrasp is viable
  bool checkPreGrasp(graspPoint &graspData);

  // 17: Object grasping pipeline
  void graspObject(graspPoint &graspData);

};

// Function to do a single pass
std::vector<double> singlePass(environment &av, std::vector<double> cameraPose, bool firstTime, bool findGrasp, int mode = 1);

// Custom wrap function
namespace pcl{
  namespace registration{
    /** \brief @b WarpPointXY enables XY only transformation
      */
    template <typename PointSourceT, typename PointTargetT, typename Scalar = float>
    class WarpPointXY : public WarpPointRigid<PointSourceT, PointTargetT, Scalar>
    {
      public:
        typedef typename WarpPointXY<PointSourceT, PointTargetT, Scalar>::Matrix4 Matrix4;
        typedef typename WarpPointXY<PointSourceT, PointTargetT, Scalar>::VectorX VectorX;

        typedef boost::shared_ptr<WarpPointXY<PointSourceT, PointTargetT, Scalar> > Ptr;
        typedef boost::shared_ptr<const WarpPointXY<PointSourceT, PointTargetT, Scalar> > ConstPtr;

        /** \brief Constructor. */
        WarpPointXY () : WarpPointRigid<PointSourceT, PointTargetT, Scalar> (2) {}

        /** \brief Empty destructor */
        virtual ~WarpPointXY () {}

        /** \brief Set warp parameters.
          * \param[in] p warp parameters (tx ty)
          */
        virtual void
        setParam (const VectorX & p)        {
          assert(p.rows() == this->getDimension());
          Matrix4& trans = this->transform_matrix_;

          trans = Matrix4::Zero();
          trans(0, 0) = 1;
          trans(1, 1) = 1;
          trans(2, 2) = 1;
          trans(3, 3) = 1;

          trans(0,3) = p[0];
          trans(1,3) = p[1];
          trans(2,3) = 0;
        }
    };
  }
}

#endif
