;; Auto-generated. Do not edit!


(when (boundp 'pose_estimator::PoseEstimation)
  (if (not (find-package "POSE_ESTIMATOR"))
    (make-package "POSE_ESTIMATOR"))
  (shadow 'PoseEstimation (find-package "POSE_ESTIMATOR")))
(unless (find-package "POSE_ESTIMATOR::POSEESTIMATION")
  (make-package "POSE_ESTIMATOR::POSEESTIMATION"))
(unless (find-package "POSE_ESTIMATOR::POSEESTIMATIONREQUEST")
  (make-package "POSE_ESTIMATOR::POSEESTIMATIONREQUEST"))
(unless (find-package "POSE_ESTIMATOR::POSEESTIMATIONRESPONSE")
  (make-package "POSE_ESTIMATOR::POSEESTIMATIONRESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass pose_estimator::PoseEstimationRequest
  :super ros::object
  :slots (_data ))

(defmethod pose_estimator::PoseEstimationRequest
  (:init
   (&key
    ((:data __data) (instance sensor_msgs::PointCloud2 :init))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/PointCloud2 _data
    (send _data :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/PointCloud2 _data
       (send _data :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/PointCloud2 _data
     (send _data :deserialize buf ptr-) (incf ptr- (send _data :serialization-length))
   ;;
   self)
  )

(defclass pose_estimator::PoseEstimationResponse
  :super ros::object
  :slots (_detected_object_names _detected_object_poses ))

(defmethod pose_estimator::PoseEstimationResponse
  (:init
   (&key
    ((:detected_object_names __detected_object_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:detected_object_poses __detected_object_poses) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Pose :init) r)) r))
    )
   (send-super :init)
   (setq _detected_object_names __detected_object_names)
   (setq _detected_object_poses __detected_object_poses)
   self)
  (:detected_object_names
   (&optional __detected_object_names)
   (if __detected_object_names (setq _detected_object_names __detected_object_names)) _detected_object_names)
  (:detected_object_poses
   (&rest __detected_object_poses)
   (if (keywordp (car __detected_object_poses))
       (send* _detected_object_poses __detected_object_poses)
     (progn
       (if __detected_object_poses (setq _detected_object_poses (car __detected_object_poses)))
       _detected_object_poses)))
  (:serialization-length
   ()
   (+
    ;; string[] _detected_object_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _detected_object_names)) 4
    ;; geometry_msgs/Pose[] _detected_object_poses
    (apply #'+ (send-all _detected_object_poses :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _detected_object_names
     (write-long (length _detected_object_names) s)
     (dolist (elem _detected_object_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; geometry_msgs/Pose[] _detected_object_poses
     (write-long (length _detected_object_poses) s)
     (dolist (elem _detected_object_poses)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _detected_object_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detected_object_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _detected_object_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; geometry_msgs/Pose[] _detected_object_poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detected_object_poses (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _detected_object_poses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass pose_estimator::PoseEstimation
  :super ros::object
  :slots ())

(setf (get pose_estimator::PoseEstimation :md5sum-) "022a68c5fe014e6ff7c9f68c41199bb3")
(setf (get pose_estimator::PoseEstimation :datatype-) "pose_estimator/PoseEstimation")
(setf (get pose_estimator::PoseEstimation :request) pose_estimator::PoseEstimationRequest)
(setf (get pose_estimator::PoseEstimation :response) pose_estimator::PoseEstimationResponse)

(defmethod pose_estimator::PoseEstimationRequest
  (:response () (instance pose_estimator::PoseEstimationResponse :init)))

(setf (get pose_estimator::PoseEstimationRequest :md5sum-) "022a68c5fe014e6ff7c9f68c41199bb3")
(setf (get pose_estimator::PoseEstimationRequest :datatype-) "pose_estimator/PoseEstimationRequest")
(setf (get pose_estimator::PoseEstimationRequest :definition-)
      "sensor_msgs/PointCloud2 data

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
---
string[] detected_object_names
geometry_msgs/Pose[] detected_object_poses

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get pose_estimator::PoseEstimationResponse :md5sum-) "022a68c5fe014e6ff7c9f68c41199bb3")
(setf (get pose_estimator::PoseEstimationResponse :datatype-) "pose_estimator/PoseEstimationResponse")
(setf (get pose_estimator::PoseEstimationResponse :definition-)
      "sensor_msgs/PointCloud2 data

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
---
string[] detected_object_names
geometry_msgs/Pose[] detected_object_poses

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :pose_estimator/PoseEstimation "022a68c5fe014e6ff7c9f68c41199bb3")


