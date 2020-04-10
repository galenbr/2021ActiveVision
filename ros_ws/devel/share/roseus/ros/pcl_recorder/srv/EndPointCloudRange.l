;; Auto-generated. Do not edit!


(when (boundp 'pcl_recorder::EndPointCloudRange)
  (if (not (find-package "PCL_RECORDER"))
    (make-package "PCL_RECORDER"))
  (shadow 'EndPointCloudRange (find-package "PCL_RECORDER")))
(unless (find-package "PCL_RECORDER::ENDPOINTCLOUDRANGE")
  (make-package "PCL_RECORDER::ENDPOINTCLOUDRANGE"))
(unless (find-package "PCL_RECORDER::ENDPOINTCLOUDRANGEREQUEST")
  (make-package "PCL_RECORDER::ENDPOINTCLOUDRANGEREQUEST"))
(unless (find-package "PCL_RECORDER::ENDPOINTCLOUDRANGERESPONSE")
  (make-package "PCL_RECORDER::ENDPOINTCLOUDRANGERESPONSE"))

(in-package "ROS")



(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass pcl_recorder::EndPointCloudRangeRequest
  :super ros::object
  :slots ())

(defmethod pcl_recorder::EndPointCloudRangeRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass pcl_recorder::EndPointCloudRangeResponse
  :super ros::object
  :slots (_mergedCloud ))

(defmethod pcl_recorder::EndPointCloudRangeResponse
  (:init
   (&key
    ((:mergedCloud __mergedCloud) (instance sensor_msgs::PointCloud2 :init))
    )
   (send-super :init)
   (setq _mergedCloud __mergedCloud)
   self)
  (:mergedCloud
   (&rest __mergedCloud)
   (if (keywordp (car __mergedCloud))
       (send* _mergedCloud __mergedCloud)
     (progn
       (if __mergedCloud (setq _mergedCloud (car __mergedCloud)))
       _mergedCloud)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/PointCloud2 _mergedCloud
    (send _mergedCloud :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/PointCloud2 _mergedCloud
       (send _mergedCloud :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/PointCloud2 _mergedCloud
     (send _mergedCloud :deserialize buf ptr-) (incf ptr- (send _mergedCloud :serialization-length))
   ;;
   self)
  )

(defclass pcl_recorder::EndPointCloudRange
  :super ros::object
  :slots ())

(setf (get pcl_recorder::EndPointCloudRange :md5sum-) "36ef54ba68cfe88c3625117f7de48f46")
(setf (get pcl_recorder::EndPointCloudRange :datatype-) "pcl_recorder/EndPointCloudRange")
(setf (get pcl_recorder::EndPointCloudRange :request) pcl_recorder::EndPointCloudRangeRequest)
(setf (get pcl_recorder::EndPointCloudRange :response) pcl_recorder::EndPointCloudRangeResponse)

(defmethod pcl_recorder::EndPointCloudRangeRequest
  (:response () (instance pcl_recorder::EndPointCloudRangeResponse :init)))

(setf (get pcl_recorder::EndPointCloudRangeRequest :md5sum-) "36ef54ba68cfe88c3625117f7de48f46")
(setf (get pcl_recorder::EndPointCloudRangeRequest :datatype-) "pcl_recorder/EndPointCloudRangeRequest")
(setf (get pcl_recorder::EndPointCloudRangeRequest :definition-)
      "---
sensor_msgs/PointCloud2 mergedCloud

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
")

(setf (get pcl_recorder::EndPointCloudRangeResponse :md5sum-) "36ef54ba68cfe88c3625117f7de48f46")
(setf (get pcl_recorder::EndPointCloudRangeResponse :datatype-) "pcl_recorder/EndPointCloudRangeResponse")
(setf (get pcl_recorder::EndPointCloudRangeResponse :definition-)
      "---
sensor_msgs/PointCloud2 mergedCloud

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
")



(provide :pcl_recorder/EndPointCloudRange "36ef54ba68cfe88c3625117f7de48f46")


