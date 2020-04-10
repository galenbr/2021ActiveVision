; Auto-generated. Do not edit!


(cl:in-package pose_estimator-srv)


;//! \htmlinclude PoseEstimation-request.msg.html

(cl:defclass <PoseEstimation-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass PoseEstimation-request (<PoseEstimation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PoseEstimation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PoseEstimation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pose_estimator-srv:<PoseEstimation-request> is deprecated: use pose_estimator-srv:PoseEstimation-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <PoseEstimation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_estimator-srv:data-val is deprecated.  Use pose_estimator-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PoseEstimation-request>) ostream)
  "Serializes a message object of type '<PoseEstimation-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PoseEstimation-request>) istream)
  "Deserializes a message object of type '<PoseEstimation-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PoseEstimation-request>)))
  "Returns string type for a service object of type '<PoseEstimation-request>"
  "pose_estimator/PoseEstimationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PoseEstimation-request)))
  "Returns string type for a service object of type 'PoseEstimation-request"
  "pose_estimator/PoseEstimationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PoseEstimation-request>)))
  "Returns md5sum for a message object of type '<PoseEstimation-request>"
  "022a68c5fe014e6ff7c9f68c41199bb3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PoseEstimation-request)))
  "Returns md5sum for a message object of type 'PoseEstimation-request"
  "022a68c5fe014e6ff7c9f68c41199bb3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PoseEstimation-request>)))
  "Returns full string definition for message of type '<PoseEstimation-request>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 data~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PoseEstimation-request)))
  "Returns full string definition for message of type 'PoseEstimation-request"
  (cl:format cl:nil "sensor_msgs/PointCloud2 data~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PoseEstimation-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PoseEstimation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PoseEstimation-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude PoseEstimation-response.msg.html

(cl:defclass <PoseEstimation-response> (roslisp-msg-protocol:ros-message)
  ((detected_object_names
    :reader detected_object_names
    :initarg :detected_object_names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (detected_object_poses
    :reader detected_object_poses
    :initarg :detected_object_poses
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose))))
)

(cl:defclass PoseEstimation-response (<PoseEstimation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PoseEstimation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PoseEstimation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pose_estimator-srv:<PoseEstimation-response> is deprecated: use pose_estimator-srv:PoseEstimation-response instead.")))

(cl:ensure-generic-function 'detected_object_names-val :lambda-list '(m))
(cl:defmethod detected_object_names-val ((m <PoseEstimation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_estimator-srv:detected_object_names-val is deprecated.  Use pose_estimator-srv:detected_object_names instead.")
  (detected_object_names m))

(cl:ensure-generic-function 'detected_object_poses-val :lambda-list '(m))
(cl:defmethod detected_object_poses-val ((m <PoseEstimation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_estimator-srv:detected_object_poses-val is deprecated.  Use pose_estimator-srv:detected_object_poses instead.")
  (detected_object_poses m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PoseEstimation-response>) ostream)
  "Serializes a message object of type '<PoseEstimation-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detected_object_names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'detected_object_names))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detected_object_poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detected_object_poses))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PoseEstimation-response>) istream)
  "Deserializes a message object of type '<PoseEstimation-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detected_object_names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detected_object_names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detected_object_poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detected_object_poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PoseEstimation-response>)))
  "Returns string type for a service object of type '<PoseEstimation-response>"
  "pose_estimator/PoseEstimationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PoseEstimation-response)))
  "Returns string type for a service object of type 'PoseEstimation-response"
  "pose_estimator/PoseEstimationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PoseEstimation-response>)))
  "Returns md5sum for a message object of type '<PoseEstimation-response>"
  "022a68c5fe014e6ff7c9f68c41199bb3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PoseEstimation-response)))
  "Returns md5sum for a message object of type 'PoseEstimation-response"
  "022a68c5fe014e6ff7c9f68c41199bb3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PoseEstimation-response>)))
  "Returns full string definition for message of type '<PoseEstimation-response>"
  (cl:format cl:nil "string[] detected_object_names~%geometry_msgs/Pose[] detected_object_poses~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PoseEstimation-response)))
  "Returns full string definition for message of type 'PoseEstimation-response"
  (cl:format cl:nil "string[] detected_object_names~%geometry_msgs/Pose[] detected_object_poses~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PoseEstimation-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detected_object_names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detected_object_poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PoseEstimation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PoseEstimation-response
    (cl:cons ':detected_object_names (detected_object_names msg))
    (cl:cons ':detected_object_poses (detected_object_poses msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PoseEstimation)))
  'PoseEstimation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PoseEstimation)))
  'PoseEstimation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PoseEstimation)))
  "Returns string type for a service object of type '<PoseEstimation>"
  "pose_estimator/PoseEstimation")