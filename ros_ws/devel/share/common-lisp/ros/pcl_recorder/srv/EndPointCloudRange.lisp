; Auto-generated. Do not edit!


(cl:in-package pcl_recorder-srv)


;//! \htmlinclude EndPointCloudRange-request.msg.html

(cl:defclass <EndPointCloudRange-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass EndPointCloudRange-request (<EndPointCloudRange-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EndPointCloudRange-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EndPointCloudRange-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pcl_recorder-srv:<EndPointCloudRange-request> is deprecated: use pcl_recorder-srv:EndPointCloudRange-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EndPointCloudRange-request>) ostream)
  "Serializes a message object of type '<EndPointCloudRange-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EndPointCloudRange-request>) istream)
  "Deserializes a message object of type '<EndPointCloudRange-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EndPointCloudRange-request>)))
  "Returns string type for a service object of type '<EndPointCloudRange-request>"
  "pcl_recorder/EndPointCloudRangeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EndPointCloudRange-request)))
  "Returns string type for a service object of type 'EndPointCloudRange-request"
  "pcl_recorder/EndPointCloudRangeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EndPointCloudRange-request>)))
  "Returns md5sum for a message object of type '<EndPointCloudRange-request>"
  "36ef54ba68cfe88c3625117f7de48f46")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EndPointCloudRange-request)))
  "Returns md5sum for a message object of type 'EndPointCloudRange-request"
  "36ef54ba68cfe88c3625117f7de48f46")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EndPointCloudRange-request>)))
  "Returns full string definition for message of type '<EndPointCloudRange-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EndPointCloudRange-request)))
  "Returns full string definition for message of type 'EndPointCloudRange-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EndPointCloudRange-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EndPointCloudRange-request>))
  "Converts a ROS message object to a list"
  (cl:list 'EndPointCloudRange-request
))
;//! \htmlinclude EndPointCloudRange-response.msg.html

(cl:defclass <EndPointCloudRange-response> (roslisp-msg-protocol:ros-message)
  ((mergedCloud
    :reader mergedCloud
    :initarg :mergedCloud
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass EndPointCloudRange-response (<EndPointCloudRange-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EndPointCloudRange-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EndPointCloudRange-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pcl_recorder-srv:<EndPointCloudRange-response> is deprecated: use pcl_recorder-srv:EndPointCloudRange-response instead.")))

(cl:ensure-generic-function 'mergedCloud-val :lambda-list '(m))
(cl:defmethod mergedCloud-val ((m <EndPointCloudRange-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pcl_recorder-srv:mergedCloud-val is deprecated.  Use pcl_recorder-srv:mergedCloud instead.")
  (mergedCloud m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EndPointCloudRange-response>) ostream)
  "Serializes a message object of type '<EndPointCloudRange-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mergedCloud) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EndPointCloudRange-response>) istream)
  "Deserializes a message object of type '<EndPointCloudRange-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mergedCloud) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EndPointCloudRange-response>)))
  "Returns string type for a service object of type '<EndPointCloudRange-response>"
  "pcl_recorder/EndPointCloudRangeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EndPointCloudRange-response)))
  "Returns string type for a service object of type 'EndPointCloudRange-response"
  "pcl_recorder/EndPointCloudRangeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EndPointCloudRange-response>)))
  "Returns md5sum for a message object of type '<EndPointCloudRange-response>"
  "36ef54ba68cfe88c3625117f7de48f46")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EndPointCloudRange-response)))
  "Returns md5sum for a message object of type 'EndPointCloudRange-response"
  "36ef54ba68cfe88c3625117f7de48f46")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EndPointCloudRange-response>)))
  "Returns full string definition for message of type '<EndPointCloudRange-response>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 mergedCloud~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EndPointCloudRange-response)))
  "Returns full string definition for message of type 'EndPointCloudRange-response"
  (cl:format cl:nil "sensor_msgs/PointCloud2 mergedCloud~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EndPointCloudRange-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mergedCloud))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EndPointCloudRange-response>))
  "Converts a ROS message object to a list"
  (cl:list 'EndPointCloudRange-response
    (cl:cons ':mergedCloud (mergedCloud msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'EndPointCloudRange)))
  'EndPointCloudRange-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'EndPointCloudRange)))
  'EndPointCloudRange-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EndPointCloudRange)))
  "Returns string type for a service object of type '<EndPointCloudRange>"
  "pcl_recorder/EndPointCloudRange")