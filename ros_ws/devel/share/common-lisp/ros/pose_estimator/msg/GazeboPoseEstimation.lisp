; Auto-generated. Do not edit!


(cl:in-package pose_estimator-msg)


;//! \htmlinclude GazeboPoseEstimation.msg.html

(cl:defclass <GazeboPoseEstimation> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (model_names
    :reader model_names
    :initarg :model_names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (model_poses
    :reader model_poses
    :initarg :model_poses
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose))))
)

(cl:defclass GazeboPoseEstimation (<GazeboPoseEstimation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GazeboPoseEstimation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GazeboPoseEstimation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pose_estimator-msg:<GazeboPoseEstimation> is deprecated: use pose_estimator-msg:GazeboPoseEstimation instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GazeboPoseEstimation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_estimator-msg:header-val is deprecated.  Use pose_estimator-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'model_names-val :lambda-list '(m))
(cl:defmethod model_names-val ((m <GazeboPoseEstimation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_estimator-msg:model_names-val is deprecated.  Use pose_estimator-msg:model_names instead.")
  (model_names m))

(cl:ensure-generic-function 'model_poses-val :lambda-list '(m))
(cl:defmethod model_poses-val ((m <GazeboPoseEstimation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_estimator-msg:model_poses-val is deprecated.  Use pose_estimator-msg:model_poses instead.")
  (model_poses m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GazeboPoseEstimation>) ostream)
  "Serializes a message object of type '<GazeboPoseEstimation>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'model_names))))
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
   (cl:slot-value msg 'model_names))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'model_poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'model_poses))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GazeboPoseEstimation>) istream)
  "Deserializes a message object of type '<GazeboPoseEstimation>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'model_names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'model_names)))
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
  (cl:setf (cl:slot-value msg 'model_poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'model_poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GazeboPoseEstimation>)))
  "Returns string type for a message object of type '<GazeboPoseEstimation>"
  "pose_estimator/GazeboPoseEstimation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GazeboPoseEstimation)))
  "Returns string type for a message object of type 'GazeboPoseEstimation"
  "pose_estimator/GazeboPoseEstimation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GazeboPoseEstimation>)))
  "Returns md5sum for a message object of type '<GazeboPoseEstimation>"
  "3bbf8867409277371ff3ab48c9274b56")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GazeboPoseEstimation)))
  "Returns md5sum for a message object of type 'GazeboPoseEstimation"
  "3bbf8867409277371ff3ab48c9274b56")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GazeboPoseEstimation>)))
  "Returns full string definition for message of type '<GazeboPoseEstimation>"
  (cl:format cl:nil "std_msgs/Header header~%string[] model_names~%geometry_msgs/Pose[] model_poses~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GazeboPoseEstimation)))
  "Returns full string definition for message of type 'GazeboPoseEstimation"
  (cl:format cl:nil "std_msgs/Header header~%string[] model_names~%geometry_msgs/Pose[] model_poses~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GazeboPoseEstimation>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'model_names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'model_poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GazeboPoseEstimation>))
  "Converts a ROS message object to a list"
  (cl:list 'GazeboPoseEstimation
    (cl:cons ':header (header msg))
    (cl:cons ':model_names (model_names msg))
    (cl:cons ':model_poses (model_poses msg))
))
