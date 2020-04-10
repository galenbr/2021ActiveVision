; Auto-generated. Do not edit!


(cl:in-package moveit_planner-srv)


;//! \htmlinclude MoveAway-request.msg.html

(cl:defclass <MoveAway-request> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0)
   (execute
    :reader execute
    :initarg :execute
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MoveAway-request (<MoveAway-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveAway-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveAway-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moveit_planner-srv:<MoveAway-request> is deprecated: use moveit_planner-srv:MoveAway-request instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <MoveAway-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moveit_planner-srv:pose-val is deprecated.  Use moveit_planner-srv:pose instead.")
  (pose m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <MoveAway-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moveit_planner-srv:distance-val is deprecated.  Use moveit_planner-srv:distance instead.")
  (distance m))

(cl:ensure-generic-function 'execute-val :lambda-list '(m))
(cl:defmethod execute-val ((m <MoveAway-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moveit_planner-srv:execute-val is deprecated.  Use moveit_planner-srv:execute instead.")
  (execute m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveAway-request>) ostream)
  "Serializes a message object of type '<MoveAway-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'execute) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveAway-request>) istream)
  "Deserializes a message object of type '<MoveAway-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'execute) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveAway-request>)))
  "Returns string type for a service object of type '<MoveAway-request>"
  "moveit_planner/MoveAwayRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveAway-request)))
  "Returns string type for a service object of type 'MoveAway-request"
  "moveit_planner/MoveAwayRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveAway-request>)))
  "Returns md5sum for a message object of type '<MoveAway-request>"
  "b0b7b3e5a88a68f810ee5cdc3adb57e7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveAway-request)))
  "Returns md5sum for a message object of type 'MoveAway-request"
  "b0b7b3e5a88a68f810ee5cdc3adb57e7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveAway-request>)))
  "Returns full string definition for message of type '<MoveAway-request>"
  (cl:format cl:nil "geometry_msgs/Pose pose~%float64 distance~%bool execute~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveAway-request)))
  "Returns full string definition for message of type 'MoveAway-request"
  (cl:format cl:nil "geometry_msgs/Pose pose~%float64 distance~%bool execute~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveAway-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveAway-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveAway-request
    (cl:cons ':pose (pose msg))
    (cl:cons ':distance (distance msg))
    (cl:cons ':execute (execute msg))
))
;//! \htmlinclude MoveAway-response.msg.html

(cl:defclass <MoveAway-response> (roslisp-msg-protocol:ros-message)
  ((awayPose
    :reader awayPose
    :initarg :awayPose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass MoveAway-response (<MoveAway-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveAway-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveAway-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moveit_planner-srv:<MoveAway-response> is deprecated: use moveit_planner-srv:MoveAway-response instead.")))

(cl:ensure-generic-function 'awayPose-val :lambda-list '(m))
(cl:defmethod awayPose-val ((m <MoveAway-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moveit_planner-srv:awayPose-val is deprecated.  Use moveit_planner-srv:awayPose instead.")
  (awayPose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveAway-response>) ostream)
  "Serializes a message object of type '<MoveAway-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'awayPose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveAway-response>) istream)
  "Deserializes a message object of type '<MoveAway-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'awayPose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveAway-response>)))
  "Returns string type for a service object of type '<MoveAway-response>"
  "moveit_planner/MoveAwayResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveAway-response)))
  "Returns string type for a service object of type 'MoveAway-response"
  "moveit_planner/MoveAwayResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveAway-response>)))
  "Returns md5sum for a message object of type '<MoveAway-response>"
  "b0b7b3e5a88a68f810ee5cdc3adb57e7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveAway-response)))
  "Returns md5sum for a message object of type 'MoveAway-response"
  "b0b7b3e5a88a68f810ee5cdc3adb57e7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveAway-response>)))
  "Returns full string definition for message of type '<MoveAway-response>"
  (cl:format cl:nil "geometry_msgs/Pose awayPose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveAway-response)))
  "Returns full string definition for message of type 'MoveAway-response"
  (cl:format cl:nil "geometry_msgs/Pose awayPose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveAway-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'awayPose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveAway-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveAway-response
    (cl:cons ':awayPose (awayPose msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveAway)))
  'MoveAway-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveAway)))
  'MoveAway-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveAway)))
  "Returns string type for a service object of type '<MoveAway>"
  "moveit_planner/MoveAway")