; Auto-generated. Do not edit!


(cl:in-package moveit_planner-srv)


;//! \htmlinclude MoveCart-request.msg.html

(cl:defclass <MoveCart-request> (roslisp-msg-protocol:ros-message)
  ((val
    :reader val
    :initarg :val
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose)))
   (execute
    :reader execute
    :initarg :execute
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MoveCart-request (<MoveCart-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveCart-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveCart-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moveit_planner-srv:<MoveCart-request> is deprecated: use moveit_planner-srv:MoveCart-request instead.")))

(cl:ensure-generic-function 'val-val :lambda-list '(m))
(cl:defmethod val-val ((m <MoveCart-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moveit_planner-srv:val-val is deprecated.  Use moveit_planner-srv:val instead.")
  (val m))

(cl:ensure-generic-function 'execute-val :lambda-list '(m))
(cl:defmethod execute-val ((m <MoveCart-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moveit_planner-srv:execute-val is deprecated.  Use moveit_planner-srv:execute instead.")
  (execute m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveCart-request>) ostream)
  "Serializes a message object of type '<MoveCart-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'val))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'val))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'execute) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveCart-request>) istream)
  "Deserializes a message object of type '<MoveCart-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'val) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'val)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:slot-value msg 'execute) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveCart-request>)))
  "Returns string type for a service object of type '<MoveCart-request>"
  "moveit_planner/MoveCartRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveCart-request)))
  "Returns string type for a service object of type 'MoveCart-request"
  "moveit_planner/MoveCartRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveCart-request>)))
  "Returns md5sum for a message object of type '<MoveCart-request>"
  "a77c545a10fa137b7b4acf08012288bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveCart-request)))
  "Returns md5sum for a message object of type 'MoveCart-request"
  "a77c545a10fa137b7b4acf08012288bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveCart-request>)))
  "Returns full string definition for message of type '<MoveCart-request>"
  (cl:format cl:nil "geometry_msgs/Pose[] val~%bool execute~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveCart-request)))
  "Returns full string definition for message of type 'MoveCart-request"
  (cl:format cl:nil "geometry_msgs/Pose[] val~%bool execute~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveCart-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'val) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveCart-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveCart-request
    (cl:cons ':val (val msg))
    (cl:cons ':execute (execute msg))
))
;//! \htmlinclude MoveCart-response.msg.html

(cl:defclass <MoveCart-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MoveCart-response (<MoveCart-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveCart-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveCart-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moveit_planner-srv:<MoveCart-response> is deprecated: use moveit_planner-srv:MoveCart-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveCart-response>) ostream)
  "Serializes a message object of type '<MoveCart-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveCart-response>) istream)
  "Deserializes a message object of type '<MoveCart-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveCart-response>)))
  "Returns string type for a service object of type '<MoveCart-response>"
  "moveit_planner/MoveCartResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveCart-response)))
  "Returns string type for a service object of type 'MoveCart-response"
  "moveit_planner/MoveCartResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveCart-response>)))
  "Returns md5sum for a message object of type '<MoveCart-response>"
  "a77c545a10fa137b7b4acf08012288bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveCart-response)))
  "Returns md5sum for a message object of type 'MoveCart-response"
  "a77c545a10fa137b7b4acf08012288bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveCart-response>)))
  "Returns full string definition for message of type '<MoveCart-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveCart-response)))
  "Returns full string definition for message of type 'MoveCart-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveCart-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveCart-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveCart-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveCart)))
  'MoveCart-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveCart)))
  'MoveCart-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveCart)))
  "Returns string type for a service object of type '<MoveCart>"
  "moveit_planner/MoveCart")