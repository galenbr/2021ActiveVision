; Auto-generated. Do not edit!


(cl:in-package moveit_planner-srv)


;//! \htmlinclude MovePose-request.msg.html

(cl:defclass <MovePose-request> (roslisp-msg-protocol:ros-message)
  ((val
    :reader val
    :initarg :val
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (execute
    :reader execute
    :initarg :execute
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MovePose-request (<MovePose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MovePose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MovePose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moveit_planner-srv:<MovePose-request> is deprecated: use moveit_planner-srv:MovePose-request instead.")))

(cl:ensure-generic-function 'val-val :lambda-list '(m))
(cl:defmethod val-val ((m <MovePose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moveit_planner-srv:val-val is deprecated.  Use moveit_planner-srv:val instead.")
  (val m))

(cl:ensure-generic-function 'execute-val :lambda-list '(m))
(cl:defmethod execute-val ((m <MovePose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moveit_planner-srv:execute-val is deprecated.  Use moveit_planner-srv:execute instead.")
  (execute m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MovePose-request>) ostream)
  "Serializes a message object of type '<MovePose-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'val) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'execute) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MovePose-request>) istream)
  "Deserializes a message object of type '<MovePose-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'val) istream)
    (cl:setf (cl:slot-value msg 'execute) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MovePose-request>)))
  "Returns string type for a service object of type '<MovePose-request>"
  "moveit_planner/MovePoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MovePose-request)))
  "Returns string type for a service object of type 'MovePose-request"
  "moveit_planner/MovePoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MovePose-request>)))
  "Returns md5sum for a message object of type '<MovePose-request>"
  "a77c545a10fa137b7b4acf08012288bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MovePose-request)))
  "Returns md5sum for a message object of type 'MovePose-request"
  "a77c545a10fa137b7b4acf08012288bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MovePose-request>)))
  "Returns full string definition for message of type '<MovePose-request>"
  (cl:format cl:nil "geometry_msgs/Pose val~%bool execute~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MovePose-request)))
  "Returns full string definition for message of type 'MovePose-request"
  (cl:format cl:nil "geometry_msgs/Pose val~%bool execute~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MovePose-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'val))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MovePose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MovePose-request
    (cl:cons ':val (val msg))
    (cl:cons ':execute (execute msg))
))
;//! \htmlinclude MovePose-response.msg.html

(cl:defclass <MovePose-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MovePose-response (<MovePose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MovePose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MovePose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moveit_planner-srv:<MovePose-response> is deprecated: use moveit_planner-srv:MovePose-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MovePose-response>) ostream)
  "Serializes a message object of type '<MovePose-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MovePose-response>) istream)
  "Deserializes a message object of type '<MovePose-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MovePose-response>)))
  "Returns string type for a service object of type '<MovePose-response>"
  "moveit_planner/MovePoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MovePose-response)))
  "Returns string type for a service object of type 'MovePose-response"
  "moveit_planner/MovePoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MovePose-response>)))
  "Returns md5sum for a message object of type '<MovePose-response>"
  "a77c545a10fa137b7b4acf08012288bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MovePose-response)))
  "Returns md5sum for a message object of type 'MovePose-response"
  "a77c545a10fa137b7b4acf08012288bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MovePose-response>)))
  "Returns full string definition for message of type '<MovePose-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MovePose-response)))
  "Returns full string definition for message of type 'MovePose-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MovePose-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MovePose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MovePose-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MovePose)))
  'MovePose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MovePose)))
  'MovePose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MovePose)))
  "Returns string type for a service object of type '<MovePose>"
  "moveit_planner/MovePose")