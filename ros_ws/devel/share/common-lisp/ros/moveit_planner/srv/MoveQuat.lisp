; Auto-generated. Do not edit!


(cl:in-package moveit_planner-srv)


;//! \htmlinclude MoveQuat-request.msg.html

(cl:defclass <MoveQuat-request> (roslisp-msg-protocol:ros-message)
  ((val
    :reader val
    :initarg :val
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (execute
    :reader execute
    :initarg :execute
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MoveQuat-request (<MoveQuat-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveQuat-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveQuat-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moveit_planner-srv:<MoveQuat-request> is deprecated: use moveit_planner-srv:MoveQuat-request instead.")))

(cl:ensure-generic-function 'val-val :lambda-list '(m))
(cl:defmethod val-val ((m <MoveQuat-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moveit_planner-srv:val-val is deprecated.  Use moveit_planner-srv:val instead.")
  (val m))

(cl:ensure-generic-function 'execute-val :lambda-list '(m))
(cl:defmethod execute-val ((m <MoveQuat-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moveit_planner-srv:execute-val is deprecated.  Use moveit_planner-srv:execute instead.")
  (execute m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveQuat-request>) ostream)
  "Serializes a message object of type '<MoveQuat-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'val) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'execute) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveQuat-request>) istream)
  "Deserializes a message object of type '<MoveQuat-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'val) istream)
    (cl:setf (cl:slot-value msg 'execute) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveQuat-request>)))
  "Returns string type for a service object of type '<MoveQuat-request>"
  "moveit_planner/MoveQuatRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveQuat-request)))
  "Returns string type for a service object of type 'MoveQuat-request"
  "moveit_planner/MoveQuatRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveQuat-request>)))
  "Returns md5sum for a message object of type '<MoveQuat-request>"
  "38b5b4ab2cb69ba284d4cfd4d2765acc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveQuat-request)))
  "Returns md5sum for a message object of type 'MoveQuat-request"
  "38b5b4ab2cb69ba284d4cfd4d2765acc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveQuat-request>)))
  "Returns full string definition for message of type '<MoveQuat-request>"
  (cl:format cl:nil "geometry_msgs/Quaternion val~%bool execute~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveQuat-request)))
  "Returns full string definition for message of type 'MoveQuat-request"
  (cl:format cl:nil "geometry_msgs/Quaternion val~%bool execute~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveQuat-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'val))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveQuat-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveQuat-request
    (cl:cons ':val (val msg))
    (cl:cons ':execute (execute msg))
))
;//! \htmlinclude MoveQuat-response.msg.html

(cl:defclass <MoveQuat-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MoveQuat-response (<MoveQuat-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveQuat-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveQuat-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moveit_planner-srv:<MoveQuat-response> is deprecated: use moveit_planner-srv:MoveQuat-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveQuat-response>) ostream)
  "Serializes a message object of type '<MoveQuat-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveQuat-response>) istream)
  "Deserializes a message object of type '<MoveQuat-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveQuat-response>)))
  "Returns string type for a service object of type '<MoveQuat-response>"
  "moveit_planner/MoveQuatResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveQuat-response)))
  "Returns string type for a service object of type 'MoveQuat-response"
  "moveit_planner/MoveQuatResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveQuat-response>)))
  "Returns md5sum for a message object of type '<MoveQuat-response>"
  "38b5b4ab2cb69ba284d4cfd4d2765acc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveQuat-response)))
  "Returns md5sum for a message object of type 'MoveQuat-response"
  "38b5b4ab2cb69ba284d4cfd4d2765acc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveQuat-response>)))
  "Returns full string definition for message of type '<MoveQuat-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveQuat-response)))
  "Returns full string definition for message of type 'MoveQuat-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveQuat-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveQuat-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveQuat-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveQuat)))
  'MoveQuat-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveQuat)))
  'MoveQuat-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveQuat)))
  "Returns string type for a service object of type '<MoveQuat>"
  "moveit_planner/MoveQuat")