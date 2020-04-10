; Auto-generated. Do not edit!


(cl:in-package moveit_planner-srv)


;//! \htmlinclude MovePoint-request.msg.html

(cl:defclass <MovePoint-request> (roslisp-msg-protocol:ros-message)
  ((val
    :reader val
    :initarg :val
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (execute
    :reader execute
    :initarg :execute
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MovePoint-request (<MovePoint-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MovePoint-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MovePoint-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moveit_planner-srv:<MovePoint-request> is deprecated: use moveit_planner-srv:MovePoint-request instead.")))

(cl:ensure-generic-function 'val-val :lambda-list '(m))
(cl:defmethod val-val ((m <MovePoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moveit_planner-srv:val-val is deprecated.  Use moveit_planner-srv:val instead.")
  (val m))

(cl:ensure-generic-function 'execute-val :lambda-list '(m))
(cl:defmethod execute-val ((m <MovePoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moveit_planner-srv:execute-val is deprecated.  Use moveit_planner-srv:execute instead.")
  (execute m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MovePoint-request>) ostream)
  "Serializes a message object of type '<MovePoint-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'val) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'execute) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MovePoint-request>) istream)
  "Deserializes a message object of type '<MovePoint-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'val) istream)
    (cl:setf (cl:slot-value msg 'execute) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MovePoint-request>)))
  "Returns string type for a service object of type '<MovePoint-request>"
  "moveit_planner/MovePointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MovePoint-request)))
  "Returns string type for a service object of type 'MovePoint-request"
  "moveit_planner/MovePointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MovePoint-request>)))
  "Returns md5sum for a message object of type '<MovePoint-request>"
  "09d178cfe7c05e5d8c3ba0747602407a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MovePoint-request)))
  "Returns md5sum for a message object of type 'MovePoint-request"
  "09d178cfe7c05e5d8c3ba0747602407a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MovePoint-request>)))
  "Returns full string definition for message of type '<MovePoint-request>"
  (cl:format cl:nil "geometry_msgs/Point val~%bool execute~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MovePoint-request)))
  "Returns full string definition for message of type 'MovePoint-request"
  (cl:format cl:nil "geometry_msgs/Point val~%bool execute~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MovePoint-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'val))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MovePoint-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MovePoint-request
    (cl:cons ':val (val msg))
    (cl:cons ':execute (execute msg))
))
;//! \htmlinclude MovePoint-response.msg.html

(cl:defclass <MovePoint-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MovePoint-response (<MovePoint-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MovePoint-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MovePoint-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moveit_planner-srv:<MovePoint-response> is deprecated: use moveit_planner-srv:MovePoint-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MovePoint-response>) ostream)
  "Serializes a message object of type '<MovePoint-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MovePoint-response>) istream)
  "Deserializes a message object of type '<MovePoint-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MovePoint-response>)))
  "Returns string type for a service object of type '<MovePoint-response>"
  "moveit_planner/MovePointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MovePoint-response)))
  "Returns string type for a service object of type 'MovePoint-response"
  "moveit_planner/MovePointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MovePoint-response>)))
  "Returns md5sum for a message object of type '<MovePoint-response>"
  "09d178cfe7c05e5d8c3ba0747602407a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MovePoint-response)))
  "Returns md5sum for a message object of type 'MovePoint-response"
  "09d178cfe7c05e5d8c3ba0747602407a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MovePoint-response>)))
  "Returns full string definition for message of type '<MovePoint-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MovePoint-response)))
  "Returns full string definition for message of type 'MovePoint-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MovePoint-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MovePoint-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MovePoint-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MovePoint)))
  'MovePoint-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MovePoint)))
  'MovePoint-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MovePoint)))
  "Returns string type for a service object of type '<MovePoint>"
  "moveit_planner/MovePoint")