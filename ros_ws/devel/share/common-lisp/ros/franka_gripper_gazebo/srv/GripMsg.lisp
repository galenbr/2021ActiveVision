; Auto-generated. Do not edit!


(cl:in-package franka_gripper_gazebo-srv)


;//! \htmlinclude GripMsg-request.msg.html

(cl:defclass <GripMsg-request> (roslisp-msg-protocol:ros-message)
  ((force
    :reader force
    :initarg :force
    :type cl:float
    :initform 0.0))
)

(cl:defclass GripMsg-request (<GripMsg-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripMsg-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripMsg-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name franka_gripper_gazebo-srv:<GripMsg-request> is deprecated: use franka_gripper_gazebo-srv:GripMsg-request instead.")))

(cl:ensure-generic-function 'force-val :lambda-list '(m))
(cl:defmethod force-val ((m <GripMsg-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_gripper_gazebo-srv:force-val is deprecated.  Use franka_gripper_gazebo-srv:force instead.")
  (force m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripMsg-request>) ostream)
  "Serializes a message object of type '<GripMsg-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'force))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripMsg-request>) istream)
  "Deserializes a message object of type '<GripMsg-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'force) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripMsg-request>)))
  "Returns string type for a service object of type '<GripMsg-request>"
  "franka_gripper_gazebo/GripMsgRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripMsg-request)))
  "Returns string type for a service object of type 'GripMsg-request"
  "franka_gripper_gazebo/GripMsgRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripMsg-request>)))
  "Returns md5sum for a message object of type '<GripMsg-request>"
  "e18a51329659ac6263f87aaf9a01fe62")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripMsg-request)))
  "Returns md5sum for a message object of type 'GripMsg-request"
  "e18a51329659ac6263f87aaf9a01fe62")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripMsg-request>)))
  "Returns full string definition for message of type '<GripMsg-request>"
  (cl:format cl:nil "float64 force~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripMsg-request)))
  "Returns full string definition for message of type 'GripMsg-request"
  (cl:format cl:nil "float64 force~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripMsg-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripMsg-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GripMsg-request
    (cl:cons ':force (force msg))
))
;//! \htmlinclude GripMsg-response.msg.html

(cl:defclass <GripMsg-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GripMsg-response (<GripMsg-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripMsg-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripMsg-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name franka_gripper_gazebo-srv:<GripMsg-response> is deprecated: use franka_gripper_gazebo-srv:GripMsg-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripMsg-response>) ostream)
  "Serializes a message object of type '<GripMsg-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripMsg-response>) istream)
  "Deserializes a message object of type '<GripMsg-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripMsg-response>)))
  "Returns string type for a service object of type '<GripMsg-response>"
  "franka_gripper_gazebo/GripMsgResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripMsg-response)))
  "Returns string type for a service object of type 'GripMsg-response"
  "franka_gripper_gazebo/GripMsgResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripMsg-response>)))
  "Returns md5sum for a message object of type '<GripMsg-response>"
  "e18a51329659ac6263f87aaf9a01fe62")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripMsg-response)))
  "Returns md5sum for a message object of type 'GripMsg-response"
  "e18a51329659ac6263f87aaf9a01fe62")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripMsg-response>)))
  "Returns full string definition for message of type '<GripMsg-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripMsg-response)))
  "Returns full string definition for message of type 'GripMsg-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripMsg-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripMsg-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GripMsg-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GripMsg)))
  'GripMsg-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GripMsg)))
  'GripMsg-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripMsg)))
  "Returns string type for a service object of type '<GripMsg>"
  "franka_gripper_gazebo/GripMsg")