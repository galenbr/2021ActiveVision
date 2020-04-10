; Auto-generated. Do not edit!


(cl:in-package piercing_experiments_msgs-srv)


;//! \htmlinclude CartesianPath-request.msg.html

(cl:defclass <CartesianPath-request> (roslisp-msg-protocol:ros-message)
  ((final_target
    :reader final_target
    :initarg :final_target
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (vel
    :reader vel
    :initarg :vel
    :type cl:float
    :initform 0.0))
)

(cl:defclass CartesianPath-request (<CartesianPath-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CartesianPath-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CartesianPath-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name piercing_experiments_msgs-srv:<CartesianPath-request> is deprecated: use piercing_experiments_msgs-srv:CartesianPath-request instead.")))

(cl:ensure-generic-function 'final_target-val :lambda-list '(m))
(cl:defmethod final_target-val ((m <CartesianPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader piercing_experiments_msgs-srv:final_target-val is deprecated.  Use piercing_experiments_msgs-srv:final_target instead.")
  (final_target m))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <CartesianPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader piercing_experiments_msgs-srv:vel-val is deprecated.  Use piercing_experiments_msgs-srv:vel instead.")
  (vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CartesianPath-request>) ostream)
  "Serializes a message object of type '<CartesianPath-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'final_target) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CartesianPath-request>) istream)
  "Deserializes a message object of type '<CartesianPath-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'final_target) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CartesianPath-request>)))
  "Returns string type for a service object of type '<CartesianPath-request>"
  "piercing_experiments_msgs/CartesianPathRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CartesianPath-request)))
  "Returns string type for a service object of type 'CartesianPath-request"
  "piercing_experiments_msgs/CartesianPathRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CartesianPath-request>)))
  "Returns md5sum for a message object of type '<CartesianPath-request>"
  "c01a67c204f217a13fc2a48450b5290b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CartesianPath-request)))
  "Returns md5sum for a message object of type 'CartesianPath-request"
  "c01a67c204f217a13fc2a48450b5290b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CartesianPath-request>)))
  "Returns full string definition for message of type '<CartesianPath-request>"
  (cl:format cl:nil "geometry_msgs/Pose final_target~%float32 vel~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CartesianPath-request)))
  "Returns full string definition for message of type 'CartesianPath-request"
  (cl:format cl:nil "geometry_msgs/Pose final_target~%float32 vel~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CartesianPath-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'final_target))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CartesianPath-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CartesianPath-request
    (cl:cons ':final_target (final_target msg))
    (cl:cons ':vel (vel msg))
))
;//! \htmlinclude CartesianPath-response.msg.html

(cl:defclass <CartesianPath-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass CartesianPath-response (<CartesianPath-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CartesianPath-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CartesianPath-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name piercing_experiments_msgs-srv:<CartesianPath-response> is deprecated: use piercing_experiments_msgs-srv:CartesianPath-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CartesianPath-response>) ostream)
  "Serializes a message object of type '<CartesianPath-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CartesianPath-response>) istream)
  "Deserializes a message object of type '<CartesianPath-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CartesianPath-response>)))
  "Returns string type for a service object of type '<CartesianPath-response>"
  "piercing_experiments_msgs/CartesianPathResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CartesianPath-response)))
  "Returns string type for a service object of type 'CartesianPath-response"
  "piercing_experiments_msgs/CartesianPathResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CartesianPath-response>)))
  "Returns md5sum for a message object of type '<CartesianPath-response>"
  "c01a67c204f217a13fc2a48450b5290b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CartesianPath-response)))
  "Returns md5sum for a message object of type 'CartesianPath-response"
  "c01a67c204f217a13fc2a48450b5290b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CartesianPath-response>)))
  "Returns full string definition for message of type '<CartesianPath-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CartesianPath-response)))
  "Returns full string definition for message of type 'CartesianPath-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CartesianPath-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CartesianPath-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CartesianPath-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CartesianPath)))
  'CartesianPath-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CartesianPath)))
  'CartesianPath-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CartesianPath)))
  "Returns string type for a service object of type '<CartesianPath>"
  "piercing_experiments_msgs/CartesianPath")