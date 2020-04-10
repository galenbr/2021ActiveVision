; Auto-generated. Do not edit!


(cl:in-package piercing_experiments_msgs-srv)


;//! \htmlinclude PrePose-request.msg.html

(cl:defclass <PrePose-request> (roslisp-msg-protocol:ros-message)
  ((target_1
    :reader target_1
    :initarg :target_1
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass PrePose-request (<PrePose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PrePose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PrePose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name piercing_experiments_msgs-srv:<PrePose-request> is deprecated: use piercing_experiments_msgs-srv:PrePose-request instead.")))

(cl:ensure-generic-function 'target_1-val :lambda-list '(m))
(cl:defmethod target_1-val ((m <PrePose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader piercing_experiments_msgs-srv:target_1-val is deprecated.  Use piercing_experiments_msgs-srv:target_1 instead.")
  (target_1 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PrePose-request>) ostream)
  "Serializes a message object of type '<PrePose-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_1) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PrePose-request>) istream)
  "Deserializes a message object of type '<PrePose-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_1) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PrePose-request>)))
  "Returns string type for a service object of type '<PrePose-request>"
  "piercing_experiments_msgs/PrePoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PrePose-request)))
  "Returns string type for a service object of type 'PrePose-request"
  "piercing_experiments_msgs/PrePoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PrePose-request>)))
  "Returns md5sum for a message object of type '<PrePose-request>"
  "315b676814b67387d174953d2c38a6f1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PrePose-request)))
  "Returns md5sum for a message object of type 'PrePose-request"
  "315b676814b67387d174953d2c38a6f1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PrePose-request>)))
  "Returns full string definition for message of type '<PrePose-request>"
  (cl:format cl:nil "geometry_msgs/Pose target_1~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PrePose-request)))
  "Returns full string definition for message of type 'PrePose-request"
  (cl:format cl:nil "geometry_msgs/Pose target_1~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PrePose-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_1))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PrePose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PrePose-request
    (cl:cons ':target_1 (target_1 msg))
))
;//! \htmlinclude PrePose-response.msg.html

(cl:defclass <PrePose-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass PrePose-response (<PrePose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PrePose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PrePose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name piercing_experiments_msgs-srv:<PrePose-response> is deprecated: use piercing_experiments_msgs-srv:PrePose-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PrePose-response>) ostream)
  "Serializes a message object of type '<PrePose-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PrePose-response>) istream)
  "Deserializes a message object of type '<PrePose-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PrePose-response>)))
  "Returns string type for a service object of type '<PrePose-response>"
  "piercing_experiments_msgs/PrePoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PrePose-response)))
  "Returns string type for a service object of type 'PrePose-response"
  "piercing_experiments_msgs/PrePoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PrePose-response>)))
  "Returns md5sum for a message object of type '<PrePose-response>"
  "315b676814b67387d174953d2c38a6f1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PrePose-response)))
  "Returns md5sum for a message object of type 'PrePose-response"
  "315b676814b67387d174953d2c38a6f1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PrePose-response>)))
  "Returns full string definition for message of type '<PrePose-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PrePose-response)))
  "Returns full string definition for message of type 'PrePose-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PrePose-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PrePose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PrePose-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PrePose)))
  'PrePose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PrePose)))
  'PrePose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PrePose)))
  "Returns string type for a service object of type '<PrePose>"
  "piercing_experiments_msgs/PrePose")