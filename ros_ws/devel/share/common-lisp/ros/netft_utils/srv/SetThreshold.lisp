; Auto-generated. Do not edit!


(cl:in-package netft_utils-srv)


;//! \htmlinclude SetThreshold-request.msg.html

(cl:defclass <SetThreshold-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type geometry_msgs-msg:WrenchStamped
    :initform (cl:make-instance 'geometry_msgs-msg:WrenchStamped)))
)

(cl:defclass SetThreshold-request (<SetThreshold-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetThreshold-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetThreshold-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name netft_utils-srv:<SetThreshold-request> is deprecated: use netft_utils-srv:SetThreshold-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <SetThreshold-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:data-val is deprecated.  Use netft_utils-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetThreshold-request>) ostream)
  "Serializes a message object of type '<SetThreshold-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetThreshold-request>) istream)
  "Deserializes a message object of type '<SetThreshold-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetThreshold-request>)))
  "Returns string type for a service object of type '<SetThreshold-request>"
  "netft_utils/SetThresholdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetThreshold-request)))
  "Returns string type for a service object of type 'SetThreshold-request"
  "netft_utils/SetThresholdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetThreshold-request>)))
  "Returns md5sum for a message object of type '<SetThreshold-request>"
  "0a518cd2ef382108fafe506ec521a6c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetThreshold-request)))
  "Returns md5sum for a message object of type 'SetThreshold-request"
  "0a518cd2ef382108fafe506ec521a6c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetThreshold-request>)))
  "Returns full string definition for message of type '<SetThreshold-request>"
  (cl:format cl:nil "geometry_msgs/WrenchStamped data~%~%================================================================================~%MSG: geometry_msgs/WrenchStamped~%# A wrench with reference coordinate frame and timestamp~%Header header~%Wrench wrench~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetThreshold-request)))
  "Returns full string definition for message of type 'SetThreshold-request"
  (cl:format cl:nil "geometry_msgs/WrenchStamped data~%~%================================================================================~%MSG: geometry_msgs/WrenchStamped~%# A wrench with reference coordinate frame and timestamp~%Header header~%Wrench wrench~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetThreshold-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetThreshold-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetThreshold-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude SetThreshold-response.msg.html

(cl:defclass <SetThreshold-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetThreshold-response (<SetThreshold-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetThreshold-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetThreshold-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name netft_utils-srv:<SetThreshold-response> is deprecated: use netft_utils-srv:SetThreshold-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetThreshold-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:success-val is deprecated.  Use netft_utils-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetThreshold-response>) ostream)
  "Serializes a message object of type '<SetThreshold-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetThreshold-response>) istream)
  "Deserializes a message object of type '<SetThreshold-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetThreshold-response>)))
  "Returns string type for a service object of type '<SetThreshold-response>"
  "netft_utils/SetThresholdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetThreshold-response)))
  "Returns string type for a service object of type 'SetThreshold-response"
  "netft_utils/SetThresholdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetThreshold-response>)))
  "Returns md5sum for a message object of type '<SetThreshold-response>"
  "0a518cd2ef382108fafe506ec521a6c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetThreshold-response)))
  "Returns md5sum for a message object of type 'SetThreshold-response"
  "0a518cd2ef382108fafe506ec521a6c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetThreshold-response>)))
  "Returns full string definition for message of type '<SetThreshold-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetThreshold-response)))
  "Returns full string definition for message of type 'SetThreshold-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetThreshold-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetThreshold-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetThreshold-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetThreshold)))
  'SetThreshold-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetThreshold)))
  'SetThreshold-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetThreshold)))
  "Returns string type for a service object of type '<SetThreshold>"
  "netft_utils/SetThreshold")