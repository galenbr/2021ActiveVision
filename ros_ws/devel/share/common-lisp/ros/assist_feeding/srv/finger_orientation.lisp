; Auto-generated. Do not edit!


(cl:in-package assist_feeding-srv)


;//! \htmlinclude finger_orientation-request.msg.html

(cl:defclass <finger_orientation-request> (roslisp-msg-protocol:ros-message)
  ((finger_angle
    :reader finger_angle
    :initarg :finger_angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass finger_orientation-request (<finger_orientation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <finger_orientation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'finger_orientation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assist_feeding-srv:<finger_orientation-request> is deprecated: use assist_feeding-srv:finger_orientation-request instead.")))

(cl:ensure-generic-function 'finger_angle-val :lambda-list '(m))
(cl:defmethod finger_angle-val ((m <finger_orientation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assist_feeding-srv:finger_angle-val is deprecated.  Use assist_feeding-srv:finger_angle instead.")
  (finger_angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <finger_orientation-request>) ostream)
  "Serializes a message object of type '<finger_orientation-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'finger_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <finger_orientation-request>) istream)
  "Deserializes a message object of type '<finger_orientation-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'finger_angle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<finger_orientation-request>)))
  "Returns string type for a service object of type '<finger_orientation-request>"
  "assist_feeding/finger_orientationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'finger_orientation-request)))
  "Returns string type for a service object of type 'finger_orientation-request"
  "assist_feeding/finger_orientationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<finger_orientation-request>)))
  "Returns md5sum for a message object of type '<finger_orientation-request>"
  "2649f48d233c8c347b13cd50f5edf8d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'finger_orientation-request)))
  "Returns md5sum for a message object of type 'finger_orientation-request"
  "2649f48d233c8c347b13cd50f5edf8d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<finger_orientation-request>)))
  "Returns full string definition for message of type '<finger_orientation-request>"
  (cl:format cl:nil "float64 finger_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'finger_orientation-request)))
  "Returns full string definition for message of type 'finger_orientation-request"
  (cl:format cl:nil "float64 finger_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <finger_orientation-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <finger_orientation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'finger_orientation-request
    (cl:cons ':finger_angle (finger_angle msg))
))
;//! \htmlinclude finger_orientation-response.msg.html

(cl:defclass <finger_orientation-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass finger_orientation-response (<finger_orientation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <finger_orientation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'finger_orientation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assist_feeding-srv:<finger_orientation-response> is deprecated: use assist_feeding-srv:finger_orientation-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <finger_orientation-response>) ostream)
  "Serializes a message object of type '<finger_orientation-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <finger_orientation-response>) istream)
  "Deserializes a message object of type '<finger_orientation-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<finger_orientation-response>)))
  "Returns string type for a service object of type '<finger_orientation-response>"
  "assist_feeding/finger_orientationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'finger_orientation-response)))
  "Returns string type for a service object of type 'finger_orientation-response"
  "assist_feeding/finger_orientationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<finger_orientation-response>)))
  "Returns md5sum for a message object of type '<finger_orientation-response>"
  "2649f48d233c8c347b13cd50f5edf8d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'finger_orientation-response)))
  "Returns md5sum for a message object of type 'finger_orientation-response"
  "2649f48d233c8c347b13cd50f5edf8d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<finger_orientation-response>)))
  "Returns full string definition for message of type '<finger_orientation-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'finger_orientation-response)))
  "Returns full string definition for message of type 'finger_orientation-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <finger_orientation-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <finger_orientation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'finger_orientation-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'finger_orientation)))
  'finger_orientation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'finger_orientation)))
  'finger_orientation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'finger_orientation)))
  "Returns string type for a service object of type '<finger_orientation>"
  "assist_feeding/finger_orientation")