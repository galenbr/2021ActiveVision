; Auto-generated. Do not edit!


(cl:in-package netft_utils-srv)


;//! \htmlinclude GetDouble-request.msg.html

(cl:defclass <GetDouble-request> (roslisp-msg-protocol:ros-message)
  ((ignore
    :reader ignore
    :initarg :ignore
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetDouble-request (<GetDouble-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDouble-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDouble-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name netft_utils-srv:<GetDouble-request> is deprecated: use netft_utils-srv:GetDouble-request instead.")))

(cl:ensure-generic-function 'ignore-val :lambda-list '(m))
(cl:defmethod ignore-val ((m <GetDouble-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:ignore-val is deprecated.  Use netft_utils-srv:ignore instead.")
  (ignore m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDouble-request>) ostream)
  "Serializes a message object of type '<GetDouble-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ignore) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDouble-request>) istream)
  "Deserializes a message object of type '<GetDouble-request>"
    (cl:setf (cl:slot-value msg 'ignore) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDouble-request>)))
  "Returns string type for a service object of type '<GetDouble-request>"
  "netft_utils/GetDoubleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDouble-request)))
  "Returns string type for a service object of type 'GetDouble-request"
  "netft_utils/GetDoubleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDouble-request>)))
  "Returns md5sum for a message object of type '<GetDouble-request>"
  "43623635c0100a1d66762de2a2c1a2da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDouble-request)))
  "Returns md5sum for a message object of type 'GetDouble-request"
  "43623635c0100a1d66762de2a2c1a2da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDouble-request>)))
  "Returns full string definition for message of type '<GetDouble-request>"
  (cl:format cl:nil "bool ignore~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDouble-request)))
  "Returns full string definition for message of type 'GetDouble-request"
  (cl:format cl:nil "bool ignore~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDouble-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDouble-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDouble-request
    (cl:cons ':ignore (ignore msg))
))
;//! \htmlinclude GetDouble-response.msg.html

(cl:defclass <GetDouble-response> (roslisp-msg-protocol:ros-message)
  ((weight
    :reader weight
    :initarg :weight
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetDouble-response (<GetDouble-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDouble-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDouble-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name netft_utils-srv:<GetDouble-response> is deprecated: use netft_utils-srv:GetDouble-response instead.")))

(cl:ensure-generic-function 'weight-val :lambda-list '(m))
(cl:defmethod weight-val ((m <GetDouble-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:weight-val is deprecated.  Use netft_utils-srv:weight instead.")
  (weight m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDouble-response>) ostream)
  "Serializes a message object of type '<GetDouble-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'weight))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDouble-response>) istream)
  "Deserializes a message object of type '<GetDouble-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'weight) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDouble-response>)))
  "Returns string type for a service object of type '<GetDouble-response>"
  "netft_utils/GetDoubleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDouble-response)))
  "Returns string type for a service object of type 'GetDouble-response"
  "netft_utils/GetDoubleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDouble-response>)))
  "Returns md5sum for a message object of type '<GetDouble-response>"
  "43623635c0100a1d66762de2a2c1a2da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDouble-response)))
  "Returns md5sum for a message object of type 'GetDouble-response"
  "43623635c0100a1d66762de2a2c1a2da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDouble-response>)))
  "Returns full string definition for message of type '<GetDouble-response>"
  (cl:format cl:nil "float64 weight~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDouble-response)))
  "Returns full string definition for message of type 'GetDouble-response"
  (cl:format cl:nil "float64 weight~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDouble-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDouble-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDouble-response
    (cl:cons ':weight (weight msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetDouble)))
  'GetDouble-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetDouble)))
  'GetDouble-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDouble)))
  "Returns string type for a service object of type '<GetDouble>"
  "netft_utils/GetDouble")