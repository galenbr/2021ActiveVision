; Auto-generated. Do not edit!


(cl:in-package netft_utils-srv)


;//! \htmlinclude SetFilter-request.msg.html

(cl:defclass <SetFilter-request> (roslisp-msg-protocol:ros-message)
  ((toFilter
    :reader toFilter
    :initarg :toFilter
    :type cl:boolean
    :initform cl:nil)
   (deltaT
    :reader deltaT
    :initarg :deltaT
    :type cl:float
    :initform 0.0)
   (cutoffFrequency
    :reader cutoffFrequency
    :initarg :cutoffFrequency
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetFilter-request (<SetFilter-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetFilter-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetFilter-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name netft_utils-srv:<SetFilter-request> is deprecated: use netft_utils-srv:SetFilter-request instead.")))

(cl:ensure-generic-function 'toFilter-val :lambda-list '(m))
(cl:defmethod toFilter-val ((m <SetFilter-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:toFilter-val is deprecated.  Use netft_utils-srv:toFilter instead.")
  (toFilter m))

(cl:ensure-generic-function 'deltaT-val :lambda-list '(m))
(cl:defmethod deltaT-val ((m <SetFilter-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:deltaT-val is deprecated.  Use netft_utils-srv:deltaT instead.")
  (deltaT m))

(cl:ensure-generic-function 'cutoffFrequency-val :lambda-list '(m))
(cl:defmethod cutoffFrequency-val ((m <SetFilter-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:cutoffFrequency-val is deprecated.  Use netft_utils-srv:cutoffFrequency instead.")
  (cutoffFrequency m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetFilter-request>) ostream)
  "Serializes a message object of type '<SetFilter-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'toFilter) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'deltaT))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'cutoffFrequency))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetFilter-request>) istream)
  "Deserializes a message object of type '<SetFilter-request>"
    (cl:setf (cl:slot-value msg 'toFilter) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'deltaT) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cutoffFrequency) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetFilter-request>)))
  "Returns string type for a service object of type '<SetFilter-request>"
  "netft_utils/SetFilterRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFilter-request)))
  "Returns string type for a service object of type 'SetFilter-request"
  "netft_utils/SetFilterRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetFilter-request>)))
  "Returns md5sum for a message object of type '<SetFilter-request>"
  "63edfd1498649d874534855980e23bf0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetFilter-request)))
  "Returns md5sum for a message object of type 'SetFilter-request"
  "63edfd1498649d874534855980e23bf0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetFilter-request>)))
  "Returns full string definition for message of type '<SetFilter-request>"
  (cl:format cl:nil "bool toFilter~%float64 deltaT~%float64 cutoffFrequency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetFilter-request)))
  "Returns full string definition for message of type 'SetFilter-request"
  (cl:format cl:nil "bool toFilter~%float64 deltaT~%float64 cutoffFrequency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetFilter-request>))
  (cl:+ 0
     1
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetFilter-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetFilter-request
    (cl:cons ':toFilter (toFilter msg))
    (cl:cons ':deltaT (deltaT msg))
    (cl:cons ':cutoffFrequency (cutoffFrequency msg))
))
;//! \htmlinclude SetFilter-response.msg.html

(cl:defclass <SetFilter-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetFilter-response (<SetFilter-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetFilter-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetFilter-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name netft_utils-srv:<SetFilter-response> is deprecated: use netft_utils-srv:SetFilter-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetFilter-response>) ostream)
  "Serializes a message object of type '<SetFilter-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetFilter-response>) istream)
  "Deserializes a message object of type '<SetFilter-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetFilter-response>)))
  "Returns string type for a service object of type '<SetFilter-response>"
  "netft_utils/SetFilterResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFilter-response)))
  "Returns string type for a service object of type 'SetFilter-response"
  "netft_utils/SetFilterResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetFilter-response>)))
  "Returns md5sum for a message object of type '<SetFilter-response>"
  "63edfd1498649d874534855980e23bf0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetFilter-response)))
  "Returns md5sum for a message object of type 'SetFilter-response"
  "63edfd1498649d874534855980e23bf0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetFilter-response>)))
  "Returns full string definition for message of type '<SetFilter-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetFilter-response)))
  "Returns full string definition for message of type 'SetFilter-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetFilter-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetFilter-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetFilter-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetFilter)))
  'SetFilter-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetFilter)))
  'SetFilter-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFilter)))
  "Returns string type for a service object of type '<SetFilter>"
  "netft_utils/SetFilter")