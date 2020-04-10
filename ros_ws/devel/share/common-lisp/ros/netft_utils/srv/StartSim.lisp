; Auto-generated. Do not edit!


(cl:in-package netft_utils-srv)


;//! \htmlinclude StartSim-request.msg.html

(cl:defclass <StartSim-request> (roslisp-msg-protocol:ros-message)
  ((simDim
    :reader simDim
    :initarg :simDim
    :type cl:integer
    :initform 0)
   (simType
    :reader simType
    :initarg :simType
    :type cl:integer
    :initform 0)
   (simSlope
    :reader simSlope
    :initarg :simSlope
    :type cl:float
    :initform 0.0)
   (maxForce
    :reader maxForce
    :initarg :maxForce
    :type cl:float
    :initform 0.0))
)

(cl:defclass StartSim-request (<StartSim-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartSim-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartSim-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name netft_utils-srv:<StartSim-request> is deprecated: use netft_utils-srv:StartSim-request instead.")))

(cl:ensure-generic-function 'simDim-val :lambda-list '(m))
(cl:defmethod simDim-val ((m <StartSim-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:simDim-val is deprecated.  Use netft_utils-srv:simDim instead.")
  (simDim m))

(cl:ensure-generic-function 'simType-val :lambda-list '(m))
(cl:defmethod simType-val ((m <StartSim-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:simType-val is deprecated.  Use netft_utils-srv:simType instead.")
  (simType m))

(cl:ensure-generic-function 'simSlope-val :lambda-list '(m))
(cl:defmethod simSlope-val ((m <StartSim-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:simSlope-val is deprecated.  Use netft_utils-srv:simSlope instead.")
  (simSlope m))

(cl:ensure-generic-function 'maxForce-val :lambda-list '(m))
(cl:defmethod maxForce-val ((m <StartSim-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:maxForce-val is deprecated.  Use netft_utils-srv:maxForce instead.")
  (maxForce m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartSim-request>) ostream)
  "Serializes a message object of type '<StartSim-request>"
  (cl:let* ((signed (cl:slot-value msg 'simDim)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'simType)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'simSlope))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'maxForce))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartSim-request>) istream)
  "Deserializes a message object of type '<StartSim-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'simDim) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'simType) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'simSlope) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'maxForce) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartSim-request>)))
  "Returns string type for a service object of type '<StartSim-request>"
  "netft_utils/StartSimRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartSim-request)))
  "Returns string type for a service object of type 'StartSim-request"
  "netft_utils/StartSimRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartSim-request>)))
  "Returns md5sum for a message object of type '<StartSim-request>"
  "f9bc0ea45e890996bb2e2ce115e7c8c3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartSim-request)))
  "Returns md5sum for a message object of type 'StartSim-request"
  "f9bc0ea45e890996bb2e2ce115e7c8c3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartSim-request>)))
  "Returns full string definition for message of type '<StartSim-request>"
  (cl:format cl:nil "int32 simDim~%int32 simType~%float64 simSlope~%float64 maxForce~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartSim-request)))
  "Returns full string definition for message of type 'StartSim-request"
  (cl:format cl:nil "int32 simDim~%int32 simType~%float64 simSlope~%float64 maxForce~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartSim-request>))
  (cl:+ 0
     4
     4
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartSim-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StartSim-request
    (cl:cons ':simDim (simDim msg))
    (cl:cons ':simType (simType msg))
    (cl:cons ':simSlope (simSlope msg))
    (cl:cons ':maxForce (maxForce msg))
))
;//! \htmlinclude StartSim-response.msg.html

(cl:defclass <StartSim-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StartSim-response (<StartSim-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartSim-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartSim-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name netft_utils-srv:<StartSim-response> is deprecated: use netft_utils-srv:StartSim-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartSim-response>) ostream)
  "Serializes a message object of type '<StartSim-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartSim-response>) istream)
  "Deserializes a message object of type '<StartSim-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartSim-response>)))
  "Returns string type for a service object of type '<StartSim-response>"
  "netft_utils/StartSimResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartSim-response)))
  "Returns string type for a service object of type 'StartSim-response"
  "netft_utils/StartSimResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartSim-response>)))
  "Returns md5sum for a message object of type '<StartSim-response>"
  "f9bc0ea45e890996bb2e2ce115e7c8c3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartSim-response)))
  "Returns md5sum for a message object of type 'StartSim-response"
  "f9bc0ea45e890996bb2e2ce115e7c8c3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartSim-response>)))
  "Returns full string definition for message of type '<StartSim-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartSim-response)))
  "Returns full string definition for message of type 'StartSim-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartSim-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartSim-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StartSim-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StartSim)))
  'StartSim-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StartSim)))
  'StartSim-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartSim)))
  "Returns string type for a service object of type '<StartSim>"
  "netft_utils/StartSim")