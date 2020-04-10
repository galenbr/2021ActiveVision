; Auto-generated. Do not edit!


(cl:in-package netft_utils-srv)


;//! \htmlinclude SetBias-request.msg.html

(cl:defclass <SetBias-request> (roslisp-msg-protocol:ros-message)
  ((toBias
    :reader toBias
    :initarg :toBias
    :type cl:boolean
    :initform cl:nil)
   (forceMax
    :reader forceMax
    :initarg :forceMax
    :type cl:float
    :initform 0.0)
   (torqueMax
    :reader torqueMax
    :initarg :torqueMax
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetBias-request (<SetBias-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetBias-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetBias-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name netft_utils-srv:<SetBias-request> is deprecated: use netft_utils-srv:SetBias-request instead.")))

(cl:ensure-generic-function 'toBias-val :lambda-list '(m))
(cl:defmethod toBias-val ((m <SetBias-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:toBias-val is deprecated.  Use netft_utils-srv:toBias instead.")
  (toBias m))

(cl:ensure-generic-function 'forceMax-val :lambda-list '(m))
(cl:defmethod forceMax-val ((m <SetBias-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:forceMax-val is deprecated.  Use netft_utils-srv:forceMax instead.")
  (forceMax m))

(cl:ensure-generic-function 'torqueMax-val :lambda-list '(m))
(cl:defmethod torqueMax-val ((m <SetBias-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:torqueMax-val is deprecated.  Use netft_utils-srv:torqueMax instead.")
  (torqueMax m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetBias-request>) ostream)
  "Serializes a message object of type '<SetBias-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'toBias) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'forceMax))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'torqueMax))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetBias-request>) istream)
  "Deserializes a message object of type '<SetBias-request>"
    (cl:setf (cl:slot-value msg 'toBias) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'forceMax) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'torqueMax) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetBias-request>)))
  "Returns string type for a service object of type '<SetBias-request>"
  "netft_utils/SetBiasRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBias-request)))
  "Returns string type for a service object of type 'SetBias-request"
  "netft_utils/SetBiasRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetBias-request>)))
  "Returns md5sum for a message object of type '<SetBias-request>"
  "24c559ab4eda1111c9526db57ba60c9c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetBias-request)))
  "Returns md5sum for a message object of type 'SetBias-request"
  "24c559ab4eda1111c9526db57ba60c9c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetBias-request>)))
  "Returns full string definition for message of type '<SetBias-request>"
  (cl:format cl:nil "bool toBias~%float64 forceMax~%float64 torqueMax~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetBias-request)))
  "Returns full string definition for message of type 'SetBias-request"
  (cl:format cl:nil "bool toBias~%float64 forceMax~%float64 torqueMax~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetBias-request>))
  (cl:+ 0
     1
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetBias-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetBias-request
    (cl:cons ':toBias (toBias msg))
    (cl:cons ':forceMax (forceMax msg))
    (cl:cons ':torqueMax (torqueMax msg))
))
;//! \htmlinclude SetBias-response.msg.html

(cl:defclass <SetBias-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetBias-response (<SetBias-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetBias-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetBias-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name netft_utils-srv:<SetBias-response> is deprecated: use netft_utils-srv:SetBias-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetBias-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:success-val is deprecated.  Use netft_utils-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetBias-response>) ostream)
  "Serializes a message object of type '<SetBias-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetBias-response>) istream)
  "Deserializes a message object of type '<SetBias-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetBias-response>)))
  "Returns string type for a service object of type '<SetBias-response>"
  "netft_utils/SetBiasResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBias-response)))
  "Returns string type for a service object of type 'SetBias-response"
  "netft_utils/SetBiasResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetBias-response>)))
  "Returns md5sum for a message object of type '<SetBias-response>"
  "24c559ab4eda1111c9526db57ba60c9c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetBias-response)))
  "Returns md5sum for a message object of type 'SetBias-response"
  "24c559ab4eda1111c9526db57ba60c9c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetBias-response>)))
  "Returns full string definition for message of type '<SetBias-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetBias-response)))
  "Returns full string definition for message of type 'SetBias-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetBias-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetBias-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetBias-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetBias)))
  'SetBias-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetBias)))
  'SetBias-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBias)))
  "Returns string type for a service object of type '<SetBias>"
  "netft_utils/SetBias")