; Auto-generated. Do not edit!


(cl:in-package netft_utils-srv)


;//! \htmlinclude SetMax-request.msg.html

(cl:defclass <SetMax-request> (roslisp-msg-protocol:ros-message)
  ((forceMax
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

(cl:defclass SetMax-request (<SetMax-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetMax-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetMax-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name netft_utils-srv:<SetMax-request> is deprecated: use netft_utils-srv:SetMax-request instead.")))

(cl:ensure-generic-function 'forceMax-val :lambda-list '(m))
(cl:defmethod forceMax-val ((m <SetMax-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:forceMax-val is deprecated.  Use netft_utils-srv:forceMax instead.")
  (forceMax m))

(cl:ensure-generic-function 'torqueMax-val :lambda-list '(m))
(cl:defmethod torqueMax-val ((m <SetMax-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:torqueMax-val is deprecated.  Use netft_utils-srv:torqueMax instead.")
  (torqueMax m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetMax-request>) ostream)
  "Serializes a message object of type '<SetMax-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetMax-request>) istream)
  "Deserializes a message object of type '<SetMax-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetMax-request>)))
  "Returns string type for a service object of type '<SetMax-request>"
  "netft_utils/SetMaxRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMax-request)))
  "Returns string type for a service object of type 'SetMax-request"
  "netft_utils/SetMaxRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetMax-request>)))
  "Returns md5sum for a message object of type '<SetMax-request>"
  "10e2f2a95c7447ec4aaa55f68d560d78")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetMax-request)))
  "Returns md5sum for a message object of type 'SetMax-request"
  "10e2f2a95c7447ec4aaa55f68d560d78")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetMax-request>)))
  "Returns full string definition for message of type '<SetMax-request>"
  (cl:format cl:nil "float64 forceMax~%float64 torqueMax~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetMax-request)))
  "Returns full string definition for message of type 'SetMax-request"
  (cl:format cl:nil "float64 forceMax~%float64 torqueMax~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetMax-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetMax-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetMax-request
    (cl:cons ':forceMax (forceMax msg))
    (cl:cons ':torqueMax (torqueMax msg))
))
;//! \htmlinclude SetMax-response.msg.html

(cl:defclass <SetMax-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetMax-response (<SetMax-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetMax-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetMax-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name netft_utils-srv:<SetMax-response> is deprecated: use netft_utils-srv:SetMax-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetMax-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:success-val is deprecated.  Use netft_utils-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetMax-response>) ostream)
  "Serializes a message object of type '<SetMax-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetMax-response>) istream)
  "Deserializes a message object of type '<SetMax-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetMax-response>)))
  "Returns string type for a service object of type '<SetMax-response>"
  "netft_utils/SetMaxResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMax-response)))
  "Returns string type for a service object of type 'SetMax-response"
  "netft_utils/SetMaxResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetMax-response>)))
  "Returns md5sum for a message object of type '<SetMax-response>"
  "10e2f2a95c7447ec4aaa55f68d560d78")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetMax-response)))
  "Returns md5sum for a message object of type 'SetMax-response"
  "10e2f2a95c7447ec4aaa55f68d560d78")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetMax-response>)))
  "Returns full string definition for message of type '<SetMax-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetMax-response)))
  "Returns full string definition for message of type 'SetMax-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetMax-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetMax-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetMax-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetMax)))
  'SetMax-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetMax)))
  'SetMax-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMax)))
  "Returns string type for a service object of type '<SetMax>"
  "netft_utils/SetMax")