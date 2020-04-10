; Auto-generated. Do not edit!


(cl:in-package netft_utils-srv)


;//! \htmlinclude SetToolData-request.msg.html

(cl:defclass <SetToolData-request> (roslisp-msg-protocol:ros-message)
  ((mass
    :reader mass
    :initarg :mass
    :type cl:float
    :initform 0.0)
   (COM
    :reader COM
    :initarg :COM
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetToolData-request (<SetToolData-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetToolData-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetToolData-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name netft_utils-srv:<SetToolData-request> is deprecated: use netft_utils-srv:SetToolData-request instead.")))

(cl:ensure-generic-function 'mass-val :lambda-list '(m))
(cl:defmethod mass-val ((m <SetToolData-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:mass-val is deprecated.  Use netft_utils-srv:mass instead.")
  (mass m))

(cl:ensure-generic-function 'COM-val :lambda-list '(m))
(cl:defmethod COM-val ((m <SetToolData-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:COM-val is deprecated.  Use netft_utils-srv:COM instead.")
  (COM m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetToolData-request>) ostream)
  "Serializes a message object of type '<SetToolData-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'mass))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'COM))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetToolData-request>) istream)
  "Deserializes a message object of type '<SetToolData-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mass) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'COM) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetToolData-request>)))
  "Returns string type for a service object of type '<SetToolData-request>"
  "netft_utils/SetToolDataRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetToolData-request)))
  "Returns string type for a service object of type 'SetToolData-request"
  "netft_utils/SetToolDataRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetToolData-request>)))
  "Returns md5sum for a message object of type '<SetToolData-request>"
  "d732082be6860b41d1ad5c3342683b63")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetToolData-request)))
  "Returns md5sum for a message object of type 'SetToolData-request"
  "d732082be6860b41d1ad5c3342683b63")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetToolData-request>)))
  "Returns full string definition for message of type '<SetToolData-request>"
  (cl:format cl:nil "float64 mass~%float64 COM~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetToolData-request)))
  "Returns full string definition for message of type 'SetToolData-request"
  (cl:format cl:nil "float64 mass~%float64 COM~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetToolData-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetToolData-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetToolData-request
    (cl:cons ':mass (mass msg))
    (cl:cons ':COM (COM msg))
))
;//! \htmlinclude SetToolData-response.msg.html

(cl:defclass <SetToolData-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetToolData-response (<SetToolData-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetToolData-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetToolData-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name netft_utils-srv:<SetToolData-response> is deprecated: use netft_utils-srv:SetToolData-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetToolData-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-srv:success-val is deprecated.  Use netft_utils-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetToolData-response>) ostream)
  "Serializes a message object of type '<SetToolData-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetToolData-response>) istream)
  "Deserializes a message object of type '<SetToolData-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetToolData-response>)))
  "Returns string type for a service object of type '<SetToolData-response>"
  "netft_utils/SetToolDataResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetToolData-response)))
  "Returns string type for a service object of type 'SetToolData-response"
  "netft_utils/SetToolDataResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetToolData-response>)))
  "Returns md5sum for a message object of type '<SetToolData-response>"
  "d732082be6860b41d1ad5c3342683b63")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetToolData-response)))
  "Returns md5sum for a message object of type 'SetToolData-response"
  "d732082be6860b41d1ad5c3342683b63")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetToolData-response>)))
  "Returns full string definition for message of type '<SetToolData-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetToolData-response)))
  "Returns full string definition for message of type 'SetToolData-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetToolData-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetToolData-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetToolData-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetToolData)))
  'SetToolData-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetToolData)))
  'SetToolData-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetToolData)))
  "Returns string type for a service object of type '<SetToolData>"
  "netft_utils/SetToolData")