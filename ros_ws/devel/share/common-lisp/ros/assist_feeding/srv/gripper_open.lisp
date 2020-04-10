; Auto-generated. Do not edit!


(cl:in-package assist_feeding-srv)


;//! \htmlinclude gripper_open-request.msg.html

(cl:defclass <gripper_open-request> (roslisp-msg-protocol:ros-message)
  ((gripper_state
    :reader gripper_state
    :initarg :gripper_state
    :type cl:integer
    :initform 0))
)

(cl:defclass gripper_open-request (<gripper_open-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gripper_open-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gripper_open-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assist_feeding-srv:<gripper_open-request> is deprecated: use assist_feeding-srv:gripper_open-request instead.")))

(cl:ensure-generic-function 'gripper_state-val :lambda-list '(m))
(cl:defmethod gripper_state-val ((m <gripper_open-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assist_feeding-srv:gripper_state-val is deprecated.  Use assist_feeding-srv:gripper_state instead.")
  (gripper_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gripper_open-request>) ostream)
  "Serializes a message object of type '<gripper_open-request>"
  (cl:let* ((signed (cl:slot-value msg 'gripper_state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gripper_open-request>) istream)
  "Deserializes a message object of type '<gripper_open-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gripper_state) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gripper_open-request>)))
  "Returns string type for a service object of type '<gripper_open-request>"
  "assist_feeding/gripper_openRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gripper_open-request)))
  "Returns string type for a service object of type 'gripper_open-request"
  "assist_feeding/gripper_openRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gripper_open-request>)))
  "Returns md5sum for a message object of type '<gripper_open-request>"
  "7626e2955bd6177e24f663e9b9a1fde1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gripper_open-request)))
  "Returns md5sum for a message object of type 'gripper_open-request"
  "7626e2955bd6177e24f663e9b9a1fde1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gripper_open-request>)))
  "Returns full string definition for message of type '<gripper_open-request>"
  (cl:format cl:nil "int32 gripper_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gripper_open-request)))
  "Returns full string definition for message of type 'gripper_open-request"
  (cl:format cl:nil "int32 gripper_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gripper_open-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gripper_open-request>))
  "Converts a ROS message object to a list"
  (cl:list 'gripper_open-request
    (cl:cons ':gripper_state (gripper_state msg))
))
;//! \htmlinclude gripper_open-response.msg.html

(cl:defclass <gripper_open-response> (roslisp-msg-protocol:ros-message)
  ((gripper_state
    :reader gripper_state
    :initarg :gripper_state
    :type cl:integer
    :initform 0))
)

(cl:defclass gripper_open-response (<gripper_open-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gripper_open-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gripper_open-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assist_feeding-srv:<gripper_open-response> is deprecated: use assist_feeding-srv:gripper_open-response instead.")))

(cl:ensure-generic-function 'gripper_state-val :lambda-list '(m))
(cl:defmethod gripper_state-val ((m <gripper_open-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assist_feeding-srv:gripper_state-val is deprecated.  Use assist_feeding-srv:gripper_state instead.")
  (gripper_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gripper_open-response>) ostream)
  "Serializes a message object of type '<gripper_open-response>"
  (cl:let* ((signed (cl:slot-value msg 'gripper_state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gripper_open-response>) istream)
  "Deserializes a message object of type '<gripper_open-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gripper_state) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gripper_open-response>)))
  "Returns string type for a service object of type '<gripper_open-response>"
  "assist_feeding/gripper_openResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gripper_open-response)))
  "Returns string type for a service object of type 'gripper_open-response"
  "assist_feeding/gripper_openResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gripper_open-response>)))
  "Returns md5sum for a message object of type '<gripper_open-response>"
  "7626e2955bd6177e24f663e9b9a1fde1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gripper_open-response)))
  "Returns md5sum for a message object of type 'gripper_open-response"
  "7626e2955bd6177e24f663e9b9a1fde1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gripper_open-response>)))
  "Returns full string definition for message of type '<gripper_open-response>"
  (cl:format cl:nil "int32 gripper_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gripper_open-response)))
  "Returns full string definition for message of type 'gripper_open-response"
  (cl:format cl:nil "int32 gripper_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gripper_open-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gripper_open-response>))
  "Converts a ROS message object to a list"
  (cl:list 'gripper_open-response
    (cl:cons ':gripper_state (gripper_state msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'gripper_open)))
  'gripper_open-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'gripper_open)))
  'gripper_open-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gripper_open)))
  "Returns string type for a service object of type '<gripper_open>"
  "assist_feeding/gripper_open")