; Auto-generated. Do not edit!


(cl:in-package assist_feeding-srv)


;//! \htmlinclude gripper_close-request.msg.html

(cl:defclass <gripper_close-request> (roslisp-msg-protocol:ros-message)
  ((gripper_state
    :reader gripper_state
    :initarg :gripper_state
    :type cl:integer
    :initform 0))
)

(cl:defclass gripper_close-request (<gripper_close-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gripper_close-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gripper_close-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assist_feeding-srv:<gripper_close-request> is deprecated: use assist_feeding-srv:gripper_close-request instead.")))

(cl:ensure-generic-function 'gripper_state-val :lambda-list '(m))
(cl:defmethod gripper_state-val ((m <gripper_close-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assist_feeding-srv:gripper_state-val is deprecated.  Use assist_feeding-srv:gripper_state instead.")
  (gripper_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gripper_close-request>) ostream)
  "Serializes a message object of type '<gripper_close-request>"
  (cl:let* ((signed (cl:slot-value msg 'gripper_state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gripper_close-request>) istream)
  "Deserializes a message object of type '<gripper_close-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gripper_state) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gripper_close-request>)))
  "Returns string type for a service object of type '<gripper_close-request>"
  "assist_feeding/gripper_closeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gripper_close-request)))
  "Returns string type for a service object of type 'gripper_close-request"
  "assist_feeding/gripper_closeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gripper_close-request>)))
  "Returns md5sum for a message object of type '<gripper_close-request>"
  "7626e2955bd6177e24f663e9b9a1fde1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gripper_close-request)))
  "Returns md5sum for a message object of type 'gripper_close-request"
  "7626e2955bd6177e24f663e9b9a1fde1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gripper_close-request>)))
  "Returns full string definition for message of type '<gripper_close-request>"
  (cl:format cl:nil "int32 gripper_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gripper_close-request)))
  "Returns full string definition for message of type 'gripper_close-request"
  (cl:format cl:nil "int32 gripper_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gripper_close-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gripper_close-request>))
  "Converts a ROS message object to a list"
  (cl:list 'gripper_close-request
    (cl:cons ':gripper_state (gripper_state msg))
))
;//! \htmlinclude gripper_close-response.msg.html

(cl:defclass <gripper_close-response> (roslisp-msg-protocol:ros-message)
  ((gripper_state
    :reader gripper_state
    :initarg :gripper_state
    :type cl:integer
    :initform 0))
)

(cl:defclass gripper_close-response (<gripper_close-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gripper_close-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gripper_close-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assist_feeding-srv:<gripper_close-response> is deprecated: use assist_feeding-srv:gripper_close-response instead.")))

(cl:ensure-generic-function 'gripper_state-val :lambda-list '(m))
(cl:defmethod gripper_state-val ((m <gripper_close-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assist_feeding-srv:gripper_state-val is deprecated.  Use assist_feeding-srv:gripper_state instead.")
  (gripper_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gripper_close-response>) ostream)
  "Serializes a message object of type '<gripper_close-response>"
  (cl:let* ((signed (cl:slot-value msg 'gripper_state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gripper_close-response>) istream)
  "Deserializes a message object of type '<gripper_close-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gripper_state) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gripper_close-response>)))
  "Returns string type for a service object of type '<gripper_close-response>"
  "assist_feeding/gripper_closeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gripper_close-response)))
  "Returns string type for a service object of type 'gripper_close-response"
  "assist_feeding/gripper_closeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gripper_close-response>)))
  "Returns md5sum for a message object of type '<gripper_close-response>"
  "7626e2955bd6177e24f663e9b9a1fde1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gripper_close-response)))
  "Returns md5sum for a message object of type 'gripper_close-response"
  "7626e2955bd6177e24f663e9b9a1fde1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gripper_close-response>)))
  "Returns full string definition for message of type '<gripper_close-response>"
  (cl:format cl:nil "int32 gripper_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gripper_close-response)))
  "Returns full string definition for message of type 'gripper_close-response"
  (cl:format cl:nil "int32 gripper_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gripper_close-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gripper_close-response>))
  "Converts a ROS message object to a list"
  (cl:list 'gripper_close-response
    (cl:cons ':gripper_state (gripper_state msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'gripper_close)))
  'gripper_close-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'gripper_close)))
  'gripper_close-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gripper_close)))
  "Returns string type for a service object of type '<gripper_close>"
  "assist_feeding/gripper_close")