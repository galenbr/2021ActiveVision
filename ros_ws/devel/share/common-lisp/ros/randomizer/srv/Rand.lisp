; Auto-generated. Do not edit!


(cl:in-package randomizer-srv)


;//! \htmlinclude Rand-request.msg.html

(cl:defclass <Rand-request> (roslisp-msg-protocol:ros-message)
  ((toStart
    :reader toStart
    :initarg :toStart
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Rand-request (<Rand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Rand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Rand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name randomizer-srv:<Rand-request> is deprecated: use randomizer-srv:Rand-request instead.")))

(cl:ensure-generic-function 'toStart-val :lambda-list '(m))
(cl:defmethod toStart-val ((m <Rand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader randomizer-srv:toStart-val is deprecated.  Use randomizer-srv:toStart instead.")
  (toStart m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Rand-request>) ostream)
  "Serializes a message object of type '<Rand-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'toStart) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Rand-request>) istream)
  "Deserializes a message object of type '<Rand-request>"
    (cl:setf (cl:slot-value msg 'toStart) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Rand-request>)))
  "Returns string type for a service object of type '<Rand-request>"
  "randomizer/RandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Rand-request)))
  "Returns string type for a service object of type 'Rand-request"
  "randomizer/RandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Rand-request>)))
  "Returns md5sum for a message object of type '<Rand-request>"
  "cb0da9f8cfc2a14ca80d5a9fa236c5fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Rand-request)))
  "Returns md5sum for a message object of type 'Rand-request"
  "cb0da9f8cfc2a14ca80d5a9fa236c5fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Rand-request>)))
  "Returns full string definition for message of type '<Rand-request>"
  (cl:format cl:nil "bool toStart~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Rand-request)))
  "Returns full string definition for message of type 'Rand-request"
  (cl:format cl:nil "bool toStart~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Rand-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Rand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Rand-request
    (cl:cons ':toStart (toStart msg))
))
;//! \htmlinclude Rand-response.msg.html

(cl:defclass <Rand-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Rand-response (<Rand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Rand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Rand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name randomizer-srv:<Rand-response> is deprecated: use randomizer-srv:Rand-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Rand-response>) ostream)
  "Serializes a message object of type '<Rand-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Rand-response>) istream)
  "Deserializes a message object of type '<Rand-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Rand-response>)))
  "Returns string type for a service object of type '<Rand-response>"
  "randomizer/RandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Rand-response)))
  "Returns string type for a service object of type 'Rand-response"
  "randomizer/RandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Rand-response>)))
  "Returns md5sum for a message object of type '<Rand-response>"
  "cb0da9f8cfc2a14ca80d5a9fa236c5fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Rand-response)))
  "Returns md5sum for a message object of type 'Rand-response"
  "cb0da9f8cfc2a14ca80d5a9fa236c5fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Rand-response>)))
  "Returns full string definition for message of type '<Rand-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Rand-response)))
  "Returns full string definition for message of type 'Rand-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Rand-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Rand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Rand-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Rand)))
  'Rand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Rand)))
  'Rand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Rand)))
  "Returns string type for a service object of type '<Rand>"
  "randomizer/Rand")