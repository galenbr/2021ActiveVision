; Auto-generated. Do not edit!


(cl:in-package netft_utils-srv)


;//! \htmlinclude StopSim-request.msg.html

(cl:defclass <StopSim-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StopSim-request (<StopSim-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopSim-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopSim-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name netft_utils-srv:<StopSim-request> is deprecated: use netft_utils-srv:StopSim-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopSim-request>) ostream)
  "Serializes a message object of type '<StopSim-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopSim-request>) istream)
  "Deserializes a message object of type '<StopSim-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopSim-request>)))
  "Returns string type for a service object of type '<StopSim-request>"
  "netft_utils/StopSimRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopSim-request)))
  "Returns string type for a service object of type 'StopSim-request"
  "netft_utils/StopSimRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopSim-request>)))
  "Returns md5sum for a message object of type '<StopSim-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopSim-request)))
  "Returns md5sum for a message object of type 'StopSim-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopSim-request>)))
  "Returns full string definition for message of type '<StopSim-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopSim-request)))
  "Returns full string definition for message of type 'StopSim-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopSim-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopSim-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StopSim-request
))
;//! \htmlinclude StopSim-response.msg.html

(cl:defclass <StopSim-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StopSim-response (<StopSim-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopSim-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopSim-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name netft_utils-srv:<StopSim-response> is deprecated: use netft_utils-srv:StopSim-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopSim-response>) ostream)
  "Serializes a message object of type '<StopSim-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopSim-response>) istream)
  "Deserializes a message object of type '<StopSim-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopSim-response>)))
  "Returns string type for a service object of type '<StopSim-response>"
  "netft_utils/StopSimResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopSim-response)))
  "Returns string type for a service object of type 'StopSim-response"
  "netft_utils/StopSimResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopSim-response>)))
  "Returns md5sum for a message object of type '<StopSim-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopSim-response)))
  "Returns md5sum for a message object of type 'StopSim-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopSim-response>)))
  "Returns full string definition for message of type '<StopSim-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopSim-response)))
  "Returns full string definition for message of type 'StopSim-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopSim-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopSim-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StopSim-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StopSim)))
  'StopSim-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StopSim)))
  'StopSim-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopSim)))
  "Returns string type for a service object of type '<StopSim>"
  "netft_utils/StopSim")