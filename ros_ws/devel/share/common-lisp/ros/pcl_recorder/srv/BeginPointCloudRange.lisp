; Auto-generated. Do not edit!


(cl:in-package pcl_recorder-srv)


;//! \htmlinclude BeginPointCloudRange-request.msg.html

(cl:defclass <BeginPointCloudRange-request> (roslisp-msg-protocol:ros-message)
  ((frequency
    :reader frequency
    :initarg :frequency
    :type cl:float
    :initform 0.0))
)

(cl:defclass BeginPointCloudRange-request (<BeginPointCloudRange-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BeginPointCloudRange-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BeginPointCloudRange-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pcl_recorder-srv:<BeginPointCloudRange-request> is deprecated: use pcl_recorder-srv:BeginPointCloudRange-request instead.")))

(cl:ensure-generic-function 'frequency-val :lambda-list '(m))
(cl:defmethod frequency-val ((m <BeginPointCloudRange-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pcl_recorder-srv:frequency-val is deprecated.  Use pcl_recorder-srv:frequency instead.")
  (frequency m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BeginPointCloudRange-request>) ostream)
  "Serializes a message object of type '<BeginPointCloudRange-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'frequency))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BeginPointCloudRange-request>) istream)
  "Deserializes a message object of type '<BeginPointCloudRange-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'frequency) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BeginPointCloudRange-request>)))
  "Returns string type for a service object of type '<BeginPointCloudRange-request>"
  "pcl_recorder/BeginPointCloudRangeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BeginPointCloudRange-request)))
  "Returns string type for a service object of type 'BeginPointCloudRange-request"
  "pcl_recorder/BeginPointCloudRangeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BeginPointCloudRange-request>)))
  "Returns md5sum for a message object of type '<BeginPointCloudRange-request>"
  "10432e08567eb968c3444f0fa65097b5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BeginPointCloudRange-request)))
  "Returns md5sum for a message object of type 'BeginPointCloudRange-request"
  "10432e08567eb968c3444f0fa65097b5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BeginPointCloudRange-request>)))
  "Returns full string definition for message of type '<BeginPointCloudRange-request>"
  (cl:format cl:nil "float64 frequency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BeginPointCloudRange-request)))
  "Returns full string definition for message of type 'BeginPointCloudRange-request"
  (cl:format cl:nil "float64 frequency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BeginPointCloudRange-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BeginPointCloudRange-request>))
  "Converts a ROS message object to a list"
  (cl:list 'BeginPointCloudRange-request
    (cl:cons ':frequency (frequency msg))
))
;//! \htmlinclude BeginPointCloudRange-response.msg.html

(cl:defclass <BeginPointCloudRange-response> (roslisp-msg-protocol:ros-message)
  ((limitSeconds
    :reader limitSeconds
    :initarg :limitSeconds
    :type cl:float
    :initform 0.0))
)

(cl:defclass BeginPointCloudRange-response (<BeginPointCloudRange-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BeginPointCloudRange-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BeginPointCloudRange-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pcl_recorder-srv:<BeginPointCloudRange-response> is deprecated: use pcl_recorder-srv:BeginPointCloudRange-response instead.")))

(cl:ensure-generic-function 'limitSeconds-val :lambda-list '(m))
(cl:defmethod limitSeconds-val ((m <BeginPointCloudRange-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pcl_recorder-srv:limitSeconds-val is deprecated.  Use pcl_recorder-srv:limitSeconds instead.")
  (limitSeconds m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BeginPointCloudRange-response>) ostream)
  "Serializes a message object of type '<BeginPointCloudRange-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'limitSeconds))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BeginPointCloudRange-response>) istream)
  "Deserializes a message object of type '<BeginPointCloudRange-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'limitSeconds) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BeginPointCloudRange-response>)))
  "Returns string type for a service object of type '<BeginPointCloudRange-response>"
  "pcl_recorder/BeginPointCloudRangeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BeginPointCloudRange-response)))
  "Returns string type for a service object of type 'BeginPointCloudRange-response"
  "pcl_recorder/BeginPointCloudRangeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BeginPointCloudRange-response>)))
  "Returns md5sum for a message object of type '<BeginPointCloudRange-response>"
  "10432e08567eb968c3444f0fa65097b5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BeginPointCloudRange-response)))
  "Returns md5sum for a message object of type 'BeginPointCloudRange-response"
  "10432e08567eb968c3444f0fa65097b5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BeginPointCloudRange-response>)))
  "Returns full string definition for message of type '<BeginPointCloudRange-response>"
  (cl:format cl:nil "float64 limitSeconds~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BeginPointCloudRange-response)))
  "Returns full string definition for message of type 'BeginPointCloudRange-response"
  (cl:format cl:nil "float64 limitSeconds~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BeginPointCloudRange-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BeginPointCloudRange-response>))
  "Converts a ROS message object to a list"
  (cl:list 'BeginPointCloudRange-response
    (cl:cons ':limitSeconds (limitSeconds msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'BeginPointCloudRange)))
  'BeginPointCloudRange-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'BeginPointCloudRange)))
  'BeginPointCloudRange-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BeginPointCloudRange)))
  "Returns string type for a service object of type '<BeginPointCloudRange>"
  "pcl_recorder/BeginPointCloudRange")