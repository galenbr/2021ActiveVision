; Auto-generated. Do not edit!


(cl:in-package moveit_planner-srv)


;//! \htmlinclude MoveJoint-request.msg.html

(cl:defclass <MoveJoint-request> (roslisp-msg-protocol:ros-message)
  ((val
    :reader val
    :initarg :val
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (execute
    :reader execute
    :initarg :execute
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MoveJoint-request (<MoveJoint-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveJoint-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveJoint-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moveit_planner-srv:<MoveJoint-request> is deprecated: use moveit_planner-srv:MoveJoint-request instead.")))

(cl:ensure-generic-function 'val-val :lambda-list '(m))
(cl:defmethod val-val ((m <MoveJoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moveit_planner-srv:val-val is deprecated.  Use moveit_planner-srv:val instead.")
  (val m))

(cl:ensure-generic-function 'execute-val :lambda-list '(m))
(cl:defmethod execute-val ((m <MoveJoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moveit_planner-srv:execute-val is deprecated.  Use moveit_planner-srv:execute instead.")
  (execute m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveJoint-request>) ostream)
  "Serializes a message object of type '<MoveJoint-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'val))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'val))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'execute) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveJoint-request>) istream)
  "Deserializes a message object of type '<MoveJoint-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'val) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'val)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:setf (cl:slot-value msg 'execute) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveJoint-request>)))
  "Returns string type for a service object of type '<MoveJoint-request>"
  "moveit_planner/MoveJointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveJoint-request)))
  "Returns string type for a service object of type 'MoveJoint-request"
  "moveit_planner/MoveJointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveJoint-request>)))
  "Returns md5sum for a message object of type '<MoveJoint-request>"
  "9c07c95fac99ad0dd277ffa3486c1adc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveJoint-request)))
  "Returns md5sum for a message object of type 'MoveJoint-request"
  "9c07c95fac99ad0dd277ffa3486c1adc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveJoint-request>)))
  "Returns full string definition for message of type '<MoveJoint-request>"
  (cl:format cl:nil "float64[] val~%bool execute~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveJoint-request)))
  "Returns full string definition for message of type 'MoveJoint-request"
  (cl:format cl:nil "float64[] val~%bool execute~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveJoint-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'val) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveJoint-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveJoint-request
    (cl:cons ':val (val msg))
    (cl:cons ':execute (execute msg))
))
;//! \htmlinclude MoveJoint-response.msg.html

(cl:defclass <MoveJoint-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MoveJoint-response (<MoveJoint-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveJoint-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveJoint-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moveit_planner-srv:<MoveJoint-response> is deprecated: use moveit_planner-srv:MoveJoint-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveJoint-response>) ostream)
  "Serializes a message object of type '<MoveJoint-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveJoint-response>) istream)
  "Deserializes a message object of type '<MoveJoint-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveJoint-response>)))
  "Returns string type for a service object of type '<MoveJoint-response>"
  "moveit_planner/MoveJointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveJoint-response)))
  "Returns string type for a service object of type 'MoveJoint-response"
  "moveit_planner/MoveJointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveJoint-response>)))
  "Returns md5sum for a message object of type '<MoveJoint-response>"
  "9c07c95fac99ad0dd277ffa3486c1adc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveJoint-response)))
  "Returns md5sum for a message object of type 'MoveJoint-response"
  "9c07c95fac99ad0dd277ffa3486c1adc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveJoint-response>)))
  "Returns full string definition for message of type '<MoveJoint-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveJoint-response)))
  "Returns full string definition for message of type 'MoveJoint-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveJoint-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveJoint-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveJoint-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveJoint)))
  'MoveJoint-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveJoint)))
  'MoveJoint-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveJoint)))
  "Returns string type for a service object of type '<MoveJoint>"
  "moveit_planner/MoveJoint")