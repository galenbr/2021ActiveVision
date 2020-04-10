; Auto-generated. Do not edit!


(cl:in-package netft_utils-msg)


;//! \htmlinclude Cancel.msg.html

(cl:defclass <Cancel> (roslisp-msg-protocol:ros-message)
  ((toCancel
    :reader toCancel
    :initarg :toCancel
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Cancel (<Cancel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Cancel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Cancel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name netft_utils-msg:<Cancel> is deprecated: use netft_utils-msg:Cancel instead.")))

(cl:ensure-generic-function 'toCancel-val :lambda-list '(m))
(cl:defmethod toCancel-val ((m <Cancel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader netft_utils-msg:toCancel-val is deprecated.  Use netft_utils-msg:toCancel instead.")
  (toCancel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Cancel>) ostream)
  "Serializes a message object of type '<Cancel>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'toCancel) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Cancel>) istream)
  "Deserializes a message object of type '<Cancel>"
    (cl:setf (cl:slot-value msg 'toCancel) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Cancel>)))
  "Returns string type for a message object of type '<Cancel>"
  "netft_utils/Cancel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Cancel)))
  "Returns string type for a message object of type 'Cancel"
  "netft_utils/Cancel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Cancel>)))
  "Returns md5sum for a message object of type '<Cancel>"
  "60c8bbbce9a3c19aed52c3a96b5d87ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Cancel)))
  "Returns md5sum for a message object of type 'Cancel"
  "60c8bbbce9a3c19aed52c3a96b5d87ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Cancel>)))
  "Returns full string definition for message of type '<Cancel>"
  (cl:format cl:nil "bool toCancel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Cancel)))
  "Returns full string definition for message of type 'Cancel"
  (cl:format cl:nil "bool toCancel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Cancel>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Cancel>))
  "Converts a ROS message object to a list"
  (cl:list 'Cancel
    (cl:cons ':toCancel (toCancel msg))
))
