;; Auto-generated. Do not edit!


(when (boundp 'netft_utils::SetThreshold)
  (if (not (find-package "NETFT_UTILS"))
    (make-package "NETFT_UTILS"))
  (shadow 'SetThreshold (find-package "NETFT_UTILS")))
(unless (find-package "NETFT_UTILS::SETTHRESHOLD")
  (make-package "NETFT_UTILS::SETTHRESHOLD"))
(unless (find-package "NETFT_UTILS::SETTHRESHOLDREQUEST")
  (make-package "NETFT_UTILS::SETTHRESHOLDREQUEST"))
(unless (find-package "NETFT_UTILS::SETTHRESHOLDRESPONSE")
  (make-package "NETFT_UTILS::SETTHRESHOLDRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass netft_utils::SetThresholdRequest
  :super ros::object
  :slots (_data ))

(defmethod netft_utils::SetThresholdRequest
  (:init
   (&key
    ((:data __data) (instance geometry_msgs::WrenchStamped :init))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/WrenchStamped _data
    (send _data :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/WrenchStamped _data
       (send _data :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/WrenchStamped _data
     (send _data :deserialize buf ptr-) (incf ptr- (send _data :serialization-length))
   ;;
   self)
  )

(defclass netft_utils::SetThresholdResponse
  :super ros::object
  :slots (_success ))

(defmethod netft_utils::SetThresholdResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass netft_utils::SetThreshold
  :super ros::object
  :slots ())

(setf (get netft_utils::SetThreshold :md5sum-) "0a518cd2ef382108fafe506ec521a6c7")
(setf (get netft_utils::SetThreshold :datatype-) "netft_utils/SetThreshold")
(setf (get netft_utils::SetThreshold :request) netft_utils::SetThresholdRequest)
(setf (get netft_utils::SetThreshold :response) netft_utils::SetThresholdResponse)

(defmethod netft_utils::SetThresholdRequest
  (:response () (instance netft_utils::SetThresholdResponse :init)))

(setf (get netft_utils::SetThresholdRequest :md5sum-) "0a518cd2ef382108fafe506ec521a6c7")
(setf (get netft_utils::SetThresholdRequest :datatype-) "netft_utils/SetThresholdRequest")
(setf (get netft_utils::SetThresholdRequest :definition-)
      "geometry_msgs/WrenchStamped data

================================================================================
MSG: geometry_msgs/WrenchStamped
# A wrench with reference coordinate frame and timestamp
Header header
Wrench wrench

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
---
bool success

")

(setf (get netft_utils::SetThresholdResponse :md5sum-) "0a518cd2ef382108fafe506ec521a6c7")
(setf (get netft_utils::SetThresholdResponse :datatype-) "netft_utils/SetThresholdResponse")
(setf (get netft_utils::SetThresholdResponse :definition-)
      "geometry_msgs/WrenchStamped data

================================================================================
MSG: geometry_msgs/WrenchStamped
# A wrench with reference coordinate frame and timestamp
Header header
Wrench wrench

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
---
bool success

")



(provide :netft_utils/SetThreshold "0a518cd2ef382108fafe506ec521a6c7")


