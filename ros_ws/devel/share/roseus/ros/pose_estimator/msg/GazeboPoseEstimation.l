;; Auto-generated. Do not edit!


(when (boundp 'pose_estimator::GazeboPoseEstimation)
  (if (not (find-package "POSE_ESTIMATOR"))
    (make-package "POSE_ESTIMATOR"))
  (shadow 'GazeboPoseEstimation (find-package "POSE_ESTIMATOR")))
(unless (find-package "POSE_ESTIMATOR::GAZEBOPOSEESTIMATION")
  (make-package "POSE_ESTIMATOR::GAZEBOPOSEESTIMATION"))

(in-package "ROS")
;;//! \htmlinclude GazeboPoseEstimation.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pose_estimator::GazeboPoseEstimation
  :super ros::object
  :slots (_header _model_names _model_poses ))

(defmethod pose_estimator::GazeboPoseEstimation
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:model_names __model_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:model_poses __model_poses) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Pose :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _model_names __model_names)
   (setq _model_poses __model_poses)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:model_names
   (&optional __model_names)
   (if __model_names (setq _model_names __model_names)) _model_names)
  (:model_poses
   (&rest __model_poses)
   (if (keywordp (car __model_poses))
       (send* _model_poses __model_poses)
     (progn
       (if __model_poses (setq _model_poses (car __model_poses)))
       _model_poses)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string[] _model_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _model_names)) 4
    ;; geometry_msgs/Pose[] _model_poses
    (apply #'+ (send-all _model_poses :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string[] _model_names
     (write-long (length _model_names) s)
     (dolist (elem _model_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; geometry_msgs/Pose[] _model_poses
     (write-long (length _model_poses) s)
     (dolist (elem _model_poses)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string[] _model_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _model_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _model_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; geometry_msgs/Pose[] _model_poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _model_poses (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _model_poses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get pose_estimator::GazeboPoseEstimation :md5sum-) "3bbf8867409277371ff3ab48c9274b56")
(setf (get pose_estimator::GazeboPoseEstimation :datatype-) "pose_estimator/GazeboPoseEstimation")
(setf (get pose_estimator::GazeboPoseEstimation :definition-)
      "std_msgs/Header header
string[] model_names
geometry_msgs/Pose[] model_poses
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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :pose_estimator/GazeboPoseEstimation "3bbf8867409277371ff3ab48c9274b56")


