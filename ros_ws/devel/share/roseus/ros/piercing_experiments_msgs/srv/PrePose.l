;; Auto-generated. Do not edit!


(when (boundp 'piercing_experiments_msgs::PrePose)
  (if (not (find-package "PIERCING_EXPERIMENTS_MSGS"))
    (make-package "PIERCING_EXPERIMENTS_MSGS"))
  (shadow 'PrePose (find-package "PIERCING_EXPERIMENTS_MSGS")))
(unless (find-package "PIERCING_EXPERIMENTS_MSGS::PREPOSE")
  (make-package "PIERCING_EXPERIMENTS_MSGS::PREPOSE"))
(unless (find-package "PIERCING_EXPERIMENTS_MSGS::PREPOSEREQUEST")
  (make-package "PIERCING_EXPERIMENTS_MSGS::PREPOSEREQUEST"))
(unless (find-package "PIERCING_EXPERIMENTS_MSGS::PREPOSERESPONSE")
  (make-package "PIERCING_EXPERIMENTS_MSGS::PREPOSERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass piercing_experiments_msgs::PrePoseRequest
  :super ros::object
  :slots (_target_1 ))

(defmethod piercing_experiments_msgs::PrePoseRequest
  (:init
   (&key
    ((:target_1 __target_1) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _target_1 __target_1)
   self)
  (:target_1
   (&rest __target_1)
   (if (keywordp (car __target_1))
       (send* _target_1 __target_1)
     (progn
       (if __target_1 (setq _target_1 (car __target_1)))
       _target_1)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _target_1
    (send _target_1 :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _target_1
       (send _target_1 :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _target_1
     (send _target_1 :deserialize buf ptr-) (incf ptr- (send _target_1 :serialization-length))
   ;;
   self)
  )

(defclass piercing_experiments_msgs::PrePoseResponse
  :super ros::object
  :slots ())

(defmethod piercing_experiments_msgs::PrePoseResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass piercing_experiments_msgs::PrePose
  :super ros::object
  :slots ())

(setf (get piercing_experiments_msgs::PrePose :md5sum-) "315b676814b67387d174953d2c38a6f1")
(setf (get piercing_experiments_msgs::PrePose :datatype-) "piercing_experiments_msgs/PrePose")
(setf (get piercing_experiments_msgs::PrePose :request) piercing_experiments_msgs::PrePoseRequest)
(setf (get piercing_experiments_msgs::PrePose :response) piercing_experiments_msgs::PrePoseResponse)

(defmethod piercing_experiments_msgs::PrePoseRequest
  (:response () (instance piercing_experiments_msgs::PrePoseResponse :init)))

(setf (get piercing_experiments_msgs::PrePoseRequest :md5sum-) "315b676814b67387d174953d2c38a6f1")
(setf (get piercing_experiments_msgs::PrePoseRequest :datatype-) "piercing_experiments_msgs/PrePoseRequest")
(setf (get piercing_experiments_msgs::PrePoseRequest :definition-)
      "geometry_msgs/Pose target_1

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
---
")

(setf (get piercing_experiments_msgs::PrePoseResponse :md5sum-) "315b676814b67387d174953d2c38a6f1")
(setf (get piercing_experiments_msgs::PrePoseResponse :datatype-) "piercing_experiments_msgs/PrePoseResponse")
(setf (get piercing_experiments_msgs::PrePoseResponse :definition-)
      "geometry_msgs/Pose target_1

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
---
")



(provide :piercing_experiments_msgs/PrePose "315b676814b67387d174953d2c38a6f1")


