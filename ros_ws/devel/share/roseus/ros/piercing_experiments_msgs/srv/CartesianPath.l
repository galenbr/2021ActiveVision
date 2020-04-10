;; Auto-generated. Do not edit!


(when (boundp 'piercing_experiments_msgs::CartesianPath)
  (if (not (find-package "PIERCING_EXPERIMENTS_MSGS"))
    (make-package "PIERCING_EXPERIMENTS_MSGS"))
  (shadow 'CartesianPath (find-package "PIERCING_EXPERIMENTS_MSGS")))
(unless (find-package "PIERCING_EXPERIMENTS_MSGS::CARTESIANPATH")
  (make-package "PIERCING_EXPERIMENTS_MSGS::CARTESIANPATH"))
(unless (find-package "PIERCING_EXPERIMENTS_MSGS::CARTESIANPATHREQUEST")
  (make-package "PIERCING_EXPERIMENTS_MSGS::CARTESIANPATHREQUEST"))
(unless (find-package "PIERCING_EXPERIMENTS_MSGS::CARTESIANPATHRESPONSE")
  (make-package "PIERCING_EXPERIMENTS_MSGS::CARTESIANPATHRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass piercing_experiments_msgs::CartesianPathRequest
  :super ros::object
  :slots (_final_target _vel ))

(defmethod piercing_experiments_msgs::CartesianPathRequest
  (:init
   (&key
    ((:final_target __final_target) (instance geometry_msgs::Pose :init))
    ((:vel __vel) 0.0)
    )
   (send-super :init)
   (setq _final_target __final_target)
   (setq _vel (float __vel))
   self)
  (:final_target
   (&rest __final_target)
   (if (keywordp (car __final_target))
       (send* _final_target __final_target)
     (progn
       (if __final_target (setq _final_target (car __final_target)))
       _final_target)))
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _final_target
    (send _final_target :serialization-length)
    ;; float32 _vel
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _final_target
       (send _final_target :serialize s)
     ;; float32 _vel
       (sys::poke _vel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _final_target
     (send _final_target :deserialize buf ptr-) (incf ptr- (send _final_target :serialization-length))
   ;; float32 _vel
     (setq _vel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass piercing_experiments_msgs::CartesianPathResponse
  :super ros::object
  :slots ())

(defmethod piercing_experiments_msgs::CartesianPathResponse
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

(defclass piercing_experiments_msgs::CartesianPath
  :super ros::object
  :slots ())

(setf (get piercing_experiments_msgs::CartesianPath :md5sum-) "c01a67c204f217a13fc2a48450b5290b")
(setf (get piercing_experiments_msgs::CartesianPath :datatype-) "piercing_experiments_msgs/CartesianPath")
(setf (get piercing_experiments_msgs::CartesianPath :request) piercing_experiments_msgs::CartesianPathRequest)
(setf (get piercing_experiments_msgs::CartesianPath :response) piercing_experiments_msgs::CartesianPathResponse)

(defmethod piercing_experiments_msgs::CartesianPathRequest
  (:response () (instance piercing_experiments_msgs::CartesianPathResponse :init)))

(setf (get piercing_experiments_msgs::CartesianPathRequest :md5sum-) "c01a67c204f217a13fc2a48450b5290b")
(setf (get piercing_experiments_msgs::CartesianPathRequest :datatype-) "piercing_experiments_msgs/CartesianPathRequest")
(setf (get piercing_experiments_msgs::CartesianPathRequest :definition-)
      "geometry_msgs/Pose final_target
float32 vel

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

(setf (get piercing_experiments_msgs::CartesianPathResponse :md5sum-) "c01a67c204f217a13fc2a48450b5290b")
(setf (get piercing_experiments_msgs::CartesianPathResponse :datatype-) "piercing_experiments_msgs/CartesianPathResponse")
(setf (get piercing_experiments_msgs::CartesianPathResponse :definition-)
      "geometry_msgs/Pose final_target
float32 vel

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



(provide :piercing_experiments_msgs/CartesianPath "c01a67c204f217a13fc2a48450b5290b")


