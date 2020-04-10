;; Auto-generated. Do not edit!


(when (boundp 'moveit_planner::MovePose)
  (if (not (find-package "MOVEIT_PLANNER"))
    (make-package "MOVEIT_PLANNER"))
  (shadow 'MovePose (find-package "MOVEIT_PLANNER")))
(unless (find-package "MOVEIT_PLANNER::MOVEPOSE")
  (make-package "MOVEIT_PLANNER::MOVEPOSE"))
(unless (find-package "MOVEIT_PLANNER::MOVEPOSEREQUEST")
  (make-package "MOVEIT_PLANNER::MOVEPOSEREQUEST"))
(unless (find-package "MOVEIT_PLANNER::MOVEPOSERESPONSE")
  (make-package "MOVEIT_PLANNER::MOVEPOSERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass moveit_planner::MovePoseRequest
  :super ros::object
  :slots (_val _execute ))

(defmethod moveit_planner::MovePoseRequest
  (:init
   (&key
    ((:val __val) (instance geometry_msgs::Pose :init))
    ((:execute __execute) nil)
    )
   (send-super :init)
   (setq _val __val)
   (setq _execute __execute)
   self)
  (:val
   (&rest __val)
   (if (keywordp (car __val))
       (send* _val __val)
     (progn
       (if __val (setq _val (car __val)))
       _val)))
  (:execute
   (&optional __execute)
   (if __execute (setq _execute __execute)) _execute)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _val
    (send _val :serialization-length)
    ;; bool _execute
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _val
       (send _val :serialize s)
     ;; bool _execute
       (if _execute (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _val
     (send _val :deserialize buf ptr-) (incf ptr- (send _val :serialization-length))
   ;; bool _execute
     (setq _execute (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass moveit_planner::MovePoseResponse
  :super ros::object
  :slots ())

(defmethod moveit_planner::MovePoseResponse
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

(defclass moveit_planner::MovePose
  :super ros::object
  :slots ())

(setf (get moveit_planner::MovePose :md5sum-) "a77c545a10fa137b7b4acf08012288bf")
(setf (get moveit_planner::MovePose :datatype-) "moveit_planner/MovePose")
(setf (get moveit_planner::MovePose :request) moveit_planner::MovePoseRequest)
(setf (get moveit_planner::MovePose :response) moveit_planner::MovePoseResponse)

(defmethod moveit_planner::MovePoseRequest
  (:response () (instance moveit_planner::MovePoseResponse :init)))

(setf (get moveit_planner::MovePoseRequest :md5sum-) "a77c545a10fa137b7b4acf08012288bf")
(setf (get moveit_planner::MovePoseRequest :datatype-) "moveit_planner/MovePoseRequest")
(setf (get moveit_planner::MovePoseRequest :definition-)
      "geometry_msgs/Pose val
bool execute

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

(setf (get moveit_planner::MovePoseResponse :md5sum-) "a77c545a10fa137b7b4acf08012288bf")
(setf (get moveit_planner::MovePoseResponse :datatype-) "moveit_planner/MovePoseResponse")
(setf (get moveit_planner::MovePoseResponse :definition-)
      "geometry_msgs/Pose val
bool execute

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



(provide :moveit_planner/MovePose "a77c545a10fa137b7b4acf08012288bf")


