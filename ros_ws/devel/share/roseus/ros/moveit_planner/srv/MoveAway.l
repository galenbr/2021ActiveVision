;; Auto-generated. Do not edit!


(when (boundp 'moveit_planner::MoveAway)
  (if (not (find-package "MOVEIT_PLANNER"))
    (make-package "MOVEIT_PLANNER"))
  (shadow 'MoveAway (find-package "MOVEIT_PLANNER")))
(unless (find-package "MOVEIT_PLANNER::MOVEAWAY")
  (make-package "MOVEIT_PLANNER::MOVEAWAY"))
(unless (find-package "MOVEIT_PLANNER::MOVEAWAYREQUEST")
  (make-package "MOVEIT_PLANNER::MOVEAWAYREQUEST"))
(unless (find-package "MOVEIT_PLANNER::MOVEAWAYRESPONSE")
  (make-package "MOVEIT_PLANNER::MOVEAWAYRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass moveit_planner::MoveAwayRequest
  :super ros::object
  :slots (_pose _distance _execute ))

(defmethod moveit_planner::MoveAwayRequest
  (:init
   (&key
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:distance __distance) 0.0)
    ((:execute __execute) nil)
    )
   (send-super :init)
   (setq _pose __pose)
   (setq _distance (float __distance))
   (setq _execute __execute)
   self)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:execute
   (&optional __execute)
   (if __execute (setq _execute __execute)) _execute)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; float64 _distance
    8
    ;; bool _execute
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; float64 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _execute
       (if _execute (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; float64 _distance
     (setq _distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _execute
     (setq _execute (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass moveit_planner::MoveAwayResponse
  :super ros::object
  :slots (_awayPose ))

(defmethod moveit_planner::MoveAwayResponse
  (:init
   (&key
    ((:awayPose __awayPose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _awayPose __awayPose)
   self)
  (:awayPose
   (&rest __awayPose)
   (if (keywordp (car __awayPose))
       (send* _awayPose __awayPose)
     (progn
       (if __awayPose (setq _awayPose (car __awayPose)))
       _awayPose)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _awayPose
    (send _awayPose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _awayPose
       (send _awayPose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _awayPose
     (send _awayPose :deserialize buf ptr-) (incf ptr- (send _awayPose :serialization-length))
   ;;
   self)
  )

(defclass moveit_planner::MoveAway
  :super ros::object
  :slots ())

(setf (get moveit_planner::MoveAway :md5sum-) "b0b7b3e5a88a68f810ee5cdc3adb57e7")
(setf (get moveit_planner::MoveAway :datatype-) "moveit_planner/MoveAway")
(setf (get moveit_planner::MoveAway :request) moveit_planner::MoveAwayRequest)
(setf (get moveit_planner::MoveAway :response) moveit_planner::MoveAwayResponse)

(defmethod moveit_planner::MoveAwayRequest
  (:response () (instance moveit_planner::MoveAwayResponse :init)))

(setf (get moveit_planner::MoveAwayRequest :md5sum-) "b0b7b3e5a88a68f810ee5cdc3adb57e7")
(setf (get moveit_planner::MoveAwayRequest :datatype-) "moveit_planner/MoveAwayRequest")
(setf (get moveit_planner::MoveAwayRequest :definition-)
      "geometry_msgs/Pose pose
float64 distance
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
geometry_msgs/Pose awayPose

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

(setf (get moveit_planner::MoveAwayResponse :md5sum-) "b0b7b3e5a88a68f810ee5cdc3adb57e7")
(setf (get moveit_planner::MoveAwayResponse :datatype-) "moveit_planner/MoveAwayResponse")
(setf (get moveit_planner::MoveAwayResponse :definition-)
      "geometry_msgs/Pose pose
float64 distance
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
geometry_msgs/Pose awayPose

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



(provide :moveit_planner/MoveAway "b0b7b3e5a88a68f810ee5cdc3adb57e7")


