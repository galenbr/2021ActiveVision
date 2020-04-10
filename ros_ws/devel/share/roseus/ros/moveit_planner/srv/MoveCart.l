;; Auto-generated. Do not edit!


(when (boundp 'moveit_planner::MoveCart)
  (if (not (find-package "MOVEIT_PLANNER"))
    (make-package "MOVEIT_PLANNER"))
  (shadow 'MoveCart (find-package "MOVEIT_PLANNER")))
(unless (find-package "MOVEIT_PLANNER::MOVECART")
  (make-package "MOVEIT_PLANNER::MOVECART"))
(unless (find-package "MOVEIT_PLANNER::MOVECARTREQUEST")
  (make-package "MOVEIT_PLANNER::MOVECARTREQUEST"))
(unless (find-package "MOVEIT_PLANNER::MOVECARTRESPONSE")
  (make-package "MOVEIT_PLANNER::MOVECARTRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass moveit_planner::MoveCartRequest
  :super ros::object
  :slots (_val _execute ))

(defmethod moveit_planner::MoveCartRequest
  (:init
   (&key
    ((:val __val) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Pose :init) r)) r))
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
    ;; geometry_msgs/Pose[] _val
    (apply #'+ (send-all _val :serialization-length)) 4
    ;; bool _execute
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose[] _val
     (write-long (length _val) s)
     (dolist (elem _val)
       (send elem :serialize s)
       )
     ;; bool _execute
       (if _execute (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose[] _val
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _val (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _val)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool _execute
     (setq _execute (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass moveit_planner::MoveCartResponse
  :super ros::object
  :slots ())

(defmethod moveit_planner::MoveCartResponse
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

(defclass moveit_planner::MoveCart
  :super ros::object
  :slots ())

(setf (get moveit_planner::MoveCart :md5sum-) "a77c545a10fa137b7b4acf08012288bf")
(setf (get moveit_planner::MoveCart :datatype-) "moveit_planner/MoveCart")
(setf (get moveit_planner::MoveCart :request) moveit_planner::MoveCartRequest)
(setf (get moveit_planner::MoveCart :response) moveit_planner::MoveCartResponse)

(defmethod moveit_planner::MoveCartRequest
  (:response () (instance moveit_planner::MoveCartResponse :init)))

(setf (get moveit_planner::MoveCartRequest :md5sum-) "a77c545a10fa137b7b4acf08012288bf")
(setf (get moveit_planner::MoveCartRequest :datatype-) "moveit_planner/MoveCartRequest")
(setf (get moveit_planner::MoveCartRequest :definition-)
      "geometry_msgs/Pose[] val
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

(setf (get moveit_planner::MoveCartResponse :md5sum-) "a77c545a10fa137b7b4acf08012288bf")
(setf (get moveit_planner::MoveCartResponse :datatype-) "moveit_planner/MoveCartResponse")
(setf (get moveit_planner::MoveCartResponse :definition-)
      "geometry_msgs/Pose[] val
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



(provide :moveit_planner/MoveCart "a77c545a10fa137b7b4acf08012288bf")


