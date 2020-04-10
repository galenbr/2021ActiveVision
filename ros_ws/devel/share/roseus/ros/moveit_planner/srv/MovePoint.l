;; Auto-generated. Do not edit!


(when (boundp 'moveit_planner::MovePoint)
  (if (not (find-package "MOVEIT_PLANNER"))
    (make-package "MOVEIT_PLANNER"))
  (shadow 'MovePoint (find-package "MOVEIT_PLANNER")))
(unless (find-package "MOVEIT_PLANNER::MOVEPOINT")
  (make-package "MOVEIT_PLANNER::MOVEPOINT"))
(unless (find-package "MOVEIT_PLANNER::MOVEPOINTREQUEST")
  (make-package "MOVEIT_PLANNER::MOVEPOINTREQUEST"))
(unless (find-package "MOVEIT_PLANNER::MOVEPOINTRESPONSE")
  (make-package "MOVEIT_PLANNER::MOVEPOINTRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass moveit_planner::MovePointRequest
  :super ros::object
  :slots (_val _execute ))

(defmethod moveit_planner::MovePointRequest
  (:init
   (&key
    ((:val __val) (instance geometry_msgs::Point :init))
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
    ;; geometry_msgs/Point _val
    (send _val :serialization-length)
    ;; bool _execute
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _val
       (send _val :serialize s)
     ;; bool _execute
       (if _execute (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _val
     (send _val :deserialize buf ptr-) (incf ptr- (send _val :serialization-length))
   ;; bool _execute
     (setq _execute (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass moveit_planner::MovePointResponse
  :super ros::object
  :slots ())

(defmethod moveit_planner::MovePointResponse
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

(defclass moveit_planner::MovePoint
  :super ros::object
  :slots ())

(setf (get moveit_planner::MovePoint :md5sum-) "09d178cfe7c05e5d8c3ba0747602407a")
(setf (get moveit_planner::MovePoint :datatype-) "moveit_planner/MovePoint")
(setf (get moveit_planner::MovePoint :request) moveit_planner::MovePointRequest)
(setf (get moveit_planner::MovePoint :response) moveit_planner::MovePointResponse)

(defmethod moveit_planner::MovePointRequest
  (:response () (instance moveit_planner::MovePointResponse :init)))

(setf (get moveit_planner::MovePointRequest :md5sum-) "09d178cfe7c05e5d8c3ba0747602407a")
(setf (get moveit_planner::MovePointRequest :datatype-) "moveit_planner/MovePointRequest")
(setf (get moveit_planner::MovePointRequest :definition-)
      "geometry_msgs/Point val
bool execute

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
")

(setf (get moveit_planner::MovePointResponse :md5sum-) "09d178cfe7c05e5d8c3ba0747602407a")
(setf (get moveit_planner::MovePointResponse :datatype-) "moveit_planner/MovePointResponse")
(setf (get moveit_planner::MovePointResponse :definition-)
      "geometry_msgs/Point val
bool execute

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
")



(provide :moveit_planner/MovePoint "09d178cfe7c05e5d8c3ba0747602407a")


