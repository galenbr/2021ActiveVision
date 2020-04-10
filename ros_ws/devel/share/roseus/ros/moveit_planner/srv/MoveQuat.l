;; Auto-generated. Do not edit!


(when (boundp 'moveit_planner::MoveQuat)
  (if (not (find-package "MOVEIT_PLANNER"))
    (make-package "MOVEIT_PLANNER"))
  (shadow 'MoveQuat (find-package "MOVEIT_PLANNER")))
(unless (find-package "MOVEIT_PLANNER::MOVEQUAT")
  (make-package "MOVEIT_PLANNER::MOVEQUAT"))
(unless (find-package "MOVEIT_PLANNER::MOVEQUATREQUEST")
  (make-package "MOVEIT_PLANNER::MOVEQUATREQUEST"))
(unless (find-package "MOVEIT_PLANNER::MOVEQUATRESPONSE")
  (make-package "MOVEIT_PLANNER::MOVEQUATRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass moveit_planner::MoveQuatRequest
  :super ros::object
  :slots (_val _execute ))

(defmethod moveit_planner::MoveQuatRequest
  (:init
   (&key
    ((:val __val) (instance geometry_msgs::Quaternion :init))
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
    ;; geometry_msgs/Quaternion _val
    (send _val :serialization-length)
    ;; bool _execute
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Quaternion _val
       (send _val :serialize s)
     ;; bool _execute
       (if _execute (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Quaternion _val
     (send _val :deserialize buf ptr-) (incf ptr- (send _val :serialization-length))
   ;; bool _execute
     (setq _execute (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass moveit_planner::MoveQuatResponse
  :super ros::object
  :slots ())

(defmethod moveit_planner::MoveQuatResponse
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

(defclass moveit_planner::MoveQuat
  :super ros::object
  :slots ())

(setf (get moveit_planner::MoveQuat :md5sum-) "38b5b4ab2cb69ba284d4cfd4d2765acc")
(setf (get moveit_planner::MoveQuat :datatype-) "moveit_planner/MoveQuat")
(setf (get moveit_planner::MoveQuat :request) moveit_planner::MoveQuatRequest)
(setf (get moveit_planner::MoveQuat :response) moveit_planner::MoveQuatResponse)

(defmethod moveit_planner::MoveQuatRequest
  (:response () (instance moveit_planner::MoveQuatResponse :init)))

(setf (get moveit_planner::MoveQuatRequest :md5sum-) "38b5b4ab2cb69ba284d4cfd4d2765acc")
(setf (get moveit_planner::MoveQuatRequest :datatype-) "moveit_planner/MoveQuatRequest")
(setf (get moveit_planner::MoveQuatRequest :definition-)
      "geometry_msgs/Quaternion val
bool execute

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
")

(setf (get moveit_planner::MoveQuatResponse :md5sum-) "38b5b4ab2cb69ba284d4cfd4d2765acc")
(setf (get moveit_planner::MoveQuatResponse :datatype-) "moveit_planner/MoveQuatResponse")
(setf (get moveit_planner::MoveQuatResponse :definition-)
      "geometry_msgs/Quaternion val
bool execute

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
")



(provide :moveit_planner/MoveQuat "38b5b4ab2cb69ba284d4cfd4d2765acc")


