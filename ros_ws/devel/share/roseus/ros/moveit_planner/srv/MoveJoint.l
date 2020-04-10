;; Auto-generated. Do not edit!


(when (boundp 'moveit_planner::MoveJoint)
  (if (not (find-package "MOVEIT_PLANNER"))
    (make-package "MOVEIT_PLANNER"))
  (shadow 'MoveJoint (find-package "MOVEIT_PLANNER")))
(unless (find-package "MOVEIT_PLANNER::MOVEJOINT")
  (make-package "MOVEIT_PLANNER::MOVEJOINT"))
(unless (find-package "MOVEIT_PLANNER::MOVEJOINTREQUEST")
  (make-package "MOVEIT_PLANNER::MOVEJOINTREQUEST"))
(unless (find-package "MOVEIT_PLANNER::MOVEJOINTRESPONSE")
  (make-package "MOVEIT_PLANNER::MOVEJOINTRESPONSE"))

(in-package "ROS")





(defclass moveit_planner::MoveJointRequest
  :super ros::object
  :slots (_val _execute ))

(defmethod moveit_planner::MoveJointRequest
  (:init
   (&key
    ((:val __val) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:execute __execute) nil)
    )
   (send-super :init)
   (setq _val __val)
   (setq _execute __execute)
   self)
  (:val
   (&optional __val)
   (if __val (setq _val __val)) _val)
  (:execute
   (&optional __execute)
   (if __execute (setq _execute __execute)) _execute)
  (:serialization-length
   ()
   (+
    ;; float64[] _val
    (* 8    (length _val)) 4
    ;; bool _execute
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _val
     (write-long (length _val) s)
     (dotimes (i (length _val))
       (sys::poke (elt _val i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _execute
       (if _execute (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _val
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _val (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _val i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; bool _execute
     (setq _execute (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass moveit_planner::MoveJointResponse
  :super ros::object
  :slots ())

(defmethod moveit_planner::MoveJointResponse
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

(defclass moveit_planner::MoveJoint
  :super ros::object
  :slots ())

(setf (get moveit_planner::MoveJoint :md5sum-) "9c07c95fac99ad0dd277ffa3486c1adc")
(setf (get moveit_planner::MoveJoint :datatype-) "moveit_planner/MoveJoint")
(setf (get moveit_planner::MoveJoint :request) moveit_planner::MoveJointRequest)
(setf (get moveit_planner::MoveJoint :response) moveit_planner::MoveJointResponse)

(defmethod moveit_planner::MoveJointRequest
  (:response () (instance moveit_planner::MoveJointResponse :init)))

(setf (get moveit_planner::MoveJointRequest :md5sum-) "9c07c95fac99ad0dd277ffa3486c1adc")
(setf (get moveit_planner::MoveJointRequest :datatype-) "moveit_planner/MoveJointRequest")
(setf (get moveit_planner::MoveJointRequest :definition-)
      "float64[] val
bool execute
---

")

(setf (get moveit_planner::MoveJointResponse :md5sum-) "9c07c95fac99ad0dd277ffa3486c1adc")
(setf (get moveit_planner::MoveJointResponse :datatype-) "moveit_planner/MoveJointResponse")
(setf (get moveit_planner::MoveJointResponse :definition-)
      "float64[] val
bool execute
---

")



(provide :moveit_planner/MoveJoint "9c07c95fac99ad0dd277ffa3486c1adc")


