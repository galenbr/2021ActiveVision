;; Auto-generated. Do not edit!


(when (boundp 'franka_gripper_gazebo::GripMsg)
  (if (not (find-package "FRANKA_GRIPPER_GAZEBO"))
    (make-package "FRANKA_GRIPPER_GAZEBO"))
  (shadow 'GripMsg (find-package "FRANKA_GRIPPER_GAZEBO")))
(unless (find-package "FRANKA_GRIPPER_GAZEBO::GRIPMSG")
  (make-package "FRANKA_GRIPPER_GAZEBO::GRIPMSG"))
(unless (find-package "FRANKA_GRIPPER_GAZEBO::GRIPMSGREQUEST")
  (make-package "FRANKA_GRIPPER_GAZEBO::GRIPMSGREQUEST"))
(unless (find-package "FRANKA_GRIPPER_GAZEBO::GRIPMSGRESPONSE")
  (make-package "FRANKA_GRIPPER_GAZEBO::GRIPMSGRESPONSE"))

(in-package "ROS")





(defclass franka_gripper_gazebo::GripMsgRequest
  :super ros::object
  :slots (_force ))

(defmethod franka_gripper_gazebo::GripMsgRequest
  (:init
   (&key
    ((:force __force) 0.0)
    )
   (send-super :init)
   (setq _force (float __force))
   self)
  (:force
   (&optional __force)
   (if __force (setq _force __force)) _force)
  (:serialization-length
   ()
   (+
    ;; float64 _force
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _force
       (sys::poke _force (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _force
     (setq _force (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass franka_gripper_gazebo::GripMsgResponse
  :super ros::object
  :slots ())

(defmethod franka_gripper_gazebo::GripMsgResponse
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

(defclass franka_gripper_gazebo::GripMsg
  :super ros::object
  :slots ())

(setf (get franka_gripper_gazebo::GripMsg :md5sum-) "e18a51329659ac6263f87aaf9a01fe62")
(setf (get franka_gripper_gazebo::GripMsg :datatype-) "franka_gripper_gazebo/GripMsg")
(setf (get franka_gripper_gazebo::GripMsg :request) franka_gripper_gazebo::GripMsgRequest)
(setf (get franka_gripper_gazebo::GripMsg :response) franka_gripper_gazebo::GripMsgResponse)

(defmethod franka_gripper_gazebo::GripMsgRequest
  (:response () (instance franka_gripper_gazebo::GripMsgResponse :init)))

(setf (get franka_gripper_gazebo::GripMsgRequest :md5sum-) "e18a51329659ac6263f87aaf9a01fe62")
(setf (get franka_gripper_gazebo::GripMsgRequest :datatype-) "franka_gripper_gazebo/GripMsgRequest")
(setf (get franka_gripper_gazebo::GripMsgRequest :definition-)
      "float64 force
---
")

(setf (get franka_gripper_gazebo::GripMsgResponse :md5sum-) "e18a51329659ac6263f87aaf9a01fe62")
(setf (get franka_gripper_gazebo::GripMsgResponse :datatype-) "franka_gripper_gazebo/GripMsgResponse")
(setf (get franka_gripper_gazebo::GripMsgResponse :definition-)
      "float64 force
---
")



(provide :franka_gripper_gazebo/GripMsg "e18a51329659ac6263f87aaf9a01fe62")


