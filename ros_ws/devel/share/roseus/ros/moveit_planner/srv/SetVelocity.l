;; Auto-generated. Do not edit!


(when (boundp 'moveit_planner::SetVelocity)
  (if (not (find-package "MOVEIT_PLANNER"))
    (make-package "MOVEIT_PLANNER"))
  (shadow 'SetVelocity (find-package "MOVEIT_PLANNER")))
(unless (find-package "MOVEIT_PLANNER::SETVELOCITY")
  (make-package "MOVEIT_PLANNER::SETVELOCITY"))
(unless (find-package "MOVEIT_PLANNER::SETVELOCITYREQUEST")
  (make-package "MOVEIT_PLANNER::SETVELOCITYREQUEST"))
(unless (find-package "MOVEIT_PLANNER::SETVELOCITYRESPONSE")
  (make-package "MOVEIT_PLANNER::SETVELOCITYRESPONSE"))

(in-package "ROS")





(defclass moveit_planner::SetVelocityRequest
  :super ros::object
  :slots (_velScaling ))

(defmethod moveit_planner::SetVelocityRequest
  (:init
   (&key
    ((:velScaling __velScaling) 0.0)
    )
   (send-super :init)
   (setq _velScaling (float __velScaling))
   self)
  (:velScaling
   (&optional __velScaling)
   (if __velScaling (setq _velScaling __velScaling)) _velScaling)
  (:serialization-length
   ()
   (+
    ;; float64 _velScaling
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _velScaling
       (sys::poke _velScaling (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _velScaling
     (setq _velScaling (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass moveit_planner::SetVelocityResponse
  :super ros::object
  :slots ())

(defmethod moveit_planner::SetVelocityResponse
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

(defclass moveit_planner::SetVelocity
  :super ros::object
  :slots ())

(setf (get moveit_planner::SetVelocity :md5sum-) "a61a7a7956fba028d52c35ae7e4e38a7")
(setf (get moveit_planner::SetVelocity :datatype-) "moveit_planner/SetVelocity")
(setf (get moveit_planner::SetVelocity :request) moveit_planner::SetVelocityRequest)
(setf (get moveit_planner::SetVelocity :response) moveit_planner::SetVelocityResponse)

(defmethod moveit_planner::SetVelocityRequest
  (:response () (instance moveit_planner::SetVelocityResponse :init)))

(setf (get moveit_planner::SetVelocityRequest :md5sum-) "a61a7a7956fba028d52c35ae7e4e38a7")
(setf (get moveit_planner::SetVelocityRequest :datatype-) "moveit_planner/SetVelocityRequest")
(setf (get moveit_planner::SetVelocityRequest :definition-)
      "float64 velScaling
---
")

(setf (get moveit_planner::SetVelocityResponse :md5sum-) "a61a7a7956fba028d52c35ae7e4e38a7")
(setf (get moveit_planner::SetVelocityResponse :datatype-) "moveit_planner/SetVelocityResponse")
(setf (get moveit_planner::SetVelocityResponse :definition-)
      "float64 velScaling
---
")



(provide :moveit_planner/SetVelocity "a61a7a7956fba028d52c35ae7e4e38a7")


