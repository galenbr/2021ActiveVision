;; Auto-generated. Do not edit!


(when (boundp 'netft_utils::StopSim)
  (if (not (find-package "NETFT_UTILS"))
    (make-package "NETFT_UTILS"))
  (shadow 'StopSim (find-package "NETFT_UTILS")))
(unless (find-package "NETFT_UTILS::STOPSIM")
  (make-package "NETFT_UTILS::STOPSIM"))
(unless (find-package "NETFT_UTILS::STOPSIMREQUEST")
  (make-package "NETFT_UTILS::STOPSIMREQUEST"))
(unless (find-package "NETFT_UTILS::STOPSIMRESPONSE")
  (make-package "NETFT_UTILS::STOPSIMRESPONSE"))

(in-package "ROS")





(defclass netft_utils::StopSimRequest
  :super ros::object
  :slots ())

(defmethod netft_utils::StopSimRequest
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

(defclass netft_utils::StopSimResponse
  :super ros::object
  :slots ())

(defmethod netft_utils::StopSimResponse
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

(defclass netft_utils::StopSim
  :super ros::object
  :slots ())

(setf (get netft_utils::StopSim :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get netft_utils::StopSim :datatype-) "netft_utils/StopSim")
(setf (get netft_utils::StopSim :request) netft_utils::StopSimRequest)
(setf (get netft_utils::StopSim :response) netft_utils::StopSimResponse)

(defmethod netft_utils::StopSimRequest
  (:response () (instance netft_utils::StopSimResponse :init)))

(setf (get netft_utils::StopSimRequest :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get netft_utils::StopSimRequest :datatype-) "netft_utils/StopSimRequest")
(setf (get netft_utils::StopSimRequest :definition-)
      "
---


")

(setf (get netft_utils::StopSimResponse :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get netft_utils::StopSimResponse :datatype-) "netft_utils/StopSimResponse")
(setf (get netft_utils::StopSimResponse :definition-)
      "
---


")



(provide :netft_utils/StopSim "d41d8cd98f00b204e9800998ecf8427e")


