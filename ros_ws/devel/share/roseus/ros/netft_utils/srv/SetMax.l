;; Auto-generated. Do not edit!


(when (boundp 'netft_utils::SetMax)
  (if (not (find-package "NETFT_UTILS"))
    (make-package "NETFT_UTILS"))
  (shadow 'SetMax (find-package "NETFT_UTILS")))
(unless (find-package "NETFT_UTILS::SETMAX")
  (make-package "NETFT_UTILS::SETMAX"))
(unless (find-package "NETFT_UTILS::SETMAXREQUEST")
  (make-package "NETFT_UTILS::SETMAXREQUEST"))
(unless (find-package "NETFT_UTILS::SETMAXRESPONSE")
  (make-package "NETFT_UTILS::SETMAXRESPONSE"))

(in-package "ROS")





(defclass netft_utils::SetMaxRequest
  :super ros::object
  :slots (_forceMax _torqueMax ))

(defmethod netft_utils::SetMaxRequest
  (:init
   (&key
    ((:forceMax __forceMax) 0.0)
    ((:torqueMax __torqueMax) 0.0)
    )
   (send-super :init)
   (setq _forceMax (float __forceMax))
   (setq _torqueMax (float __torqueMax))
   self)
  (:forceMax
   (&optional __forceMax)
   (if __forceMax (setq _forceMax __forceMax)) _forceMax)
  (:torqueMax
   (&optional __torqueMax)
   (if __torqueMax (setq _torqueMax __torqueMax)) _torqueMax)
  (:serialization-length
   ()
   (+
    ;; float64 _forceMax
    8
    ;; float64 _torqueMax
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _forceMax
       (sys::poke _forceMax (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _torqueMax
       (sys::poke _torqueMax (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _forceMax
     (setq _forceMax (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _torqueMax
     (setq _torqueMax (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass netft_utils::SetMaxResponse
  :super ros::object
  :slots (_success ))

(defmethod netft_utils::SetMaxResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass netft_utils::SetMax
  :super ros::object
  :slots ())

(setf (get netft_utils::SetMax :md5sum-) "10e2f2a95c7447ec4aaa55f68d560d78")
(setf (get netft_utils::SetMax :datatype-) "netft_utils/SetMax")
(setf (get netft_utils::SetMax :request) netft_utils::SetMaxRequest)
(setf (get netft_utils::SetMax :response) netft_utils::SetMaxResponse)

(defmethod netft_utils::SetMaxRequest
  (:response () (instance netft_utils::SetMaxResponse :init)))

(setf (get netft_utils::SetMaxRequest :md5sum-) "10e2f2a95c7447ec4aaa55f68d560d78")
(setf (get netft_utils::SetMaxRequest :datatype-) "netft_utils/SetMaxRequest")
(setf (get netft_utils::SetMaxRequest :definition-)
      "float64 forceMax
float64 torqueMax
---
bool success

")

(setf (get netft_utils::SetMaxResponse :md5sum-) "10e2f2a95c7447ec4aaa55f68d560d78")
(setf (get netft_utils::SetMaxResponse :datatype-) "netft_utils/SetMaxResponse")
(setf (get netft_utils::SetMaxResponse :definition-)
      "float64 forceMax
float64 torqueMax
---
bool success

")



(provide :netft_utils/SetMax "10e2f2a95c7447ec4aaa55f68d560d78")


