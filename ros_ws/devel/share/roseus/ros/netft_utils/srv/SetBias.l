;; Auto-generated. Do not edit!


(when (boundp 'netft_utils::SetBias)
  (if (not (find-package "NETFT_UTILS"))
    (make-package "NETFT_UTILS"))
  (shadow 'SetBias (find-package "NETFT_UTILS")))
(unless (find-package "NETFT_UTILS::SETBIAS")
  (make-package "NETFT_UTILS::SETBIAS"))
(unless (find-package "NETFT_UTILS::SETBIASREQUEST")
  (make-package "NETFT_UTILS::SETBIASREQUEST"))
(unless (find-package "NETFT_UTILS::SETBIASRESPONSE")
  (make-package "NETFT_UTILS::SETBIASRESPONSE"))

(in-package "ROS")





(defclass netft_utils::SetBiasRequest
  :super ros::object
  :slots (_toBias _forceMax _torqueMax ))

(defmethod netft_utils::SetBiasRequest
  (:init
   (&key
    ((:toBias __toBias) nil)
    ((:forceMax __forceMax) 0.0)
    ((:torqueMax __torqueMax) 0.0)
    )
   (send-super :init)
   (setq _toBias __toBias)
   (setq _forceMax (float __forceMax))
   (setq _torqueMax (float __torqueMax))
   self)
  (:toBias
   (&optional __toBias)
   (if __toBias (setq _toBias __toBias)) _toBias)
  (:forceMax
   (&optional __forceMax)
   (if __forceMax (setq _forceMax __forceMax)) _forceMax)
  (:torqueMax
   (&optional __torqueMax)
   (if __torqueMax (setq _torqueMax __torqueMax)) _torqueMax)
  (:serialization-length
   ()
   (+
    ;; bool _toBias
    1
    ;; float64 _forceMax
    8
    ;; float64 _torqueMax
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _toBias
       (if _toBias (write-byte -1 s) (write-byte 0 s))
     ;; float64 _forceMax
       (sys::poke _forceMax (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _torqueMax
       (sys::poke _torqueMax (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _toBias
     (setq _toBias (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _forceMax
     (setq _forceMax (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _torqueMax
     (setq _torqueMax (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass netft_utils::SetBiasResponse
  :super ros::object
  :slots (_success ))

(defmethod netft_utils::SetBiasResponse
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

(defclass netft_utils::SetBias
  :super ros::object
  :slots ())

(setf (get netft_utils::SetBias :md5sum-) "24c559ab4eda1111c9526db57ba60c9c")
(setf (get netft_utils::SetBias :datatype-) "netft_utils/SetBias")
(setf (get netft_utils::SetBias :request) netft_utils::SetBiasRequest)
(setf (get netft_utils::SetBias :response) netft_utils::SetBiasResponse)

(defmethod netft_utils::SetBiasRequest
  (:response () (instance netft_utils::SetBiasResponse :init)))

(setf (get netft_utils::SetBiasRequest :md5sum-) "24c559ab4eda1111c9526db57ba60c9c")
(setf (get netft_utils::SetBiasRequest :datatype-) "netft_utils/SetBiasRequest")
(setf (get netft_utils::SetBiasRequest :definition-)
      "bool toBias
float64 forceMax
float64 torqueMax
---
bool success

")

(setf (get netft_utils::SetBiasResponse :md5sum-) "24c559ab4eda1111c9526db57ba60c9c")
(setf (get netft_utils::SetBiasResponse :datatype-) "netft_utils/SetBiasResponse")
(setf (get netft_utils::SetBiasResponse :definition-)
      "bool toBias
float64 forceMax
float64 torqueMax
---
bool success

")



(provide :netft_utils/SetBias "24c559ab4eda1111c9526db57ba60c9c")


