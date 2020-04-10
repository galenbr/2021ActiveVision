;; Auto-generated. Do not edit!


(when (boundp 'netft_utils::SetToolData)
  (if (not (find-package "NETFT_UTILS"))
    (make-package "NETFT_UTILS"))
  (shadow 'SetToolData (find-package "NETFT_UTILS")))
(unless (find-package "NETFT_UTILS::SETTOOLDATA")
  (make-package "NETFT_UTILS::SETTOOLDATA"))
(unless (find-package "NETFT_UTILS::SETTOOLDATAREQUEST")
  (make-package "NETFT_UTILS::SETTOOLDATAREQUEST"))
(unless (find-package "NETFT_UTILS::SETTOOLDATARESPONSE")
  (make-package "NETFT_UTILS::SETTOOLDATARESPONSE"))

(in-package "ROS")





(defclass netft_utils::SetToolDataRequest
  :super ros::object
  :slots (_mass _COM ))

(defmethod netft_utils::SetToolDataRequest
  (:init
   (&key
    ((:mass __mass) 0.0)
    ((:COM __COM) 0.0)
    )
   (send-super :init)
   (setq _mass (float __mass))
   (setq _COM (float __COM))
   self)
  (:mass
   (&optional __mass)
   (if __mass (setq _mass __mass)) _mass)
  (:COM
   (&optional __COM)
   (if __COM (setq _COM __COM)) _COM)
  (:serialization-length
   ()
   (+
    ;; float64 _mass
    8
    ;; float64 _COM
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _mass
       (sys::poke _mass (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _COM
       (sys::poke _COM (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _mass
     (setq _mass (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _COM
     (setq _COM (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass netft_utils::SetToolDataResponse
  :super ros::object
  :slots (_success ))

(defmethod netft_utils::SetToolDataResponse
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

(defclass netft_utils::SetToolData
  :super ros::object
  :slots ())

(setf (get netft_utils::SetToolData :md5sum-) "d732082be6860b41d1ad5c3342683b63")
(setf (get netft_utils::SetToolData :datatype-) "netft_utils/SetToolData")
(setf (get netft_utils::SetToolData :request) netft_utils::SetToolDataRequest)
(setf (get netft_utils::SetToolData :response) netft_utils::SetToolDataResponse)

(defmethod netft_utils::SetToolDataRequest
  (:response () (instance netft_utils::SetToolDataResponse :init)))

(setf (get netft_utils::SetToolDataRequest :md5sum-) "d732082be6860b41d1ad5c3342683b63")
(setf (get netft_utils::SetToolDataRequest :datatype-) "netft_utils/SetToolDataRequest")
(setf (get netft_utils::SetToolDataRequest :definition-)
      "float64 mass
float64 COM
---
bool success

")

(setf (get netft_utils::SetToolDataResponse :md5sum-) "d732082be6860b41d1ad5c3342683b63")
(setf (get netft_utils::SetToolDataResponse :datatype-) "netft_utils/SetToolDataResponse")
(setf (get netft_utils::SetToolDataResponse :definition-)
      "float64 mass
float64 COM
---
bool success

")



(provide :netft_utils/SetToolData "d732082be6860b41d1ad5c3342683b63")


