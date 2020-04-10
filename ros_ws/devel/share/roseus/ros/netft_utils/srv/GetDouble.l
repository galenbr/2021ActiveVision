;; Auto-generated. Do not edit!


(when (boundp 'netft_utils::GetDouble)
  (if (not (find-package "NETFT_UTILS"))
    (make-package "NETFT_UTILS"))
  (shadow 'GetDouble (find-package "NETFT_UTILS")))
(unless (find-package "NETFT_UTILS::GETDOUBLE")
  (make-package "NETFT_UTILS::GETDOUBLE"))
(unless (find-package "NETFT_UTILS::GETDOUBLEREQUEST")
  (make-package "NETFT_UTILS::GETDOUBLEREQUEST"))
(unless (find-package "NETFT_UTILS::GETDOUBLERESPONSE")
  (make-package "NETFT_UTILS::GETDOUBLERESPONSE"))

(in-package "ROS")





(defclass netft_utils::GetDoubleRequest
  :super ros::object
  :slots (_ignore ))

(defmethod netft_utils::GetDoubleRequest
  (:init
   (&key
    ((:ignore __ignore) nil)
    )
   (send-super :init)
   (setq _ignore __ignore)
   self)
  (:ignore
   (&optional __ignore)
   (if __ignore (setq _ignore __ignore)) _ignore)
  (:serialization-length
   ()
   (+
    ;; bool _ignore
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ignore
       (if _ignore (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ignore
     (setq _ignore (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass netft_utils::GetDoubleResponse
  :super ros::object
  :slots (_weight ))

(defmethod netft_utils::GetDoubleResponse
  (:init
   (&key
    ((:weight __weight) 0.0)
    )
   (send-super :init)
   (setq _weight (float __weight))
   self)
  (:weight
   (&optional __weight)
   (if __weight (setq _weight __weight)) _weight)
  (:serialization-length
   ()
   (+
    ;; float64 _weight
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _weight
       (sys::poke _weight (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _weight
     (setq _weight (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass netft_utils::GetDouble
  :super ros::object
  :slots ())

(setf (get netft_utils::GetDouble :md5sum-) "43623635c0100a1d66762de2a2c1a2da")
(setf (get netft_utils::GetDouble :datatype-) "netft_utils/GetDouble")
(setf (get netft_utils::GetDouble :request) netft_utils::GetDoubleRequest)
(setf (get netft_utils::GetDouble :response) netft_utils::GetDoubleResponse)

(defmethod netft_utils::GetDoubleRequest
  (:response () (instance netft_utils::GetDoubleResponse :init)))

(setf (get netft_utils::GetDoubleRequest :md5sum-) "43623635c0100a1d66762de2a2c1a2da")
(setf (get netft_utils::GetDoubleRequest :datatype-) "netft_utils/GetDoubleRequest")
(setf (get netft_utils::GetDoubleRequest :definition-)
      "bool ignore
---
float64 weight

")

(setf (get netft_utils::GetDoubleResponse :md5sum-) "43623635c0100a1d66762de2a2c1a2da")
(setf (get netft_utils::GetDoubleResponse :datatype-) "netft_utils/GetDoubleResponse")
(setf (get netft_utils::GetDoubleResponse :definition-)
      "bool ignore
---
float64 weight

")



(provide :netft_utils/GetDouble "43623635c0100a1d66762de2a2c1a2da")


