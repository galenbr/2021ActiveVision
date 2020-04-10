;; Auto-generated. Do not edit!


(when (boundp 'netft_utils::SetFilter)
  (if (not (find-package "NETFT_UTILS"))
    (make-package "NETFT_UTILS"))
  (shadow 'SetFilter (find-package "NETFT_UTILS")))
(unless (find-package "NETFT_UTILS::SETFILTER")
  (make-package "NETFT_UTILS::SETFILTER"))
(unless (find-package "NETFT_UTILS::SETFILTERREQUEST")
  (make-package "NETFT_UTILS::SETFILTERREQUEST"))
(unless (find-package "NETFT_UTILS::SETFILTERRESPONSE")
  (make-package "NETFT_UTILS::SETFILTERRESPONSE"))

(in-package "ROS")





(defclass netft_utils::SetFilterRequest
  :super ros::object
  :slots (_toFilter _deltaT _cutoffFrequency ))

(defmethod netft_utils::SetFilterRequest
  (:init
   (&key
    ((:toFilter __toFilter) nil)
    ((:deltaT __deltaT) 0.0)
    ((:cutoffFrequency __cutoffFrequency) 0.0)
    )
   (send-super :init)
   (setq _toFilter __toFilter)
   (setq _deltaT (float __deltaT))
   (setq _cutoffFrequency (float __cutoffFrequency))
   self)
  (:toFilter
   (&optional __toFilter)
   (if __toFilter (setq _toFilter __toFilter)) _toFilter)
  (:deltaT
   (&optional __deltaT)
   (if __deltaT (setq _deltaT __deltaT)) _deltaT)
  (:cutoffFrequency
   (&optional __cutoffFrequency)
   (if __cutoffFrequency (setq _cutoffFrequency __cutoffFrequency)) _cutoffFrequency)
  (:serialization-length
   ()
   (+
    ;; bool _toFilter
    1
    ;; float64 _deltaT
    8
    ;; float64 _cutoffFrequency
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _toFilter
       (if _toFilter (write-byte -1 s) (write-byte 0 s))
     ;; float64 _deltaT
       (sys::poke _deltaT (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cutoffFrequency
       (sys::poke _cutoffFrequency (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _toFilter
     (setq _toFilter (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _deltaT
     (setq _deltaT (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cutoffFrequency
     (setq _cutoffFrequency (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass netft_utils::SetFilterResponse
  :super ros::object
  :slots ())

(defmethod netft_utils::SetFilterResponse
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

(defclass netft_utils::SetFilter
  :super ros::object
  :slots ())

(setf (get netft_utils::SetFilter :md5sum-) "63edfd1498649d874534855980e23bf0")
(setf (get netft_utils::SetFilter :datatype-) "netft_utils/SetFilter")
(setf (get netft_utils::SetFilter :request) netft_utils::SetFilterRequest)
(setf (get netft_utils::SetFilter :response) netft_utils::SetFilterResponse)

(defmethod netft_utils::SetFilterRequest
  (:response () (instance netft_utils::SetFilterResponse :init)))

(setf (get netft_utils::SetFilterRequest :md5sum-) "63edfd1498649d874534855980e23bf0")
(setf (get netft_utils::SetFilterRequest :datatype-) "netft_utils/SetFilterRequest")
(setf (get netft_utils::SetFilterRequest :definition-)
      "bool toFilter
float64 deltaT
float64 cutoffFrequency
---

")

(setf (get netft_utils::SetFilterResponse :md5sum-) "63edfd1498649d874534855980e23bf0")
(setf (get netft_utils::SetFilterResponse :datatype-) "netft_utils/SetFilterResponse")
(setf (get netft_utils::SetFilterResponse :definition-)
      "bool toFilter
float64 deltaT
float64 cutoffFrequency
---

")



(provide :netft_utils/SetFilter "63edfd1498649d874534855980e23bf0")


