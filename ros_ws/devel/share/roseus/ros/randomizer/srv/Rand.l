;; Auto-generated. Do not edit!


(when (boundp 'randomizer::Rand)
  (if (not (find-package "RANDOMIZER"))
    (make-package "RANDOMIZER"))
  (shadow 'Rand (find-package "RANDOMIZER")))
(unless (find-package "RANDOMIZER::RAND")
  (make-package "RANDOMIZER::RAND"))
(unless (find-package "RANDOMIZER::RANDREQUEST")
  (make-package "RANDOMIZER::RANDREQUEST"))
(unless (find-package "RANDOMIZER::RANDRESPONSE")
  (make-package "RANDOMIZER::RANDRESPONSE"))

(in-package "ROS")





(defclass randomizer::RandRequest
  :super ros::object
  :slots (_toStart ))

(defmethod randomizer::RandRequest
  (:init
   (&key
    ((:toStart __toStart) nil)
    )
   (send-super :init)
   (setq _toStart __toStart)
   self)
  (:toStart
   (&optional __toStart)
   (if __toStart (setq _toStart __toStart)) _toStart)
  (:serialization-length
   ()
   (+
    ;; bool _toStart
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _toStart
       (if _toStart (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _toStart
     (setq _toStart (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass randomizer::RandResponse
  :super ros::object
  :slots ())

(defmethod randomizer::RandResponse
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

(defclass randomizer::Rand
  :super ros::object
  :slots ())

(setf (get randomizer::Rand :md5sum-) "cb0da9f8cfc2a14ca80d5a9fa236c5fd")
(setf (get randomizer::Rand :datatype-) "randomizer/Rand")
(setf (get randomizer::Rand :request) randomizer::RandRequest)
(setf (get randomizer::Rand :response) randomizer::RandResponse)

(defmethod randomizer::RandRequest
  (:response () (instance randomizer::RandResponse :init)))

(setf (get randomizer::RandRequest :md5sum-) "cb0da9f8cfc2a14ca80d5a9fa236c5fd")
(setf (get randomizer::RandRequest :datatype-) "randomizer/RandRequest")
(setf (get randomizer::RandRequest :definition-)
      "bool toStart
---
")

(setf (get randomizer::RandResponse :md5sum-) "cb0da9f8cfc2a14ca80d5a9fa236c5fd")
(setf (get randomizer::RandResponse :datatype-) "randomizer/RandResponse")
(setf (get randomizer::RandResponse :definition-)
      "bool toStart
---
")



(provide :randomizer/Rand "cb0da9f8cfc2a14ca80d5a9fa236c5fd")


