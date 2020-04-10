;; Auto-generated. Do not edit!


(when (boundp 'assist_feeding::finger_orientation)
  (if (not (find-package "ASSIST_FEEDING"))
    (make-package "ASSIST_FEEDING"))
  (shadow 'finger_orientation (find-package "ASSIST_FEEDING")))
(unless (find-package "ASSIST_FEEDING::FINGER_ORIENTATION")
  (make-package "ASSIST_FEEDING::FINGER_ORIENTATION"))
(unless (find-package "ASSIST_FEEDING::FINGER_ORIENTATIONREQUEST")
  (make-package "ASSIST_FEEDING::FINGER_ORIENTATIONREQUEST"))
(unless (find-package "ASSIST_FEEDING::FINGER_ORIENTATIONRESPONSE")
  (make-package "ASSIST_FEEDING::FINGER_ORIENTATIONRESPONSE"))

(in-package "ROS")





(defclass assist_feeding::finger_orientationRequest
  :super ros::object
  :slots (_finger_angle ))

(defmethod assist_feeding::finger_orientationRequest
  (:init
   (&key
    ((:finger_angle __finger_angle) 0.0)
    )
   (send-super :init)
   (setq _finger_angle (float __finger_angle))
   self)
  (:finger_angle
   (&optional __finger_angle)
   (if __finger_angle (setq _finger_angle __finger_angle)) _finger_angle)
  (:serialization-length
   ()
   (+
    ;; float64 _finger_angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _finger_angle
       (sys::poke _finger_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _finger_angle
     (setq _finger_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass assist_feeding::finger_orientationResponse
  :super ros::object
  :slots ())

(defmethod assist_feeding::finger_orientationResponse
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

(defclass assist_feeding::finger_orientation
  :super ros::object
  :slots ())

(setf (get assist_feeding::finger_orientation :md5sum-) "2649f48d233c8c347b13cd50f5edf8d2")
(setf (get assist_feeding::finger_orientation :datatype-) "assist_feeding/finger_orientation")
(setf (get assist_feeding::finger_orientation :request) assist_feeding::finger_orientationRequest)
(setf (get assist_feeding::finger_orientation :response) assist_feeding::finger_orientationResponse)

(defmethod assist_feeding::finger_orientationRequest
  (:response () (instance assist_feeding::finger_orientationResponse :init)))

(setf (get assist_feeding::finger_orientationRequest :md5sum-) "2649f48d233c8c347b13cd50f5edf8d2")
(setf (get assist_feeding::finger_orientationRequest :datatype-) "assist_feeding/finger_orientationRequest")
(setf (get assist_feeding::finger_orientationRequest :definition-)
      "float64 finger_angle
---

")

(setf (get assist_feeding::finger_orientationResponse :md5sum-) "2649f48d233c8c347b13cd50f5edf8d2")
(setf (get assist_feeding::finger_orientationResponse :datatype-) "assist_feeding/finger_orientationResponse")
(setf (get assist_feeding::finger_orientationResponse :definition-)
      "float64 finger_angle
---

")



(provide :assist_feeding/finger_orientation "2649f48d233c8c347b13cd50f5edf8d2")


