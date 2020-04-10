;; Auto-generated. Do not edit!


(when (boundp 'netft_utils::StartSim)
  (if (not (find-package "NETFT_UTILS"))
    (make-package "NETFT_UTILS"))
  (shadow 'StartSim (find-package "NETFT_UTILS")))
(unless (find-package "NETFT_UTILS::STARTSIM")
  (make-package "NETFT_UTILS::STARTSIM"))
(unless (find-package "NETFT_UTILS::STARTSIMREQUEST")
  (make-package "NETFT_UTILS::STARTSIMREQUEST"))
(unless (find-package "NETFT_UTILS::STARTSIMRESPONSE")
  (make-package "NETFT_UTILS::STARTSIMRESPONSE"))

(in-package "ROS")





(defclass netft_utils::StartSimRequest
  :super ros::object
  :slots (_simDim _simType _simSlope _maxForce ))

(defmethod netft_utils::StartSimRequest
  (:init
   (&key
    ((:simDim __simDim) 0)
    ((:simType __simType) 0)
    ((:simSlope __simSlope) 0.0)
    ((:maxForce __maxForce) 0.0)
    )
   (send-super :init)
   (setq _simDim (round __simDim))
   (setq _simType (round __simType))
   (setq _simSlope (float __simSlope))
   (setq _maxForce (float __maxForce))
   self)
  (:simDim
   (&optional __simDim)
   (if __simDim (setq _simDim __simDim)) _simDim)
  (:simType
   (&optional __simType)
   (if __simType (setq _simType __simType)) _simType)
  (:simSlope
   (&optional __simSlope)
   (if __simSlope (setq _simSlope __simSlope)) _simSlope)
  (:maxForce
   (&optional __maxForce)
   (if __maxForce (setq _maxForce __maxForce)) _maxForce)
  (:serialization-length
   ()
   (+
    ;; int32 _simDim
    4
    ;; int32 _simType
    4
    ;; float64 _simSlope
    8
    ;; float64 _maxForce
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _simDim
       (write-long _simDim s)
     ;; int32 _simType
       (write-long _simType s)
     ;; float64 _simSlope
       (sys::poke _simSlope (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _maxForce
       (sys::poke _maxForce (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _simDim
     (setq _simDim (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _simType
     (setq _simType (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _simSlope
     (setq _simSlope (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _maxForce
     (setq _maxForce (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass netft_utils::StartSimResponse
  :super ros::object
  :slots ())

(defmethod netft_utils::StartSimResponse
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

(defclass netft_utils::StartSim
  :super ros::object
  :slots ())

(setf (get netft_utils::StartSim :md5sum-) "f9bc0ea45e890996bb2e2ce115e7c8c3")
(setf (get netft_utils::StartSim :datatype-) "netft_utils/StartSim")
(setf (get netft_utils::StartSim :request) netft_utils::StartSimRequest)
(setf (get netft_utils::StartSim :response) netft_utils::StartSimResponse)

(defmethod netft_utils::StartSimRequest
  (:response () (instance netft_utils::StartSimResponse :init)))

(setf (get netft_utils::StartSimRequest :md5sum-) "f9bc0ea45e890996bb2e2ce115e7c8c3")
(setf (get netft_utils::StartSimRequest :datatype-) "netft_utils/StartSimRequest")
(setf (get netft_utils::StartSimRequest :definition-)
      "int32 simDim
int32 simType
float64 simSlope
float64 maxForce
---

")

(setf (get netft_utils::StartSimResponse :md5sum-) "f9bc0ea45e890996bb2e2ce115e7c8c3")
(setf (get netft_utils::StartSimResponse :datatype-) "netft_utils/StartSimResponse")
(setf (get netft_utils::StartSimResponse :definition-)
      "int32 simDim
int32 simType
float64 simSlope
float64 maxForce
---

")



(provide :netft_utils/StartSim "f9bc0ea45e890996bb2e2ce115e7c8c3")


