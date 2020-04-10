;; Auto-generated. Do not edit!


(when (boundp 'pcl_recorder::BeginPointCloudRange)
  (if (not (find-package "PCL_RECORDER"))
    (make-package "PCL_RECORDER"))
  (shadow 'BeginPointCloudRange (find-package "PCL_RECORDER")))
(unless (find-package "PCL_RECORDER::BEGINPOINTCLOUDRANGE")
  (make-package "PCL_RECORDER::BEGINPOINTCLOUDRANGE"))
(unless (find-package "PCL_RECORDER::BEGINPOINTCLOUDRANGEREQUEST")
  (make-package "PCL_RECORDER::BEGINPOINTCLOUDRANGEREQUEST"))
(unless (find-package "PCL_RECORDER::BEGINPOINTCLOUDRANGERESPONSE")
  (make-package "PCL_RECORDER::BEGINPOINTCLOUDRANGERESPONSE"))

(in-package "ROS")





(defclass pcl_recorder::BeginPointCloudRangeRequest
  :super ros::object
  :slots (_frequency ))

(defmethod pcl_recorder::BeginPointCloudRangeRequest
  (:init
   (&key
    ((:frequency __frequency) 0.0)
    )
   (send-super :init)
   (setq _frequency (float __frequency))
   self)
  (:frequency
   (&optional __frequency)
   (if __frequency (setq _frequency __frequency)) _frequency)
  (:serialization-length
   ()
   (+
    ;; float64 _frequency
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _frequency
       (sys::poke _frequency (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _frequency
     (setq _frequency (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass pcl_recorder::BeginPointCloudRangeResponse
  :super ros::object
  :slots (_limitSeconds ))

(defmethod pcl_recorder::BeginPointCloudRangeResponse
  (:init
   (&key
    ((:limitSeconds __limitSeconds) 0.0)
    )
   (send-super :init)
   (setq _limitSeconds (float __limitSeconds))
   self)
  (:limitSeconds
   (&optional __limitSeconds)
   (if __limitSeconds (setq _limitSeconds __limitSeconds)) _limitSeconds)
  (:serialization-length
   ()
   (+
    ;; float64 _limitSeconds
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _limitSeconds
       (sys::poke _limitSeconds (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _limitSeconds
     (setq _limitSeconds (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass pcl_recorder::BeginPointCloudRange
  :super ros::object
  :slots ())

(setf (get pcl_recorder::BeginPointCloudRange :md5sum-) "10432e08567eb968c3444f0fa65097b5")
(setf (get pcl_recorder::BeginPointCloudRange :datatype-) "pcl_recorder/BeginPointCloudRange")
(setf (get pcl_recorder::BeginPointCloudRange :request) pcl_recorder::BeginPointCloudRangeRequest)
(setf (get pcl_recorder::BeginPointCloudRange :response) pcl_recorder::BeginPointCloudRangeResponse)

(defmethod pcl_recorder::BeginPointCloudRangeRequest
  (:response () (instance pcl_recorder::BeginPointCloudRangeResponse :init)))

(setf (get pcl_recorder::BeginPointCloudRangeRequest :md5sum-) "10432e08567eb968c3444f0fa65097b5")
(setf (get pcl_recorder::BeginPointCloudRangeRequest :datatype-) "pcl_recorder/BeginPointCloudRangeRequest")
(setf (get pcl_recorder::BeginPointCloudRangeRequest :definition-)
      "float64 frequency
---
float64 limitSeconds
")

(setf (get pcl_recorder::BeginPointCloudRangeResponse :md5sum-) "10432e08567eb968c3444f0fa65097b5")
(setf (get pcl_recorder::BeginPointCloudRangeResponse :datatype-) "pcl_recorder/BeginPointCloudRangeResponse")
(setf (get pcl_recorder::BeginPointCloudRangeResponse :definition-)
      "float64 frequency
---
float64 limitSeconds
")



(provide :pcl_recorder/BeginPointCloudRange "10432e08567eb968c3444f0fa65097b5")


