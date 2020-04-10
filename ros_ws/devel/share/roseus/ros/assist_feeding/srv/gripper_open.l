;; Auto-generated. Do not edit!


(when (boundp 'assist_feeding::gripper_open)
  (if (not (find-package "ASSIST_FEEDING"))
    (make-package "ASSIST_FEEDING"))
  (shadow 'gripper_open (find-package "ASSIST_FEEDING")))
(unless (find-package "ASSIST_FEEDING::GRIPPER_OPEN")
  (make-package "ASSIST_FEEDING::GRIPPER_OPEN"))
(unless (find-package "ASSIST_FEEDING::GRIPPER_OPENREQUEST")
  (make-package "ASSIST_FEEDING::GRIPPER_OPENREQUEST"))
(unless (find-package "ASSIST_FEEDING::GRIPPER_OPENRESPONSE")
  (make-package "ASSIST_FEEDING::GRIPPER_OPENRESPONSE"))

(in-package "ROS")





(defclass assist_feeding::gripper_openRequest
  :super ros::object
  :slots (_gripper_state ))

(defmethod assist_feeding::gripper_openRequest
  (:init
   (&key
    ((:gripper_state __gripper_state) 0)
    )
   (send-super :init)
   (setq _gripper_state (round __gripper_state))
   self)
  (:gripper_state
   (&optional __gripper_state)
   (if __gripper_state (setq _gripper_state __gripper_state)) _gripper_state)
  (:serialization-length
   ()
   (+
    ;; int32 _gripper_state
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _gripper_state
       (write-long _gripper_state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _gripper_state
     (setq _gripper_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass assist_feeding::gripper_openResponse
  :super ros::object
  :slots (_gripper_state ))

(defmethod assist_feeding::gripper_openResponse
  (:init
   (&key
    ((:gripper_state __gripper_state) 0)
    )
   (send-super :init)
   (setq _gripper_state (round __gripper_state))
   self)
  (:gripper_state
   (&optional __gripper_state)
   (if __gripper_state (setq _gripper_state __gripper_state)) _gripper_state)
  (:serialization-length
   ()
   (+
    ;; int32 _gripper_state
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _gripper_state
       (write-long _gripper_state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _gripper_state
     (setq _gripper_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass assist_feeding::gripper_open
  :super ros::object
  :slots ())

(setf (get assist_feeding::gripper_open :md5sum-) "7626e2955bd6177e24f663e9b9a1fde1")
(setf (get assist_feeding::gripper_open :datatype-) "assist_feeding/gripper_open")
(setf (get assist_feeding::gripper_open :request) assist_feeding::gripper_openRequest)
(setf (get assist_feeding::gripper_open :response) assist_feeding::gripper_openResponse)

(defmethod assist_feeding::gripper_openRequest
  (:response () (instance assist_feeding::gripper_openResponse :init)))

(setf (get assist_feeding::gripper_openRequest :md5sum-) "7626e2955bd6177e24f663e9b9a1fde1")
(setf (get assist_feeding::gripper_openRequest :datatype-) "assist_feeding/gripper_openRequest")
(setf (get assist_feeding::gripper_openRequest :definition-)
      "int32 gripper_state
---
int32 gripper_state
")

(setf (get assist_feeding::gripper_openResponse :md5sum-) "7626e2955bd6177e24f663e9b9a1fde1")
(setf (get assist_feeding::gripper_openResponse :datatype-) "assist_feeding/gripper_openResponse")
(setf (get assist_feeding::gripper_openResponse :definition-)
      "int32 gripper_state
---
int32 gripper_state
")



(provide :assist_feeding/gripper_open "7626e2955bd6177e24f663e9b9a1fde1")


