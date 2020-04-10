;; Auto-generated. Do not edit!


(when (boundp 'assist_feeding::gripper_close)
  (if (not (find-package "ASSIST_FEEDING"))
    (make-package "ASSIST_FEEDING"))
  (shadow 'gripper_close (find-package "ASSIST_FEEDING")))
(unless (find-package "ASSIST_FEEDING::GRIPPER_CLOSE")
  (make-package "ASSIST_FEEDING::GRIPPER_CLOSE"))
(unless (find-package "ASSIST_FEEDING::GRIPPER_CLOSEREQUEST")
  (make-package "ASSIST_FEEDING::GRIPPER_CLOSEREQUEST"))
(unless (find-package "ASSIST_FEEDING::GRIPPER_CLOSERESPONSE")
  (make-package "ASSIST_FEEDING::GRIPPER_CLOSERESPONSE"))

(in-package "ROS")





(defclass assist_feeding::gripper_closeRequest
  :super ros::object
  :slots (_gripper_state ))

(defmethod assist_feeding::gripper_closeRequest
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

(defclass assist_feeding::gripper_closeResponse
  :super ros::object
  :slots (_gripper_state ))

(defmethod assist_feeding::gripper_closeResponse
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

(defclass assist_feeding::gripper_close
  :super ros::object
  :slots ())

(setf (get assist_feeding::gripper_close :md5sum-) "7626e2955bd6177e24f663e9b9a1fde1")
(setf (get assist_feeding::gripper_close :datatype-) "assist_feeding/gripper_close")
(setf (get assist_feeding::gripper_close :request) assist_feeding::gripper_closeRequest)
(setf (get assist_feeding::gripper_close :response) assist_feeding::gripper_closeResponse)

(defmethod assist_feeding::gripper_closeRequest
  (:response () (instance assist_feeding::gripper_closeResponse :init)))

(setf (get assist_feeding::gripper_closeRequest :md5sum-) "7626e2955bd6177e24f663e9b9a1fde1")
(setf (get assist_feeding::gripper_closeRequest :datatype-) "assist_feeding/gripper_closeRequest")
(setf (get assist_feeding::gripper_closeRequest :definition-)
      "int32 gripper_state
---
int32 gripper_state
")

(setf (get assist_feeding::gripper_closeResponse :md5sum-) "7626e2955bd6177e24f663e9b9a1fde1")
(setf (get assist_feeding::gripper_closeResponse :datatype-) "assist_feeding/gripper_closeResponse")
(setf (get assist_feeding::gripper_closeResponse :definition-)
      "int32 gripper_state
---
int32 gripper_state
")



(provide :assist_feeding/gripper_close "7626e2955bd6177e24f663e9b9a1fde1")


