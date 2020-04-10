;; Auto-generated. Do not edit!


(when (boundp 'netft_utils::Cancel)
  (if (not (find-package "NETFT_UTILS"))
    (make-package "NETFT_UTILS"))
  (shadow 'Cancel (find-package "NETFT_UTILS")))
(unless (find-package "NETFT_UTILS::CANCEL")
  (make-package "NETFT_UTILS::CANCEL"))

(in-package "ROS")
;;//! \htmlinclude Cancel.msg.html


(defclass netft_utils::Cancel
  :super ros::object
  :slots (_toCancel ))

(defmethod netft_utils::Cancel
  (:init
   (&key
    ((:toCancel __toCancel) nil)
    )
   (send-super :init)
   (setq _toCancel __toCancel)
   self)
  (:toCancel
   (&optional __toCancel)
   (if __toCancel (setq _toCancel __toCancel)) _toCancel)
  (:serialization-length
   ()
   (+
    ;; bool _toCancel
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _toCancel
       (if _toCancel (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _toCancel
     (setq _toCancel (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get netft_utils::Cancel :md5sum-) "60c8bbbce9a3c19aed52c3a96b5d87ff")
(setf (get netft_utils::Cancel :datatype-) "netft_utils/Cancel")
(setf (get netft_utils::Cancel :definition-)
      "bool toCancel

")



(provide :netft_utils/Cancel "60c8bbbce9a3c19aed52c3a96b5d87ff")


