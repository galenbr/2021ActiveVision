
(cl:in-package :asdf)

(defsystem "pose_estimator-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "PoseEstimation" :depends-on ("_package_PoseEstimation"))
    (:file "_package_PoseEstimation" :depends-on ("_package"))
  ))