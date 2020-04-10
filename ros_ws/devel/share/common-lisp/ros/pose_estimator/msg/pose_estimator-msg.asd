
(cl:in-package :asdf)

(defsystem "pose_estimator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GazeboPoseEstimation" :depends-on ("_package_GazeboPoseEstimation"))
    (:file "_package_GazeboPoseEstimation" :depends-on ("_package"))
  ))