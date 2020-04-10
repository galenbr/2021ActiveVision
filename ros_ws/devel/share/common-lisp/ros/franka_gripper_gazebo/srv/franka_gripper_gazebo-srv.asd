
(cl:in-package :asdf)

(defsystem "franka_gripper_gazebo-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GripMsg" :depends-on ("_package_GripMsg"))
    (:file "_package_GripMsg" :depends-on ("_package"))
  ))