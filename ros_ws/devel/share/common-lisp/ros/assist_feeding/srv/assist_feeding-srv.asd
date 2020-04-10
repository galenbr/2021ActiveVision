
(cl:in-package :asdf)

(defsystem "assist_feeding-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "finger_orientation" :depends-on ("_package_finger_orientation"))
    (:file "_package_finger_orientation" :depends-on ("_package"))
    (:file "gripper_close" :depends-on ("_package_gripper_close"))
    (:file "_package_gripper_close" :depends-on ("_package"))
    (:file "gripper_open" :depends-on ("_package_gripper_open"))
    (:file "_package_gripper_open" :depends-on ("_package"))
  ))