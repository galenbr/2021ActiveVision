
(cl:in-package :asdf)

(defsystem "randomizer-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Rand" :depends-on ("_package_Rand"))
    (:file "_package_Rand" :depends-on ("_package"))
  ))