
(cl:in-package :asdf)

(defsystem "netft_utils-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Cancel" :depends-on ("_package_Cancel"))
    (:file "_package_Cancel" :depends-on ("_package"))
  ))