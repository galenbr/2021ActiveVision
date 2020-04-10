
(cl:in-package :asdf)

(defsystem "pcl_recorder-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "BeginPointCloudRange" :depends-on ("_package_BeginPointCloudRange"))
    (:file "_package_BeginPointCloudRange" :depends-on ("_package"))
    (:file "EndPointCloudRange" :depends-on ("_package_EndPointCloudRange"))
    (:file "_package_EndPointCloudRange" :depends-on ("_package"))
    (:file "GetPointCloud" :depends-on ("_package_GetPointCloud"))
    (:file "_package_GetPointCloud" :depends-on ("_package"))
  ))