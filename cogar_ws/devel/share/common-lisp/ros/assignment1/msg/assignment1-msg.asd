
(cl:in-package :asdf)

(defsystem "assignment1-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :nav_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "SensorFusion" :depends-on ("_package_SensorFusion"))
    (:file "_package_SensorFusion" :depends-on ("_package"))
  ))