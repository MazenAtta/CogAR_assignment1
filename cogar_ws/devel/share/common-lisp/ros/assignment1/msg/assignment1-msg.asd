
(cl:in-package :asdf)

(defsystem "assignment1-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SensorFusion" :depends-on ("_package_SensorFusion"))
    (:file "_package_SensorFusion" :depends-on ("_package"))
    (:file "fusion" :depends-on ("_package_fusion"))
    (:file "_package_fusion" :depends-on ("_package"))
  ))