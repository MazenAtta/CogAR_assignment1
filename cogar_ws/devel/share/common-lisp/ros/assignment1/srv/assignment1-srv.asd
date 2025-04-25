
(cl:in-package :asdf)

(defsystem "assignment1-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Speaker" :depends-on ("_package_Speaker"))
    (:file "_package_Speaker" :depends-on ("_package"))
  ))