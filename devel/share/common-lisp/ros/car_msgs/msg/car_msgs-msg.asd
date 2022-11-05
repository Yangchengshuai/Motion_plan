
(cl:in-package :asdf)

(defsystem "car_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CarCmd" :depends-on ("_package_CarCmd"))
    (:file "_package_CarCmd" :depends-on ("_package"))
  ))