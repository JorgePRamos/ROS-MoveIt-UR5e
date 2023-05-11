
(cl:in-package :asdf)

(defsystem "robotiq_ft_sensor-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ft_sensor" :depends-on ("_package_ft_sensor"))
    (:file "_package_ft_sensor" :depends-on ("_package"))
  ))