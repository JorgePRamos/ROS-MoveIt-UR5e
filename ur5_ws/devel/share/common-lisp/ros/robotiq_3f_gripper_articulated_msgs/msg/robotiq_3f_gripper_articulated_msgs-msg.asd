
(cl:in-package :asdf)

(defsystem "robotiq_3f_gripper_articulated_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Robotiq3FGripperRobotInput" :depends-on ("_package_Robotiq3FGripperRobotInput"))
    (:file "_package_Robotiq3FGripperRobotInput" :depends-on ("_package"))
    (:file "Robotiq3FGripperRobotOutput" :depends-on ("_package_Robotiq3FGripperRobotOutput"))
    (:file "_package_Robotiq3FGripperRobotOutput" :depends-on ("_package"))
  ))