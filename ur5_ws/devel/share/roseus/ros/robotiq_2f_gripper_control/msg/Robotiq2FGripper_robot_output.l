;; Auto-generated. Do not edit!


(when (boundp 'robotiq_2f_gripper_control::Robotiq2FGripper_robot_output)
  (if (not (find-package "ROBOTIQ_2F_GRIPPER_CONTROL"))
    (make-package "ROBOTIQ_2F_GRIPPER_CONTROL"))
  (shadow 'Robotiq2FGripper_robot_output (find-package "ROBOTIQ_2F_GRIPPER_CONTROL")))
(unless (find-package "ROBOTIQ_2F_GRIPPER_CONTROL::ROBOTIQ2FGRIPPER_ROBOT_OUTPUT")
  (make-package "ROBOTIQ_2F_GRIPPER_CONTROL::ROBOTIQ2FGRIPPER_ROBOT_OUTPUT"))

(in-package "ROS")
;;//! \htmlinclude Robotiq2FGripper_robot_output.msg.html


(defclass robotiq_2f_gripper_control::Robotiq2FGripper_robot_output
  :super ros::object
  :slots (_rACT _rGTO _rATR _rPR _rSP _rFR ))

(defmethod robotiq_2f_gripper_control::Robotiq2FGripper_robot_output
  (:init
   (&key
    ((:rACT __rACT) 0)
    ((:rGTO __rGTO) 0)
    ((:rATR __rATR) 0)
    ((:rPR __rPR) 0)
    ((:rSP __rSP) 0)
    ((:rFR __rFR) 0)
    )
   (send-super :init)
   (setq _rACT (round __rACT))
   (setq _rGTO (round __rGTO))
   (setq _rATR (round __rATR))
   (setq _rPR (round __rPR))
   (setq _rSP (round __rSP))
   (setq _rFR (round __rFR))
   self)
  (:rACT
   (&optional __rACT)
   (if __rACT (setq _rACT __rACT)) _rACT)
  (:rGTO
   (&optional __rGTO)
   (if __rGTO (setq _rGTO __rGTO)) _rGTO)
  (:rATR
   (&optional __rATR)
   (if __rATR (setq _rATR __rATR)) _rATR)
  (:rPR
   (&optional __rPR)
   (if __rPR (setq _rPR __rPR)) _rPR)
  (:rSP
   (&optional __rSP)
   (if __rSP (setq _rSP __rSP)) _rSP)
  (:rFR
   (&optional __rFR)
   (if __rFR (setq _rFR __rFR)) _rFR)
  (:serialization-length
   ()
   (+
    ;; uint8 _rACT
    1
    ;; uint8 _rGTO
    1
    ;; uint8 _rATR
    1
    ;; uint8 _rPR
    1
    ;; uint8 _rSP
    1
    ;; uint8 _rFR
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _rACT
       (write-byte _rACT s)
     ;; uint8 _rGTO
       (write-byte _rGTO s)
     ;; uint8 _rATR
       (write-byte _rATR s)
     ;; uint8 _rPR
       (write-byte _rPR s)
     ;; uint8 _rSP
       (write-byte _rSP s)
     ;; uint8 _rFR
       (write-byte _rFR s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _rACT
     (setq _rACT (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _rGTO
     (setq _rGTO (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _rATR
     (setq _rATR (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _rPR
     (setq _rPR (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _rSP
     (setq _rSP (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _rFR
     (setq _rFR (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get robotiq_2f_gripper_control::Robotiq2FGripper_robot_output :md5sum-) "481503a99d995d0e403b7ee708c6862c")
(setf (get robotiq_2f_gripper_control::Robotiq2FGripper_robot_output :datatype-) "robotiq_2f_gripper_control/Robotiq2FGripper_robot_output")
(setf (get robotiq_2f_gripper_control::Robotiq2FGripper_robot_output :definition-)
      "uint8 rACT
uint8 rGTO
uint8 rATR
uint8 rPR
uint8 rSP
uint8 rFR


")



(provide :robotiq_2f_gripper_control/Robotiq2FGripper_robot_output "481503a99d995d0e403b7ee708c6862c")


