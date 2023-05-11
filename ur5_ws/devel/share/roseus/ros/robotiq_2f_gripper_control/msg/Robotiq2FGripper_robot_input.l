;; Auto-generated. Do not edit!


(when (boundp 'robotiq_2f_gripper_control::Robotiq2FGripper_robot_input)
  (if (not (find-package "ROBOTIQ_2F_GRIPPER_CONTROL"))
    (make-package "ROBOTIQ_2F_GRIPPER_CONTROL"))
  (shadow 'Robotiq2FGripper_robot_input (find-package "ROBOTIQ_2F_GRIPPER_CONTROL")))
(unless (find-package "ROBOTIQ_2F_GRIPPER_CONTROL::ROBOTIQ2FGRIPPER_ROBOT_INPUT")
  (make-package "ROBOTIQ_2F_GRIPPER_CONTROL::ROBOTIQ2FGRIPPER_ROBOT_INPUT"))

(in-package "ROS")
;;//! \htmlinclude Robotiq2FGripper_robot_input.msg.html


(defclass robotiq_2f_gripper_control::Robotiq2FGripper_robot_input
  :super ros::object
  :slots (_gACT _gGTO _gSTA _gOBJ _gFLT _gPR _gPO _gCU ))

(defmethod robotiq_2f_gripper_control::Robotiq2FGripper_robot_input
  (:init
   (&key
    ((:gACT __gACT) 0)
    ((:gGTO __gGTO) 0)
    ((:gSTA __gSTA) 0)
    ((:gOBJ __gOBJ) 0)
    ((:gFLT __gFLT) 0)
    ((:gPR __gPR) 0)
    ((:gPO __gPO) 0)
    ((:gCU __gCU) 0)
    )
   (send-super :init)
   (setq _gACT (round __gACT))
   (setq _gGTO (round __gGTO))
   (setq _gSTA (round __gSTA))
   (setq _gOBJ (round __gOBJ))
   (setq _gFLT (round __gFLT))
   (setq _gPR (round __gPR))
   (setq _gPO (round __gPO))
   (setq _gCU (round __gCU))
   self)
  (:gACT
   (&optional __gACT)
   (if __gACT (setq _gACT __gACT)) _gACT)
  (:gGTO
   (&optional __gGTO)
   (if __gGTO (setq _gGTO __gGTO)) _gGTO)
  (:gSTA
   (&optional __gSTA)
   (if __gSTA (setq _gSTA __gSTA)) _gSTA)
  (:gOBJ
   (&optional __gOBJ)
   (if __gOBJ (setq _gOBJ __gOBJ)) _gOBJ)
  (:gFLT
   (&optional __gFLT)
   (if __gFLT (setq _gFLT __gFLT)) _gFLT)
  (:gPR
   (&optional __gPR)
   (if __gPR (setq _gPR __gPR)) _gPR)
  (:gPO
   (&optional __gPO)
   (if __gPO (setq _gPO __gPO)) _gPO)
  (:gCU
   (&optional __gCU)
   (if __gCU (setq _gCU __gCU)) _gCU)
  (:serialization-length
   ()
   (+
    ;; uint8 _gACT
    1
    ;; uint8 _gGTO
    1
    ;; uint8 _gSTA
    1
    ;; uint8 _gOBJ
    1
    ;; uint8 _gFLT
    1
    ;; uint8 _gPR
    1
    ;; uint8 _gPO
    1
    ;; uint8 _gCU
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _gACT
       (write-byte _gACT s)
     ;; uint8 _gGTO
       (write-byte _gGTO s)
     ;; uint8 _gSTA
       (write-byte _gSTA s)
     ;; uint8 _gOBJ
       (write-byte _gOBJ s)
     ;; uint8 _gFLT
       (write-byte _gFLT s)
     ;; uint8 _gPR
       (write-byte _gPR s)
     ;; uint8 _gPO
       (write-byte _gPO s)
     ;; uint8 _gCU
       (write-byte _gCU s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _gACT
     (setq _gACT (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gGTO
     (setq _gGTO (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gSTA
     (setq _gSTA (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gOBJ
     (setq _gOBJ (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gFLT
     (setq _gFLT (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gPR
     (setq _gPR (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gPO
     (setq _gPO (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gCU
     (setq _gCU (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get robotiq_2f_gripper_control::Robotiq2FGripper_robot_input :md5sum-) "17d49e32c00f4f2fb4fe664060553362")
(setf (get robotiq_2f_gripper_control::Robotiq2FGripper_robot_input :datatype-) "robotiq_2f_gripper_control/Robotiq2FGripper_robot_input")
(setf (get robotiq_2f_gripper_control::Robotiq2FGripper_robot_input :definition-)
      "uint8 gACT 
uint8 gGTO 
uint8 gSTA 
uint8 gOBJ 
uint8 gFLT
uint8 gPR
uint8 gPO
uint8 gCU

")



(provide :robotiq_2f_gripper_control/Robotiq2FGripper_robot_input "17d49e32c00f4f2fb4fe664060553362")


