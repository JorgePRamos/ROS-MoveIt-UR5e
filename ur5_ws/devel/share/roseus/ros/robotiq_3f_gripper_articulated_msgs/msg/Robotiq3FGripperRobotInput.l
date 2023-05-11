;; Auto-generated. Do not edit!


(when (boundp 'robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput)
  (if (not (find-package "ROBOTIQ_3F_GRIPPER_ARTICULATED_MSGS"))
    (make-package "ROBOTIQ_3F_GRIPPER_ARTICULATED_MSGS"))
  (shadow 'Robotiq3FGripperRobotInput (find-package "ROBOTIQ_3F_GRIPPER_ARTICULATED_MSGS")))
(unless (find-package "ROBOTIQ_3F_GRIPPER_ARTICULATED_MSGS::ROBOTIQ3FGRIPPERROBOTINPUT")
  (make-package "ROBOTIQ_3F_GRIPPER_ARTICULATED_MSGS::ROBOTIQ3FGRIPPERROBOTINPUT"))

(in-package "ROS")
;;//! \htmlinclude Robotiq3FGripperRobotInput.msg.html


(defclass robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput
  :super ros::object
  :slots (_gACT _gMOD _gGTO _gIMC _gSTA _gDTA _gDTB _gDTC _gDTS _gFLT _gPRA _gPOA _gCUA _gPRB _gPOB _gCUB _gPRC _gPOC _gCUC _gPRS _gPOS _gCUS ))

(defmethod robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput
  (:init
   (&key
    ((:gACT __gACT) 0)
    ((:gMOD __gMOD) 0)
    ((:gGTO __gGTO) 0)
    ((:gIMC __gIMC) 0)
    ((:gSTA __gSTA) 0)
    ((:gDTA __gDTA) 0)
    ((:gDTB __gDTB) 0)
    ((:gDTC __gDTC) 0)
    ((:gDTS __gDTS) 0)
    ((:gFLT __gFLT) 0)
    ((:gPRA __gPRA) 0)
    ((:gPOA __gPOA) 0)
    ((:gCUA __gCUA) 0)
    ((:gPRB __gPRB) 0)
    ((:gPOB __gPOB) 0)
    ((:gCUB __gCUB) 0)
    ((:gPRC __gPRC) 0)
    ((:gPOC __gPOC) 0)
    ((:gCUC __gCUC) 0)
    ((:gPRS __gPRS) 0)
    ((:gPOS __gPOS) 0)
    ((:gCUS __gCUS) 0)
    )
   (send-super :init)
   (setq _gACT (round __gACT))
   (setq _gMOD (round __gMOD))
   (setq _gGTO (round __gGTO))
   (setq _gIMC (round __gIMC))
   (setq _gSTA (round __gSTA))
   (setq _gDTA (round __gDTA))
   (setq _gDTB (round __gDTB))
   (setq _gDTC (round __gDTC))
   (setq _gDTS (round __gDTS))
   (setq _gFLT (round __gFLT))
   (setq _gPRA (round __gPRA))
   (setq _gPOA (round __gPOA))
   (setq _gCUA (round __gCUA))
   (setq _gPRB (round __gPRB))
   (setq _gPOB (round __gPOB))
   (setq _gCUB (round __gCUB))
   (setq _gPRC (round __gPRC))
   (setq _gPOC (round __gPOC))
   (setq _gCUC (round __gCUC))
   (setq _gPRS (round __gPRS))
   (setq _gPOS (round __gPOS))
   (setq _gCUS (round __gCUS))
   self)
  (:gACT
   (&optional __gACT)
   (if __gACT (setq _gACT __gACT)) _gACT)
  (:gMOD
   (&optional __gMOD)
   (if __gMOD (setq _gMOD __gMOD)) _gMOD)
  (:gGTO
   (&optional __gGTO)
   (if __gGTO (setq _gGTO __gGTO)) _gGTO)
  (:gIMC
   (&optional __gIMC)
   (if __gIMC (setq _gIMC __gIMC)) _gIMC)
  (:gSTA
   (&optional __gSTA)
   (if __gSTA (setq _gSTA __gSTA)) _gSTA)
  (:gDTA
   (&optional __gDTA)
   (if __gDTA (setq _gDTA __gDTA)) _gDTA)
  (:gDTB
   (&optional __gDTB)
   (if __gDTB (setq _gDTB __gDTB)) _gDTB)
  (:gDTC
   (&optional __gDTC)
   (if __gDTC (setq _gDTC __gDTC)) _gDTC)
  (:gDTS
   (&optional __gDTS)
   (if __gDTS (setq _gDTS __gDTS)) _gDTS)
  (:gFLT
   (&optional __gFLT)
   (if __gFLT (setq _gFLT __gFLT)) _gFLT)
  (:gPRA
   (&optional __gPRA)
   (if __gPRA (setq _gPRA __gPRA)) _gPRA)
  (:gPOA
   (&optional __gPOA)
   (if __gPOA (setq _gPOA __gPOA)) _gPOA)
  (:gCUA
   (&optional __gCUA)
   (if __gCUA (setq _gCUA __gCUA)) _gCUA)
  (:gPRB
   (&optional __gPRB)
   (if __gPRB (setq _gPRB __gPRB)) _gPRB)
  (:gPOB
   (&optional __gPOB)
   (if __gPOB (setq _gPOB __gPOB)) _gPOB)
  (:gCUB
   (&optional __gCUB)
   (if __gCUB (setq _gCUB __gCUB)) _gCUB)
  (:gPRC
   (&optional __gPRC)
   (if __gPRC (setq _gPRC __gPRC)) _gPRC)
  (:gPOC
   (&optional __gPOC)
   (if __gPOC (setq _gPOC __gPOC)) _gPOC)
  (:gCUC
   (&optional __gCUC)
   (if __gCUC (setq _gCUC __gCUC)) _gCUC)
  (:gPRS
   (&optional __gPRS)
   (if __gPRS (setq _gPRS __gPRS)) _gPRS)
  (:gPOS
   (&optional __gPOS)
   (if __gPOS (setq _gPOS __gPOS)) _gPOS)
  (:gCUS
   (&optional __gCUS)
   (if __gCUS (setq _gCUS __gCUS)) _gCUS)
  (:serialization-length
   ()
   (+
    ;; uint8 _gACT
    1
    ;; uint8 _gMOD
    1
    ;; uint8 _gGTO
    1
    ;; uint8 _gIMC
    1
    ;; uint8 _gSTA
    1
    ;; uint8 _gDTA
    1
    ;; uint8 _gDTB
    1
    ;; uint8 _gDTC
    1
    ;; uint8 _gDTS
    1
    ;; uint8 _gFLT
    1
    ;; uint8 _gPRA
    1
    ;; uint8 _gPOA
    1
    ;; uint8 _gCUA
    1
    ;; uint8 _gPRB
    1
    ;; uint8 _gPOB
    1
    ;; uint8 _gCUB
    1
    ;; uint8 _gPRC
    1
    ;; uint8 _gPOC
    1
    ;; uint8 _gCUC
    1
    ;; uint8 _gPRS
    1
    ;; uint8 _gPOS
    1
    ;; uint8 _gCUS
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _gACT
       (write-byte _gACT s)
     ;; uint8 _gMOD
       (write-byte _gMOD s)
     ;; uint8 _gGTO
       (write-byte _gGTO s)
     ;; uint8 _gIMC
       (write-byte _gIMC s)
     ;; uint8 _gSTA
       (write-byte _gSTA s)
     ;; uint8 _gDTA
       (write-byte _gDTA s)
     ;; uint8 _gDTB
       (write-byte _gDTB s)
     ;; uint8 _gDTC
       (write-byte _gDTC s)
     ;; uint8 _gDTS
       (write-byte _gDTS s)
     ;; uint8 _gFLT
       (write-byte _gFLT s)
     ;; uint8 _gPRA
       (write-byte _gPRA s)
     ;; uint8 _gPOA
       (write-byte _gPOA s)
     ;; uint8 _gCUA
       (write-byte _gCUA s)
     ;; uint8 _gPRB
       (write-byte _gPRB s)
     ;; uint8 _gPOB
       (write-byte _gPOB s)
     ;; uint8 _gCUB
       (write-byte _gCUB s)
     ;; uint8 _gPRC
       (write-byte _gPRC s)
     ;; uint8 _gPOC
       (write-byte _gPOC s)
     ;; uint8 _gCUC
       (write-byte _gCUC s)
     ;; uint8 _gPRS
       (write-byte _gPRS s)
     ;; uint8 _gPOS
       (write-byte _gPOS s)
     ;; uint8 _gCUS
       (write-byte _gCUS s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _gACT
     (setq _gACT (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gMOD
     (setq _gMOD (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gGTO
     (setq _gGTO (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gIMC
     (setq _gIMC (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gSTA
     (setq _gSTA (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gDTA
     (setq _gDTA (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gDTB
     (setq _gDTB (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gDTC
     (setq _gDTC (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gDTS
     (setq _gDTS (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gFLT
     (setq _gFLT (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gPRA
     (setq _gPRA (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gPOA
     (setq _gPOA (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gCUA
     (setq _gCUA (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gPRB
     (setq _gPRB (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gPOB
     (setq _gPOB (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gCUB
     (setq _gCUB (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gPRC
     (setq _gPRC (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gPOC
     (setq _gPOC (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gCUC
     (setq _gCUC (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gPRS
     (setq _gPRS (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gPOS
     (setq _gPOS (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gCUS
     (setq _gCUS (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput :md5sum-) "4d0701156e580a420c48833f57bc83f3")
(setf (get robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput :datatype-) "robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotInput")
(setf (get robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput :definition-)
      "# gACT : Initialization status, echo of the rACT bit (activation bit).
# 0x0 - Gripper reset.
# 0x1 - Gripper activation.

uint8 gACT

# gMOD : Operation mode status, echo of the rMOD bits (grasping mode requested).
# 0x00 - Basic mode.
# 0x01 - Pinch mode.
# 0x02 - Wide mode.
# 0x03 - Scissor mode.

uint8 gMOD

# gGTO : Action status. echo of the rGTO bit (go to bit).
# 0x0 - Stopped (or performing activation / grasping mode change / automatic release)
# 0x1 - Go to Position Request

uint8 gGTO

# gIMC : Gripper status, returns the current status of the Gripper.
# 0x00 - Gripper is in reset (or automatic release) state. See Fault status if Gripper is activated.
# 0x01 - Activation is in progress.
# 0x02 - Mode change is in progress.
# 0x03 - Activation and mode change are completed.

uint8 gIMC

# gSTA : Motion status, returns the current motion of the Gripper fingers.
# 0x00 - Gripper is in motion towards requested position (only meaningful if gGTO = 1)
# 0x01 - Gripper is stopped. One or two fingers stopped before requested position
# 0x02 - Gripper is stopped. All fingers stopped before requested position
# 0x03 - Gripper is stopped. All fingers reached requested position

uint8 gSTA

# gDTA : Finger A object detection status returns information on possible object contact from finger A.
# 0x00 - Finger A is in motion (only meaningful if gGTO = 1).
# 0x01 - Finger A has stopped due to a contact while opening.
# 0x02 - Finger A has stopped due to a contact while closing.
# 0x03 - Finger A is at requested position.

uint8 gDTA

# gDTB : Finger B object detection status returns information on possible object contact from finger B.
# 0x00 - Finger B is in motion (only meaningful if gGTO = 1).
# 0x01 - Finger B has stopped due to a contact while opening.
# 0x02 - Finger B has stopped due to a contact while closing.
# 0x03 - Finger B is at requested position.

uint8 gDTB

# gDTC : Finger C object detection status returns information on possible object contact from finger A.
# 0x00 - Finger C is in motion (only meaningful if gGTO = 1).
# 0x01 - Finger C has stopped due to a contact while opening.
# 0x02 - Finger C has stopped due to a contact while closing.
# 0x03 - Finger C is at requested position.

uint8 gDTC

# gDTS : Scissor object detection status returns information on possible object contact from scissor.
# 0x00 - Scissor is in motion (only meaningful if gGTO = 1).
# 0x01 - Scissor has stopped due to a contact while opening.
# 0x02 - Scissor has stopped due to a contact while closing.
# 0x03 - Scissor is at requested position.

uint8 gDTS

# gFLT : Fault status returns general error messages useful for troubleshooting.
#   0x00 - No fault (fault LED off)
#   Priority faults (fault LED off)
#     0x05 - Action delayed, activation (reactivation) must be completed prior to action.
#     0x06 - Action delayed, mode change must be completed prior to action.
#     0x07 - The activation bit must be set prior to action.
#   Minor faults (fault LED continuous red)
#     0x09 - The communication chip is not ready (may be booting).
#     0x0A - Changing mode fault, interferences detected on Scissor (for less than 20 sec).
#     0x0B - Automatic release in progress.
#   Major faults (fault LED blinking red) - Reset is required
#     0x0D - Activation fault, verify that no interference or other error occurred.
#     0x0E - Changing mode fault, interferences detected on Scissor (for more than 20 sec).
#     0x0F - Automatic release completed. Reset and activation is required.

uint8 gFLT


# gPRA : Echo of the requested position of the Gripper (rPRA),
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).
# If commanding the Gripper in
#   individual control mode, gPRA is the echo of finger A,
#   otherwise it is the general position requested to all fingers.

uint8 gPRA

# gPOA : Returns the actual position of the Gripper finger A,
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 gPOA

# gCUA : Returns a value that represents the finger A instantaneous current consumption from 0x00 to 0xFF.

uint8 gCUA

# gPRB : Echo of the requested position of finger B (rPRB),
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 gPRB

# gPOB : Returns the actual position of the Gripper finger B,
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 gPOB

# gCUB : Returns a value that represents the finger B instantaneous current consumption from 0x00 to 0xFF.

uint8 gCUB

# gPRC : Echo of the requested position of finger C (rPRC),
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 gPRC

# gPOC : Returns the actual position of the Gripper finger C,
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 gPOC

# gCUC : Returns a value that represents the finger C instantaneous current consumption from 0x00 to 0xFF.

uint8 gCUC

# gPRS : Echo of the requested position of scissor (rPRS),
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 gPRS

# gPOS : Returns the actual position of the Gripper scissor,
#   0x00 is minimum position (full opening) and
#   0xFF is maximum position (full closing).

uint8 gPOS

# gCUS : Returns a value that represents the scissor instantaneous current consumption from 0x00 to 0xFF.
#   0x00 is ...
#   0xFF is ...

uint8 gCUS

")



(provide :robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotInput "4d0701156e580a420c48833f57bc83f3")


