; Auto-generated. Do not edit!


(cl:in-package robotiq_3f_gripper_articulated_msgs-msg)


;//! \htmlinclude Robotiq3FGripperRobotInput.msg.html

(cl:defclass <Robotiq3FGripperRobotInput> (roslisp-msg-protocol:ros-message)
  ((gACT
    :reader gACT
    :initarg :gACT
    :type cl:fixnum
    :initform 0)
   (gMOD
    :reader gMOD
    :initarg :gMOD
    :type cl:fixnum
    :initform 0)
   (gGTO
    :reader gGTO
    :initarg :gGTO
    :type cl:fixnum
    :initform 0)
   (gIMC
    :reader gIMC
    :initarg :gIMC
    :type cl:fixnum
    :initform 0)
   (gSTA
    :reader gSTA
    :initarg :gSTA
    :type cl:fixnum
    :initform 0)
   (gDTA
    :reader gDTA
    :initarg :gDTA
    :type cl:fixnum
    :initform 0)
   (gDTB
    :reader gDTB
    :initarg :gDTB
    :type cl:fixnum
    :initform 0)
   (gDTC
    :reader gDTC
    :initarg :gDTC
    :type cl:fixnum
    :initform 0)
   (gDTS
    :reader gDTS
    :initarg :gDTS
    :type cl:fixnum
    :initform 0)
   (gFLT
    :reader gFLT
    :initarg :gFLT
    :type cl:fixnum
    :initform 0)
   (gPRA
    :reader gPRA
    :initarg :gPRA
    :type cl:fixnum
    :initform 0)
   (gPOA
    :reader gPOA
    :initarg :gPOA
    :type cl:fixnum
    :initform 0)
   (gCUA
    :reader gCUA
    :initarg :gCUA
    :type cl:fixnum
    :initform 0)
   (gPRB
    :reader gPRB
    :initarg :gPRB
    :type cl:fixnum
    :initform 0)
   (gPOB
    :reader gPOB
    :initarg :gPOB
    :type cl:fixnum
    :initform 0)
   (gCUB
    :reader gCUB
    :initarg :gCUB
    :type cl:fixnum
    :initform 0)
   (gPRC
    :reader gPRC
    :initarg :gPRC
    :type cl:fixnum
    :initform 0)
   (gPOC
    :reader gPOC
    :initarg :gPOC
    :type cl:fixnum
    :initform 0)
   (gCUC
    :reader gCUC
    :initarg :gCUC
    :type cl:fixnum
    :initform 0)
   (gPRS
    :reader gPRS
    :initarg :gPRS
    :type cl:fixnum
    :initform 0)
   (gPOS
    :reader gPOS
    :initarg :gPOS
    :type cl:fixnum
    :initform 0)
   (gCUS
    :reader gCUS
    :initarg :gCUS
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Robotiq3FGripperRobotInput (<Robotiq3FGripperRobotInput>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Robotiq3FGripperRobotInput>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Robotiq3FGripperRobotInput)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotiq_3f_gripper_articulated_msgs-msg:<Robotiq3FGripperRobotInput> is deprecated: use robotiq_3f_gripper_articulated_msgs-msg:Robotiq3FGripperRobotInput instead.")))

(cl:ensure-generic-function 'gACT-val :lambda-list '(m))
(cl:defmethod gACT-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gACT-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gACT instead.")
  (gACT m))

(cl:ensure-generic-function 'gMOD-val :lambda-list '(m))
(cl:defmethod gMOD-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gMOD-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gMOD instead.")
  (gMOD m))

(cl:ensure-generic-function 'gGTO-val :lambda-list '(m))
(cl:defmethod gGTO-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gGTO-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gGTO instead.")
  (gGTO m))

(cl:ensure-generic-function 'gIMC-val :lambda-list '(m))
(cl:defmethod gIMC-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gIMC-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gIMC instead.")
  (gIMC m))

(cl:ensure-generic-function 'gSTA-val :lambda-list '(m))
(cl:defmethod gSTA-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gSTA-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gSTA instead.")
  (gSTA m))

(cl:ensure-generic-function 'gDTA-val :lambda-list '(m))
(cl:defmethod gDTA-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gDTA-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gDTA instead.")
  (gDTA m))

(cl:ensure-generic-function 'gDTB-val :lambda-list '(m))
(cl:defmethod gDTB-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gDTB-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gDTB instead.")
  (gDTB m))

(cl:ensure-generic-function 'gDTC-val :lambda-list '(m))
(cl:defmethod gDTC-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gDTC-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gDTC instead.")
  (gDTC m))

(cl:ensure-generic-function 'gDTS-val :lambda-list '(m))
(cl:defmethod gDTS-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gDTS-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gDTS instead.")
  (gDTS m))

(cl:ensure-generic-function 'gFLT-val :lambda-list '(m))
(cl:defmethod gFLT-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gFLT-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gFLT instead.")
  (gFLT m))

(cl:ensure-generic-function 'gPRA-val :lambda-list '(m))
(cl:defmethod gPRA-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gPRA-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gPRA instead.")
  (gPRA m))

(cl:ensure-generic-function 'gPOA-val :lambda-list '(m))
(cl:defmethod gPOA-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gPOA-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gPOA instead.")
  (gPOA m))

(cl:ensure-generic-function 'gCUA-val :lambda-list '(m))
(cl:defmethod gCUA-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gCUA-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gCUA instead.")
  (gCUA m))

(cl:ensure-generic-function 'gPRB-val :lambda-list '(m))
(cl:defmethod gPRB-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gPRB-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gPRB instead.")
  (gPRB m))

(cl:ensure-generic-function 'gPOB-val :lambda-list '(m))
(cl:defmethod gPOB-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gPOB-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gPOB instead.")
  (gPOB m))

(cl:ensure-generic-function 'gCUB-val :lambda-list '(m))
(cl:defmethod gCUB-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gCUB-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gCUB instead.")
  (gCUB m))

(cl:ensure-generic-function 'gPRC-val :lambda-list '(m))
(cl:defmethod gPRC-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gPRC-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gPRC instead.")
  (gPRC m))

(cl:ensure-generic-function 'gPOC-val :lambda-list '(m))
(cl:defmethod gPOC-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gPOC-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gPOC instead.")
  (gPOC m))

(cl:ensure-generic-function 'gCUC-val :lambda-list '(m))
(cl:defmethod gCUC-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gCUC-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gCUC instead.")
  (gCUC m))

(cl:ensure-generic-function 'gPRS-val :lambda-list '(m))
(cl:defmethod gPRS-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gPRS-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gPRS instead.")
  (gPRS m))

(cl:ensure-generic-function 'gPOS-val :lambda-list '(m))
(cl:defmethod gPOS-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gPOS-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gPOS instead.")
  (gPOS m))

(cl:ensure-generic-function 'gCUS-val :lambda-list '(m))
(cl:defmethod gCUS-val ((m <Robotiq3FGripperRobotInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:gCUS-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:gCUS instead.")
  (gCUS m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Robotiq3FGripperRobotInput>) ostream)
  "Serializes a message object of type '<Robotiq3FGripperRobotInput>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gACT)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gMOD)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gGTO)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gIMC)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gSTA)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gDTA)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gDTB)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gDTC)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gDTS)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gFLT)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gPRA)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gPOA)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gCUA)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gPRB)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gPOB)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gCUB)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gPRC)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gPOC)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gCUC)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gPRS)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gPOS)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gCUS)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Robotiq3FGripperRobotInput>) istream)
  "Deserializes a message object of type '<Robotiq3FGripperRobotInput>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gACT)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gMOD)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gGTO)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gIMC)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gSTA)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gDTA)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gDTB)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gDTC)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gDTS)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gFLT)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gPRA)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gPOA)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gCUA)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gPRB)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gPOB)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gCUB)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gPRC)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gPOC)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gCUC)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gPRS)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gPOS)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gCUS)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Robotiq3FGripperRobotInput>)))
  "Returns string type for a message object of type '<Robotiq3FGripperRobotInput>"
  "robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotInput")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Robotiq3FGripperRobotInput)))
  "Returns string type for a message object of type 'Robotiq3FGripperRobotInput"
  "robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotInput")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Robotiq3FGripperRobotInput>)))
  "Returns md5sum for a message object of type '<Robotiq3FGripperRobotInput>"
  "4d0701156e580a420c48833f57bc83f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Robotiq3FGripperRobotInput)))
  "Returns md5sum for a message object of type 'Robotiq3FGripperRobotInput"
  "4d0701156e580a420c48833f57bc83f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Robotiq3FGripperRobotInput>)))
  "Returns full string definition for message of type '<Robotiq3FGripperRobotInput>"
  (cl:format cl:nil "# gACT : Initialization status, echo of the rACT bit (activation bit).~%# 0x0 - Gripper reset.~%# 0x1 - Gripper activation.~%~%uint8 gACT~%~%# gMOD : Operation mode status, echo of the rMOD bits (grasping mode requested).~%# 0x00 - Basic mode.~%# 0x01 - Pinch mode.~%# 0x02 - Wide mode.~%# 0x03 - Scissor mode.~%~%uint8 gMOD~%~%# gGTO : Action status. echo of the rGTO bit (go to bit).~%# 0x0 - Stopped (or performing activation / grasping mode change / automatic release)~%# 0x1 - Go to Position Request~%~%uint8 gGTO~%~%# gIMC : Gripper status, returns the current status of the Gripper.~%# 0x00 - Gripper is in reset (or automatic release) state. See Fault status if Gripper is activated.~%# 0x01 - Activation is in progress.~%# 0x02 - Mode change is in progress.~%# 0x03 - Activation and mode change are completed.~%~%uint8 gIMC~%~%# gSTA : Motion status, returns the current motion of the Gripper fingers.~%# 0x00 - Gripper is in motion towards requested position (only meaningful if gGTO = 1)~%# 0x01 - Gripper is stopped. One or two fingers stopped before requested position~%# 0x02 - Gripper is stopped. All fingers stopped before requested position~%# 0x03 - Gripper is stopped. All fingers reached requested position~%~%uint8 gSTA~%~%# gDTA : Finger A object detection status returns information on possible object contact from finger A.~%# 0x00 - Finger A is in motion (only meaningful if gGTO = 1).~%# 0x01 - Finger A has stopped due to a contact while opening.~%# 0x02 - Finger A has stopped due to a contact while closing.~%# 0x03 - Finger A is at requested position.~%~%uint8 gDTA~%~%# gDTB : Finger B object detection status returns information on possible object contact from finger B.~%# 0x00 - Finger B is in motion (only meaningful if gGTO = 1).~%# 0x01 - Finger B has stopped due to a contact while opening.~%# 0x02 - Finger B has stopped due to a contact while closing.~%# 0x03 - Finger B is at requested position.~%~%uint8 gDTB~%~%# gDTC : Finger C object detection status returns information on possible object contact from finger A.~%# 0x00 - Finger C is in motion (only meaningful if gGTO = 1).~%# 0x01 - Finger C has stopped due to a contact while opening.~%# 0x02 - Finger C has stopped due to a contact while closing.~%# 0x03 - Finger C is at requested position.~%~%uint8 gDTC~%~%# gDTS : Scissor object detection status returns information on possible object contact from scissor.~%# 0x00 - Scissor is in motion (only meaningful if gGTO = 1).~%# 0x01 - Scissor has stopped due to a contact while opening.~%# 0x02 - Scissor has stopped due to a contact while closing.~%# 0x03 - Scissor is at requested position.~%~%uint8 gDTS~%~%# gFLT : Fault status returns general error messages useful for troubleshooting.~%#   0x00 - No fault (fault LED off)~%#   Priority faults (fault LED off)~%#     0x05 - Action delayed, activation (reactivation) must be completed prior to action.~%#     0x06 - Action delayed, mode change must be completed prior to action.~%#     0x07 - The activation bit must be set prior to action.~%#   Minor faults (fault LED continuous red)~%#     0x09 - The communication chip is not ready (may be booting).~%#     0x0A - Changing mode fault, interferences detected on Scissor (for less than 20 sec).~%#     0x0B - Automatic release in progress.~%#   Major faults (fault LED blinking red) - Reset is required~%#     0x0D - Activation fault, verify that no interference or other error occurred.~%#     0x0E - Changing mode fault, interferences detected on Scissor (for more than 20 sec).~%#     0x0F - Automatic release completed. Reset and activation is required.~%~%uint8 gFLT~%~%~%# gPRA : Echo of the requested position of the Gripper (rPRA),~%#   0x00 is minimum position (full opening) and~%#   0xFF is maximum position (full closing).~%# If commanding the Gripper in~%#   individual control mode, gPRA is the echo of finger A,~%#   otherwise it is the general position requested to all fingers.~%~%uint8 gPRA~%~%# gPOA : Returns the actual position of the Gripper finger A,~%#   0x00 is minimum position (full opening) and~%#   0xFF is maximum position (full closing).~%~%uint8 gPOA~%~%# gCUA : Returns a value that represents the finger A instantaneous current consumption from 0x00 to 0xFF.~%~%uint8 gCUA~%~%# gPRB : Echo of the requested position of finger B (rPRB),~%#   0x00 is minimum position (full opening) and~%#   0xFF is maximum position (full closing).~%~%uint8 gPRB~%~%# gPOB : Returns the actual position of the Gripper finger B,~%#   0x00 is minimum position (full opening) and~%#   0xFF is maximum position (full closing).~%~%uint8 gPOB~%~%# gCUB : Returns a value that represents the finger B instantaneous current consumption from 0x00 to 0xFF.~%~%uint8 gCUB~%~%# gPRC : Echo of the requested position of finger C (rPRC),~%#   0x00 is minimum position (full opening) and~%#   0xFF is maximum position (full closing).~%~%uint8 gPRC~%~%# gPOC : Returns the actual position of the Gripper finger C,~%#   0x00 is minimum position (full opening) and~%#   0xFF is maximum position (full closing).~%~%uint8 gPOC~%~%# gCUC : Returns a value that represents the finger C instantaneous current consumption from 0x00 to 0xFF.~%~%uint8 gCUC~%~%# gPRS : Echo of the requested position of scissor (rPRS),~%#   0x00 is minimum position (full opening) and~%#   0xFF is maximum position (full closing).~%~%uint8 gPRS~%~%# gPOS : Returns the actual position of the Gripper scissor,~%#   0x00 is minimum position (full opening) and~%#   0xFF is maximum position (full closing).~%~%uint8 gPOS~%~%# gCUS : Returns a value that represents the scissor instantaneous current consumption from 0x00 to 0xFF.~%#   0x00 is ...~%#   0xFF is ...~%~%uint8 gCUS~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Robotiq3FGripperRobotInput)))
  "Returns full string definition for message of type 'Robotiq3FGripperRobotInput"
  (cl:format cl:nil "# gACT : Initialization status, echo of the rACT bit (activation bit).~%# 0x0 - Gripper reset.~%# 0x1 - Gripper activation.~%~%uint8 gACT~%~%# gMOD : Operation mode status, echo of the rMOD bits (grasping mode requested).~%# 0x00 - Basic mode.~%# 0x01 - Pinch mode.~%# 0x02 - Wide mode.~%# 0x03 - Scissor mode.~%~%uint8 gMOD~%~%# gGTO : Action status. echo of the rGTO bit (go to bit).~%# 0x0 - Stopped (or performing activation / grasping mode change / automatic release)~%# 0x1 - Go to Position Request~%~%uint8 gGTO~%~%# gIMC : Gripper status, returns the current status of the Gripper.~%# 0x00 - Gripper is in reset (or automatic release) state. See Fault status if Gripper is activated.~%# 0x01 - Activation is in progress.~%# 0x02 - Mode change is in progress.~%# 0x03 - Activation and mode change are completed.~%~%uint8 gIMC~%~%# gSTA : Motion status, returns the current motion of the Gripper fingers.~%# 0x00 - Gripper is in motion towards requested position (only meaningful if gGTO = 1)~%# 0x01 - Gripper is stopped. One or two fingers stopped before requested position~%# 0x02 - Gripper is stopped. All fingers stopped before requested position~%# 0x03 - Gripper is stopped. All fingers reached requested position~%~%uint8 gSTA~%~%# gDTA : Finger A object detection status returns information on possible object contact from finger A.~%# 0x00 - Finger A is in motion (only meaningful if gGTO = 1).~%# 0x01 - Finger A has stopped due to a contact while opening.~%# 0x02 - Finger A has stopped due to a contact while closing.~%# 0x03 - Finger A is at requested position.~%~%uint8 gDTA~%~%# gDTB : Finger B object detection status returns information on possible object contact from finger B.~%# 0x00 - Finger B is in motion (only meaningful if gGTO = 1).~%# 0x01 - Finger B has stopped due to a contact while opening.~%# 0x02 - Finger B has stopped due to a contact while closing.~%# 0x03 - Finger B is at requested position.~%~%uint8 gDTB~%~%# gDTC : Finger C object detection status returns information on possible object contact from finger A.~%# 0x00 - Finger C is in motion (only meaningful if gGTO = 1).~%# 0x01 - Finger C has stopped due to a contact while opening.~%# 0x02 - Finger C has stopped due to a contact while closing.~%# 0x03 - Finger C is at requested position.~%~%uint8 gDTC~%~%# gDTS : Scissor object detection status returns information on possible object contact from scissor.~%# 0x00 - Scissor is in motion (only meaningful if gGTO = 1).~%# 0x01 - Scissor has stopped due to a contact while opening.~%# 0x02 - Scissor has stopped due to a contact while closing.~%# 0x03 - Scissor is at requested position.~%~%uint8 gDTS~%~%# gFLT : Fault status returns general error messages useful for troubleshooting.~%#   0x00 - No fault (fault LED off)~%#   Priority faults (fault LED off)~%#     0x05 - Action delayed, activation (reactivation) must be completed prior to action.~%#     0x06 - Action delayed, mode change must be completed prior to action.~%#     0x07 - The activation bit must be set prior to action.~%#   Minor faults (fault LED continuous red)~%#     0x09 - The communication chip is not ready (may be booting).~%#     0x0A - Changing mode fault, interferences detected on Scissor (for less than 20 sec).~%#     0x0B - Automatic release in progress.~%#   Major faults (fault LED blinking red) - Reset is required~%#     0x0D - Activation fault, verify that no interference or other error occurred.~%#     0x0E - Changing mode fault, interferences detected on Scissor (for more than 20 sec).~%#     0x0F - Automatic release completed. Reset and activation is required.~%~%uint8 gFLT~%~%~%# gPRA : Echo of the requested position of the Gripper (rPRA),~%#   0x00 is minimum position (full opening) and~%#   0xFF is maximum position (full closing).~%# If commanding the Gripper in~%#   individual control mode, gPRA is the echo of finger A,~%#   otherwise it is the general position requested to all fingers.~%~%uint8 gPRA~%~%# gPOA : Returns the actual position of the Gripper finger A,~%#   0x00 is minimum position (full opening) and~%#   0xFF is maximum position (full closing).~%~%uint8 gPOA~%~%# gCUA : Returns a value that represents the finger A instantaneous current consumption from 0x00 to 0xFF.~%~%uint8 gCUA~%~%# gPRB : Echo of the requested position of finger B (rPRB),~%#   0x00 is minimum position (full opening) and~%#   0xFF is maximum position (full closing).~%~%uint8 gPRB~%~%# gPOB : Returns the actual position of the Gripper finger B,~%#   0x00 is minimum position (full opening) and~%#   0xFF is maximum position (full closing).~%~%uint8 gPOB~%~%# gCUB : Returns a value that represents the finger B instantaneous current consumption from 0x00 to 0xFF.~%~%uint8 gCUB~%~%# gPRC : Echo of the requested position of finger C (rPRC),~%#   0x00 is minimum position (full opening) and~%#   0xFF is maximum position (full closing).~%~%uint8 gPRC~%~%# gPOC : Returns the actual position of the Gripper finger C,~%#   0x00 is minimum position (full opening) and~%#   0xFF is maximum position (full closing).~%~%uint8 gPOC~%~%# gCUC : Returns a value that represents the finger C instantaneous current consumption from 0x00 to 0xFF.~%~%uint8 gCUC~%~%# gPRS : Echo of the requested position of scissor (rPRS),~%#   0x00 is minimum position (full opening) and~%#   0xFF is maximum position (full closing).~%~%uint8 gPRS~%~%# gPOS : Returns the actual position of the Gripper scissor,~%#   0x00 is minimum position (full opening) and~%#   0xFF is maximum position (full closing).~%~%uint8 gPOS~%~%# gCUS : Returns a value that represents the scissor instantaneous current consumption from 0x00 to 0xFF.~%#   0x00 is ...~%#   0xFF is ...~%~%uint8 gCUS~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Robotiq3FGripperRobotInput>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Robotiq3FGripperRobotInput>))
  "Converts a ROS message object to a list"
  (cl:list 'Robotiq3FGripperRobotInput
    (cl:cons ':gACT (gACT msg))
    (cl:cons ':gMOD (gMOD msg))
    (cl:cons ':gGTO (gGTO msg))
    (cl:cons ':gIMC (gIMC msg))
    (cl:cons ':gSTA (gSTA msg))
    (cl:cons ':gDTA (gDTA msg))
    (cl:cons ':gDTB (gDTB msg))
    (cl:cons ':gDTC (gDTC msg))
    (cl:cons ':gDTS (gDTS msg))
    (cl:cons ':gFLT (gFLT msg))
    (cl:cons ':gPRA (gPRA msg))
    (cl:cons ':gPOA (gPOA msg))
    (cl:cons ':gCUA (gCUA msg))
    (cl:cons ':gPRB (gPRB msg))
    (cl:cons ':gPOB (gPOB msg))
    (cl:cons ':gCUB (gCUB msg))
    (cl:cons ':gPRC (gPRC msg))
    (cl:cons ':gPOC (gPOC msg))
    (cl:cons ':gCUC (gCUC msg))
    (cl:cons ':gPRS (gPRS msg))
    (cl:cons ':gPOS (gPOS msg))
    (cl:cons ':gCUS (gCUS msg))
))
