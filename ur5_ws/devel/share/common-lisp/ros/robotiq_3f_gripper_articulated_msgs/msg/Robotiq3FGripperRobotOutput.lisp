; Auto-generated. Do not edit!


(cl:in-package robotiq_3f_gripper_articulated_msgs-msg)


;//! \htmlinclude Robotiq3FGripperRobotOutput.msg.html

(cl:defclass <Robotiq3FGripperRobotOutput> (roslisp-msg-protocol:ros-message)
  ((rACT
    :reader rACT
    :initarg :rACT
    :type cl:fixnum
    :initform 0)
   (rMOD
    :reader rMOD
    :initarg :rMOD
    :type cl:fixnum
    :initform 0)
   (rGTO
    :reader rGTO
    :initarg :rGTO
    :type cl:fixnum
    :initform 0)
   (rATR
    :reader rATR
    :initarg :rATR
    :type cl:fixnum
    :initform 0)
   (rGLV
    :reader rGLV
    :initarg :rGLV
    :type cl:fixnum
    :initform 0)
   (rICF
    :reader rICF
    :initarg :rICF
    :type cl:fixnum
    :initform 0)
   (rICS
    :reader rICS
    :initarg :rICS
    :type cl:fixnum
    :initform 0)
   (rPRA
    :reader rPRA
    :initarg :rPRA
    :type cl:fixnum
    :initform 0)
   (rSPA
    :reader rSPA
    :initarg :rSPA
    :type cl:fixnum
    :initform 0)
   (rFRA
    :reader rFRA
    :initarg :rFRA
    :type cl:fixnum
    :initform 0)
   (rPRB
    :reader rPRB
    :initarg :rPRB
    :type cl:fixnum
    :initform 0)
   (rSPB
    :reader rSPB
    :initarg :rSPB
    :type cl:fixnum
    :initform 0)
   (rFRB
    :reader rFRB
    :initarg :rFRB
    :type cl:fixnum
    :initform 0)
   (rPRC
    :reader rPRC
    :initarg :rPRC
    :type cl:fixnum
    :initform 0)
   (rSPC
    :reader rSPC
    :initarg :rSPC
    :type cl:fixnum
    :initform 0)
   (rFRC
    :reader rFRC
    :initarg :rFRC
    :type cl:fixnum
    :initform 0)
   (rPRS
    :reader rPRS
    :initarg :rPRS
    :type cl:fixnum
    :initform 0)
   (rSPS
    :reader rSPS
    :initarg :rSPS
    :type cl:fixnum
    :initform 0)
   (rFRS
    :reader rFRS
    :initarg :rFRS
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Robotiq3FGripperRobotOutput (<Robotiq3FGripperRobotOutput>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Robotiq3FGripperRobotOutput>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Robotiq3FGripperRobotOutput)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotiq_3f_gripper_articulated_msgs-msg:<Robotiq3FGripperRobotOutput> is deprecated: use robotiq_3f_gripper_articulated_msgs-msg:Robotiq3FGripperRobotOutput instead.")))

(cl:ensure-generic-function 'rACT-val :lambda-list '(m))
(cl:defmethod rACT-val ((m <Robotiq3FGripperRobotOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:rACT-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:rACT instead.")
  (rACT m))

(cl:ensure-generic-function 'rMOD-val :lambda-list '(m))
(cl:defmethod rMOD-val ((m <Robotiq3FGripperRobotOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:rMOD-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:rMOD instead.")
  (rMOD m))

(cl:ensure-generic-function 'rGTO-val :lambda-list '(m))
(cl:defmethod rGTO-val ((m <Robotiq3FGripperRobotOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:rGTO-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:rGTO instead.")
  (rGTO m))

(cl:ensure-generic-function 'rATR-val :lambda-list '(m))
(cl:defmethod rATR-val ((m <Robotiq3FGripperRobotOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:rATR-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:rATR instead.")
  (rATR m))

(cl:ensure-generic-function 'rGLV-val :lambda-list '(m))
(cl:defmethod rGLV-val ((m <Robotiq3FGripperRobotOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:rGLV-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:rGLV instead.")
  (rGLV m))

(cl:ensure-generic-function 'rICF-val :lambda-list '(m))
(cl:defmethod rICF-val ((m <Robotiq3FGripperRobotOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:rICF-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:rICF instead.")
  (rICF m))

(cl:ensure-generic-function 'rICS-val :lambda-list '(m))
(cl:defmethod rICS-val ((m <Robotiq3FGripperRobotOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:rICS-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:rICS instead.")
  (rICS m))

(cl:ensure-generic-function 'rPRA-val :lambda-list '(m))
(cl:defmethod rPRA-val ((m <Robotiq3FGripperRobotOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:rPRA-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:rPRA instead.")
  (rPRA m))

(cl:ensure-generic-function 'rSPA-val :lambda-list '(m))
(cl:defmethod rSPA-val ((m <Robotiq3FGripperRobotOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:rSPA-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:rSPA instead.")
  (rSPA m))

(cl:ensure-generic-function 'rFRA-val :lambda-list '(m))
(cl:defmethod rFRA-val ((m <Robotiq3FGripperRobotOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:rFRA-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:rFRA instead.")
  (rFRA m))

(cl:ensure-generic-function 'rPRB-val :lambda-list '(m))
(cl:defmethod rPRB-val ((m <Robotiq3FGripperRobotOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:rPRB-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:rPRB instead.")
  (rPRB m))

(cl:ensure-generic-function 'rSPB-val :lambda-list '(m))
(cl:defmethod rSPB-val ((m <Robotiq3FGripperRobotOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:rSPB-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:rSPB instead.")
  (rSPB m))

(cl:ensure-generic-function 'rFRB-val :lambda-list '(m))
(cl:defmethod rFRB-val ((m <Robotiq3FGripperRobotOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:rFRB-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:rFRB instead.")
  (rFRB m))

(cl:ensure-generic-function 'rPRC-val :lambda-list '(m))
(cl:defmethod rPRC-val ((m <Robotiq3FGripperRobotOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:rPRC-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:rPRC instead.")
  (rPRC m))

(cl:ensure-generic-function 'rSPC-val :lambda-list '(m))
(cl:defmethod rSPC-val ((m <Robotiq3FGripperRobotOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:rSPC-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:rSPC instead.")
  (rSPC m))

(cl:ensure-generic-function 'rFRC-val :lambda-list '(m))
(cl:defmethod rFRC-val ((m <Robotiq3FGripperRobotOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:rFRC-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:rFRC instead.")
  (rFRC m))

(cl:ensure-generic-function 'rPRS-val :lambda-list '(m))
(cl:defmethod rPRS-val ((m <Robotiq3FGripperRobotOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:rPRS-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:rPRS instead.")
  (rPRS m))

(cl:ensure-generic-function 'rSPS-val :lambda-list '(m))
(cl:defmethod rSPS-val ((m <Robotiq3FGripperRobotOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:rSPS-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:rSPS instead.")
  (rSPS m))

(cl:ensure-generic-function 'rFRS-val :lambda-list '(m))
(cl:defmethod rFRS-val ((m <Robotiq3FGripperRobotOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_3f_gripper_articulated_msgs-msg:rFRS-val is deprecated.  Use robotiq_3f_gripper_articulated_msgs-msg:rFRS instead.")
  (rFRS m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Robotiq3FGripperRobotOutput>) ostream)
  "Serializes a message object of type '<Robotiq3FGripperRobotOutput>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rACT)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rMOD)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rGTO)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rATR)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rGLV)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rICF)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rICS)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rPRA)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rSPA)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rFRA)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rPRB)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rSPB)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rFRB)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rPRC)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rSPC)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rFRC)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rPRS)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rSPS)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rFRS)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Robotiq3FGripperRobotOutput>) istream)
  "Deserializes a message object of type '<Robotiq3FGripperRobotOutput>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rACT)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rMOD)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rGTO)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rATR)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rGLV)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rICF)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rICS)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rPRA)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rSPA)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rFRA)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rPRB)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rSPB)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rFRB)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rPRC)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rSPC)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rFRC)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rPRS)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rSPS)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rFRS)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Robotiq3FGripperRobotOutput>)))
  "Returns string type for a message object of type '<Robotiq3FGripperRobotOutput>"
  "robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Robotiq3FGripperRobotOutput)))
  "Returns string type for a message object of type 'Robotiq3FGripperRobotOutput"
  "robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Robotiq3FGripperRobotOutput>)))
  "Returns md5sum for a message object of type '<Robotiq3FGripperRobotOutput>"
  "31ba91390a569c669af204c3d006a806")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Robotiq3FGripperRobotOutput)))
  "Returns md5sum for a message object of type 'Robotiq3FGripperRobotOutput"
  "31ba91390a569c669af204c3d006a806")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Robotiq3FGripperRobotOutput>)))
  "Returns full string definition for message of type '<Robotiq3FGripperRobotOutput>"
  (cl:format cl:nil "# rACT: Action request (activation bit).~%# 0x0 - Decativate gripper.~%# 0x1 - Activate gripper.~%~%uint8 rACT~%~%# rMOD: Changes the gripper grasping mode.~%# 0x0 - Basic mode.~%# 0x1 - Pinch mode.~%# 0x2 - Wide mode.~%# 0x3 - Scissor mode.~%~%uint8 rMOD~%~%# rGTO: \"Go To\" action moves the gripper fingers to the requested position.~%# 0x0 - Stop.~%# 0x1 - Go to the requested position.~%~%uint8 rGTO~%~%# rATR: Automatic release routine.~%# 0x0 - Stop.~%# 0x1 Emergency auto-release.~%~%uint8 rATR~%~%# rGLV: Glove mode~%# 0x0 - Glove mode off~%# 0x1 - Glove mode on~%~%uint8 rGLV~%~%# rICF: Individual Control of Fingers mode. Each finger receives its own command~%# unless the gripper is in the Scissor Grasping Mode and the Independent Control~%# of Scissor (rICS) is not activated.~%# 0x0 - Normal.~%# 0x1 - Enable Individual Control of Fingers A, B and C.~%~%uint8 rICF~%~%# rICS: Individual Control of Scissor. The scissor axis moves independenly from~%# the grasping mode. When this option is selected, the rMOD bits (grasping mode)~%# are ignored as the scissor axis position is defined by the rPRS.~%# 0x0 - Normal.~%# 0x1 - Enable Individual Control of Scissor. Disable Mode Selection.~%~%uint8 rICS~%~%# rPRA: Target position of the fingers (or finger A only if bit rICF is set).~%# 0x00 Minimum position (open).~%# 0xFF Maximum position (close).~%~%uint8 rPRA~%~%# rSPA: Gripper closing or opening speed (or finger A only if bit rICF is set).~%# Setting a speed will not initiate a motion.~%# 0x00 Minimum speed.~%# 0xFF Maximum speed.~%~%uint8 rSPA~%~%# rFRA: Final grasping force of the gripper (or finger A only if bit rICF is~%# set).~%# 0x00 Minimum force.~%# 0xFF Maximum force.~%~%uint8 rFRA~%~%# rPRB: Finger B target position. It is only available if bit rICF is set.~%# 0x00 Minimum position (open).~%# 0xFF Maximum position (close).~%~%uint8 rPRB~%~%# rSPB: Finger B speed. It is only available if bit rICF is set.~%# Setting a speed will not initiate a motion.~%# 0x00 Minimum speed.~%# 0xFF Maximum speed.~%~%uint8 rSPB~%~%# rFRB: Finger B force. It is only available if bit rICF is set.~%# 0x00 Minimum force.~%# 0xFF Maximum force.~%~%uint8 rFRB~%~%# rPRC: Finger C target position. It is only available if bit rICF is set.~%# 0x00 Minimum position (open).~%# 0xFF Maximum position (close).~%~%uint8 rPRC~%~%# rSPC: Finger C speed. It is only available if bit rICF is set.~%# Setting a speed will not initiate a motion.~%# 0x00 Minimum speed.~%# 0xFF Maximum speed.~%~%uint8 rSPC~%~%# rFRC: Finger C force. It is only available if bit rICF is set.~%# 0x00 Minimum force.~%# 0xFF Maximum force.~%~%uint8 rFRC~%~%# rPRS: Scissor axis target position. It is only applied if the Individual~%# Control of Scissor option is selected (bit rICS is set).~%# 0x00 Minimum position (open).~%# 0xFF Maximum position (close).~%~%uint8 rPRS~%~%# rSPS: Scissor axis speed. It is only applied if the Individual~%# Control of Scissor option is selected (bit rICS is set).~%# 0x00 Minimum speed.~%# 0xFF Maximum speed.~%~%uint8 rSPS~%~%# rFRS: Scissor axis force. It is only applied if the Individual~%# Control of Scissor option is selected (bit rICS is set).~%# 0x00 Minimum force.~%# 0xFF Maximum force.~%~%uint8 rFRS~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Robotiq3FGripperRobotOutput)))
  "Returns full string definition for message of type 'Robotiq3FGripperRobotOutput"
  (cl:format cl:nil "# rACT: Action request (activation bit).~%# 0x0 - Decativate gripper.~%# 0x1 - Activate gripper.~%~%uint8 rACT~%~%# rMOD: Changes the gripper grasping mode.~%# 0x0 - Basic mode.~%# 0x1 - Pinch mode.~%# 0x2 - Wide mode.~%# 0x3 - Scissor mode.~%~%uint8 rMOD~%~%# rGTO: \"Go To\" action moves the gripper fingers to the requested position.~%# 0x0 - Stop.~%# 0x1 - Go to the requested position.~%~%uint8 rGTO~%~%# rATR: Automatic release routine.~%# 0x0 - Stop.~%# 0x1 Emergency auto-release.~%~%uint8 rATR~%~%# rGLV: Glove mode~%# 0x0 - Glove mode off~%# 0x1 - Glove mode on~%~%uint8 rGLV~%~%# rICF: Individual Control of Fingers mode. Each finger receives its own command~%# unless the gripper is in the Scissor Grasping Mode and the Independent Control~%# of Scissor (rICS) is not activated.~%# 0x0 - Normal.~%# 0x1 - Enable Individual Control of Fingers A, B and C.~%~%uint8 rICF~%~%# rICS: Individual Control of Scissor. The scissor axis moves independenly from~%# the grasping mode. When this option is selected, the rMOD bits (grasping mode)~%# are ignored as the scissor axis position is defined by the rPRS.~%# 0x0 - Normal.~%# 0x1 - Enable Individual Control of Scissor. Disable Mode Selection.~%~%uint8 rICS~%~%# rPRA: Target position of the fingers (or finger A only if bit rICF is set).~%# 0x00 Minimum position (open).~%# 0xFF Maximum position (close).~%~%uint8 rPRA~%~%# rSPA: Gripper closing or opening speed (or finger A only if bit rICF is set).~%# Setting a speed will not initiate a motion.~%# 0x00 Minimum speed.~%# 0xFF Maximum speed.~%~%uint8 rSPA~%~%# rFRA: Final grasping force of the gripper (or finger A only if bit rICF is~%# set).~%# 0x00 Minimum force.~%# 0xFF Maximum force.~%~%uint8 rFRA~%~%# rPRB: Finger B target position. It is only available if bit rICF is set.~%# 0x00 Minimum position (open).~%# 0xFF Maximum position (close).~%~%uint8 rPRB~%~%# rSPB: Finger B speed. It is only available if bit rICF is set.~%# Setting a speed will not initiate a motion.~%# 0x00 Minimum speed.~%# 0xFF Maximum speed.~%~%uint8 rSPB~%~%# rFRB: Finger B force. It is only available if bit rICF is set.~%# 0x00 Minimum force.~%# 0xFF Maximum force.~%~%uint8 rFRB~%~%# rPRC: Finger C target position. It is only available if bit rICF is set.~%# 0x00 Minimum position (open).~%# 0xFF Maximum position (close).~%~%uint8 rPRC~%~%# rSPC: Finger C speed. It is only available if bit rICF is set.~%# Setting a speed will not initiate a motion.~%# 0x00 Minimum speed.~%# 0xFF Maximum speed.~%~%uint8 rSPC~%~%# rFRC: Finger C force. It is only available if bit rICF is set.~%# 0x00 Minimum force.~%# 0xFF Maximum force.~%~%uint8 rFRC~%~%# rPRS: Scissor axis target position. It is only applied if the Individual~%# Control of Scissor option is selected (bit rICS is set).~%# 0x00 Minimum position (open).~%# 0xFF Maximum position (close).~%~%uint8 rPRS~%~%# rSPS: Scissor axis speed. It is only applied if the Individual~%# Control of Scissor option is selected (bit rICS is set).~%# 0x00 Minimum speed.~%# 0xFF Maximum speed.~%~%uint8 rSPS~%~%# rFRS: Scissor axis force. It is only applied if the Individual~%# Control of Scissor option is selected (bit rICS is set).~%# 0x00 Minimum force.~%# 0xFF Maximum force.~%~%uint8 rFRS~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Robotiq3FGripperRobotOutput>))
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
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Robotiq3FGripperRobotOutput>))
  "Converts a ROS message object to a list"
  (cl:list 'Robotiq3FGripperRobotOutput
    (cl:cons ':rACT (rACT msg))
    (cl:cons ':rMOD (rMOD msg))
    (cl:cons ':rGTO (rGTO msg))
    (cl:cons ':rATR (rATR msg))
    (cl:cons ':rGLV (rGLV msg))
    (cl:cons ':rICF (rICF msg))
    (cl:cons ':rICS (rICS msg))
    (cl:cons ':rPRA (rPRA msg))
    (cl:cons ':rSPA (rSPA msg))
    (cl:cons ':rFRA (rFRA msg))
    (cl:cons ':rPRB (rPRB msg))
    (cl:cons ':rSPB (rSPB msg))
    (cl:cons ':rFRB (rFRB msg))
    (cl:cons ':rPRC (rPRC msg))
    (cl:cons ':rSPC (rSPC msg))
    (cl:cons ':rFRC (rFRC msg))
    (cl:cons ':rPRS (rPRS msg))
    (cl:cons ':rSPS (rSPS msg))
    (cl:cons ':rFRS (rFRS msg))
))
