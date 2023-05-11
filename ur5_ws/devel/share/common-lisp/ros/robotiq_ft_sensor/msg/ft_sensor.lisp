; Auto-generated. Do not edit!


(cl:in-package robotiq_ft_sensor-msg)


;//! \htmlinclude ft_sensor.msg.html

(cl:defclass <ft_sensor> (roslisp-msg-protocol:ros-message)
  ((Fx
    :reader Fx
    :initarg :Fx
    :type cl:float
    :initform 0.0)
   (Fy
    :reader Fy
    :initarg :Fy
    :type cl:float
    :initform 0.0)
   (Fz
    :reader Fz
    :initarg :Fz
    :type cl:float
    :initform 0.0)
   (Mx
    :reader Mx
    :initarg :Mx
    :type cl:float
    :initform 0.0)
   (My
    :reader My
    :initarg :My
    :type cl:float
    :initform 0.0)
   (Mz
    :reader Mz
    :initarg :Mz
    :type cl:float
    :initform 0.0))
)

(cl:defclass ft_sensor (<ft_sensor>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ft_sensor>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ft_sensor)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotiq_ft_sensor-msg:<ft_sensor> is deprecated: use robotiq_ft_sensor-msg:ft_sensor instead.")))

(cl:ensure-generic-function 'Fx-val :lambda-list '(m))
(cl:defmethod Fx-val ((m <ft_sensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_ft_sensor-msg:Fx-val is deprecated.  Use robotiq_ft_sensor-msg:Fx instead.")
  (Fx m))

(cl:ensure-generic-function 'Fy-val :lambda-list '(m))
(cl:defmethod Fy-val ((m <ft_sensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_ft_sensor-msg:Fy-val is deprecated.  Use robotiq_ft_sensor-msg:Fy instead.")
  (Fy m))

(cl:ensure-generic-function 'Fz-val :lambda-list '(m))
(cl:defmethod Fz-val ((m <ft_sensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_ft_sensor-msg:Fz-val is deprecated.  Use robotiq_ft_sensor-msg:Fz instead.")
  (Fz m))

(cl:ensure-generic-function 'Mx-val :lambda-list '(m))
(cl:defmethod Mx-val ((m <ft_sensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_ft_sensor-msg:Mx-val is deprecated.  Use robotiq_ft_sensor-msg:Mx instead.")
  (Mx m))

(cl:ensure-generic-function 'My-val :lambda-list '(m))
(cl:defmethod My-val ((m <ft_sensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_ft_sensor-msg:My-val is deprecated.  Use robotiq_ft_sensor-msg:My instead.")
  (My m))

(cl:ensure-generic-function 'Mz-val :lambda-list '(m))
(cl:defmethod Mz-val ((m <ft_sensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotiq_ft_sensor-msg:Mz-val is deprecated.  Use robotiq_ft_sensor-msg:Mz instead.")
  (Mz m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ft_sensor>) ostream)
  "Serializes a message object of type '<ft_sensor>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Fx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Fy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Fz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Mx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'My))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Mz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ft_sensor>) istream)
  "Deserializes a message object of type '<ft_sensor>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Fx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Fy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Fz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Mx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'My) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Mz) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ft_sensor>)))
  "Returns string type for a message object of type '<ft_sensor>"
  "robotiq_ft_sensor/ft_sensor")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ft_sensor)))
  "Returns string type for a message object of type 'ft_sensor"
  "robotiq_ft_sensor/ft_sensor")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ft_sensor>)))
  "Returns md5sum for a message object of type '<ft_sensor>"
  "060fd5fcbaefcff9daec9d5bb2d41714")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ft_sensor)))
  "Returns md5sum for a message object of type 'ft_sensor"
  "060fd5fcbaefcff9daec9d5bb2d41714")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ft_sensor>)))
  "Returns full string definition for message of type '<ft_sensor>"
  (cl:format cl:nil "float32 Fx~%float32 Fy~%float32 Fz~%float32 Mx~%float32 My~%float32 Mz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ft_sensor)))
  "Returns full string definition for message of type 'ft_sensor"
  (cl:format cl:nil "float32 Fx~%float32 Fy~%float32 Fz~%float32 Mx~%float32 My~%float32 Mz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ft_sensor>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ft_sensor>))
  "Converts a ROS message object to a list"
  (cl:list 'ft_sensor
    (cl:cons ':Fx (Fx msg))
    (cl:cons ':Fy (Fy msg))
    (cl:cons ':Fz (Fz msg))
    (cl:cons ':Mx (Mx msg))
    (cl:cons ':My (My msg))
    (cl:cons ':Mz (Mz msg))
))
