; Auto-generated. Do not edit!


(cl:in-package ur_dashboard_msgs-srv)


;//! \htmlinclude IsInRemoteControl-request.msg.html

(cl:defclass <IsInRemoteControl-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass IsInRemoteControl-request (<IsInRemoteControl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IsInRemoteControl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IsInRemoteControl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur_dashboard_msgs-srv:<IsInRemoteControl-request> is deprecated: use ur_dashboard_msgs-srv:IsInRemoteControl-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IsInRemoteControl-request>) ostream)
  "Serializes a message object of type '<IsInRemoteControl-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IsInRemoteControl-request>) istream)
  "Deserializes a message object of type '<IsInRemoteControl-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IsInRemoteControl-request>)))
  "Returns string type for a service object of type '<IsInRemoteControl-request>"
  "ur_dashboard_msgs/IsInRemoteControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IsInRemoteControl-request)))
  "Returns string type for a service object of type 'IsInRemoteControl-request"
  "ur_dashboard_msgs/IsInRemoteControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IsInRemoteControl-request>)))
  "Returns md5sum for a message object of type '<IsInRemoteControl-request>"
  "032fdd19f824627299e7ba024ba3c0bc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IsInRemoteControl-request)))
  "Returns md5sum for a message object of type 'IsInRemoteControl-request"
  "032fdd19f824627299e7ba024ba3c0bc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IsInRemoteControl-request>)))
  "Returns full string definition for message of type '<IsInRemoteControl-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IsInRemoteControl-request)))
  "Returns full string definition for message of type 'IsInRemoteControl-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IsInRemoteControl-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IsInRemoteControl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'IsInRemoteControl-request
))
;//! \htmlinclude IsInRemoteControl-response.msg.html

(cl:defclass <IsInRemoteControl-response> (roslisp-msg-protocol:ros-message)
  ((answer
    :reader answer
    :initarg :answer
    :type cl:string
    :initform "")
   (in_remote_control
    :reader in_remote_control
    :initarg :in_remote_control
    :type cl:boolean
    :initform cl:nil)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass IsInRemoteControl-response (<IsInRemoteControl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IsInRemoteControl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IsInRemoteControl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur_dashboard_msgs-srv:<IsInRemoteControl-response> is deprecated: use ur_dashboard_msgs-srv:IsInRemoteControl-response instead.")))

(cl:ensure-generic-function 'answer-val :lambda-list '(m))
(cl:defmethod answer-val ((m <IsInRemoteControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_dashboard_msgs-srv:answer-val is deprecated.  Use ur_dashboard_msgs-srv:answer instead.")
  (answer m))

(cl:ensure-generic-function 'in_remote_control-val :lambda-list '(m))
(cl:defmethod in_remote_control-val ((m <IsInRemoteControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_dashboard_msgs-srv:in_remote_control-val is deprecated.  Use ur_dashboard_msgs-srv:in_remote_control instead.")
  (in_remote_control m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <IsInRemoteControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_dashboard_msgs-srv:success-val is deprecated.  Use ur_dashboard_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IsInRemoteControl-response>) ostream)
  "Serializes a message object of type '<IsInRemoteControl-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'answer))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'answer))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'in_remote_control) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IsInRemoteControl-response>) istream)
  "Deserializes a message object of type '<IsInRemoteControl-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'answer) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'answer) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'in_remote_control) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IsInRemoteControl-response>)))
  "Returns string type for a service object of type '<IsInRemoteControl-response>"
  "ur_dashboard_msgs/IsInRemoteControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IsInRemoteControl-response)))
  "Returns string type for a service object of type 'IsInRemoteControl-response"
  "ur_dashboard_msgs/IsInRemoteControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IsInRemoteControl-response>)))
  "Returns md5sum for a message object of type '<IsInRemoteControl-response>"
  "032fdd19f824627299e7ba024ba3c0bc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IsInRemoteControl-response)))
  "Returns md5sum for a message object of type 'IsInRemoteControl-response"
  "032fdd19f824627299e7ba024ba3c0bc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IsInRemoteControl-response>)))
  "Returns full string definition for message of type '<IsInRemoteControl-response>"
  (cl:format cl:nil "string answer~%bool in_remote_control # is the robot currently in remote control mode?~%bool success # Did the dashboard server call succeed?~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IsInRemoteControl-response)))
  "Returns full string definition for message of type 'IsInRemoteControl-response"
  (cl:format cl:nil "string answer~%bool in_remote_control # is the robot currently in remote control mode?~%bool success # Did the dashboard server call succeed?~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IsInRemoteControl-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'answer))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IsInRemoteControl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'IsInRemoteControl-response
    (cl:cons ':answer (answer msg))
    (cl:cons ':in_remote_control (in_remote_control msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'IsInRemoteControl)))
  'IsInRemoteControl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'IsInRemoteControl)))
  'IsInRemoteControl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IsInRemoteControl)))
  "Returns string type for a service object of type '<IsInRemoteControl>"
  "ur_dashboard_msgs/IsInRemoteControl")