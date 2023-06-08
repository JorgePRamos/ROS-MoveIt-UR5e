;; Auto-generated. Do not edit!


(when (boundp 'ur_dashboard_msgs::IsInRemoteControl)
  (if (not (find-package "UR_DASHBOARD_MSGS"))
    (make-package "UR_DASHBOARD_MSGS"))
  (shadow 'IsInRemoteControl (find-package "UR_DASHBOARD_MSGS")))
(unless (find-package "UR_DASHBOARD_MSGS::ISINREMOTECONTROL")
  (make-package "UR_DASHBOARD_MSGS::ISINREMOTECONTROL"))
(unless (find-package "UR_DASHBOARD_MSGS::ISINREMOTECONTROLREQUEST")
  (make-package "UR_DASHBOARD_MSGS::ISINREMOTECONTROLREQUEST"))
(unless (find-package "UR_DASHBOARD_MSGS::ISINREMOTECONTROLRESPONSE")
  (make-package "UR_DASHBOARD_MSGS::ISINREMOTECONTROLRESPONSE"))

(in-package "ROS")





(defclass ur_dashboard_msgs::IsInRemoteControlRequest
  :super ros::object
  :slots ())

(defmethod ur_dashboard_msgs::IsInRemoteControlRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass ur_dashboard_msgs::IsInRemoteControlResponse
  :super ros::object
  :slots (_answer _in_remote_control _success ))

(defmethod ur_dashboard_msgs::IsInRemoteControlResponse
  (:init
   (&key
    ((:answer __answer) "")
    ((:in_remote_control __in_remote_control) nil)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _answer (string __answer))
   (setq _in_remote_control __in_remote_control)
   (setq _success __success)
   self)
  (:answer
   (&optional __answer)
   (if __answer (setq _answer __answer)) _answer)
  (:in_remote_control
   (&optional (__in_remote_control :null))
   (if (not (eq __in_remote_control :null)) (setq _in_remote_control __in_remote_control)) _in_remote_control)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; string _answer
    4 (length _answer)
    ;; bool _in_remote_control
    1
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _answer
       (write-long (length _answer) s) (princ _answer s)
     ;; bool _in_remote_control
       (if _in_remote_control (write-byte -1 s) (write-byte 0 s))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _answer
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _answer (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _in_remote_control
     (setq _in_remote_control (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ur_dashboard_msgs::IsInRemoteControl
  :super ros::object
  :slots ())

(setf (get ur_dashboard_msgs::IsInRemoteControl :md5sum-) "032fdd19f824627299e7ba024ba3c0bc")
(setf (get ur_dashboard_msgs::IsInRemoteControl :datatype-) "ur_dashboard_msgs/IsInRemoteControl")
(setf (get ur_dashboard_msgs::IsInRemoteControl :request) ur_dashboard_msgs::IsInRemoteControlRequest)
(setf (get ur_dashboard_msgs::IsInRemoteControl :response) ur_dashboard_msgs::IsInRemoteControlResponse)

(defmethod ur_dashboard_msgs::IsInRemoteControlRequest
  (:response () (instance ur_dashboard_msgs::IsInRemoteControlResponse :init)))

(setf (get ur_dashboard_msgs::IsInRemoteControlRequest :md5sum-) "032fdd19f824627299e7ba024ba3c0bc")
(setf (get ur_dashboard_msgs::IsInRemoteControlRequest :datatype-) "ur_dashboard_msgs/IsInRemoteControlRequest")
(setf (get ur_dashboard_msgs::IsInRemoteControlRequest :definition-)
      "---
string answer
bool in_remote_control # is the robot currently in remote control mode?
bool success # Did the dashboard server call succeed?

")

(setf (get ur_dashboard_msgs::IsInRemoteControlResponse :md5sum-) "032fdd19f824627299e7ba024ba3c0bc")
(setf (get ur_dashboard_msgs::IsInRemoteControlResponse :datatype-) "ur_dashboard_msgs/IsInRemoteControlResponse")
(setf (get ur_dashboard_msgs::IsInRemoteControlResponse :definition-)
      "---
string answer
bool in_remote_control # is the robot currently in remote control mode?
bool success # Did the dashboard server call succeed?

")



(provide :ur_dashboard_msgs/IsInRemoteControl "032fdd19f824627299e7ba024ba3c0bc")


