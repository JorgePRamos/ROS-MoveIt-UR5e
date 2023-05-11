;; Auto-generated. Do not edit!


(when (boundp 'robotiq_ft_sensor::sensor_accessor)
  (if (not (find-package "ROBOTIQ_FT_SENSOR"))
    (make-package "ROBOTIQ_FT_SENSOR"))
  (shadow 'sensor_accessor (find-package "ROBOTIQ_FT_SENSOR")))
(unless (find-package "ROBOTIQ_FT_SENSOR::SENSOR_ACCESSOR")
  (make-package "ROBOTIQ_FT_SENSOR::SENSOR_ACCESSOR"))
(unless (find-package "ROBOTIQ_FT_SENSOR::SENSOR_ACCESSORREQUEST")
  (make-package "ROBOTIQ_FT_SENSOR::SENSOR_ACCESSORREQUEST"))
(unless (find-package "ROBOTIQ_FT_SENSOR::SENSOR_ACCESSORRESPONSE")
  (make-package "ROBOTIQ_FT_SENSOR::SENSOR_ACCESSORRESPONSE"))

(in-package "ROS")





(intern "*COMMAND_GET_SERIAL_NUMBER*" (find-package "ROBOTIQ_FT_SENSOR::SENSOR_ACCESSORREQUEST"))
(shadow '*COMMAND_GET_SERIAL_NUMBER* (find-package "ROBOTIQ_FT_SENSOR::SENSOR_ACCESSORREQUEST"))
(defconstant robotiq_ft_sensor::sensor_accessorRequest::*COMMAND_GET_SERIAL_NUMBER* 1)
(intern "*COMMAND_GET_FIRMWARE_VERSION*" (find-package "ROBOTIQ_FT_SENSOR::SENSOR_ACCESSORREQUEST"))
(shadow '*COMMAND_GET_FIRMWARE_VERSION* (find-package "ROBOTIQ_FT_SENSOR::SENSOR_ACCESSORREQUEST"))
(defconstant robotiq_ft_sensor::sensor_accessorRequest::*COMMAND_GET_FIRMWARE_VERSION* 2)
(intern "*COMMAND_GET_PRODUCTION_YEAR*" (find-package "ROBOTIQ_FT_SENSOR::SENSOR_ACCESSORREQUEST"))
(shadow '*COMMAND_GET_PRODUCTION_YEAR* (find-package "ROBOTIQ_FT_SENSOR::SENSOR_ACCESSORREQUEST"))
(defconstant robotiq_ft_sensor::sensor_accessorRequest::*COMMAND_GET_PRODUCTION_YEAR* 4)
(intern "*COMMAND_SET_ZERO*" (find-package "ROBOTIQ_FT_SENSOR::SENSOR_ACCESSORREQUEST"))
(shadow '*COMMAND_SET_ZERO* (find-package "ROBOTIQ_FT_SENSOR::SENSOR_ACCESSORREQUEST"))
(defconstant robotiq_ft_sensor::sensor_accessorRequest::*COMMAND_SET_ZERO* 8)

(defun robotiq_ft_sensor::sensor_accessorRequest-to-symbol (const)
  (cond
        ((= const 1) 'robotiq_ft_sensor::sensor_accessorRequest::*COMMAND_GET_SERIAL_NUMBER*)
        ((= const 2) 'robotiq_ft_sensor::sensor_accessorRequest::*COMMAND_GET_FIRMWARE_VERSION*)
        ((= const 4) 'robotiq_ft_sensor::sensor_accessorRequest::*COMMAND_GET_PRODUCTION_YEAR*)
        ((= const 8) 'robotiq_ft_sensor::sensor_accessorRequest::*COMMAND_SET_ZERO*)
        (t nil)))

(defclass robotiq_ft_sensor::sensor_accessorRequest
  :super ros::object
  :slots (_command_id _command ))

(defmethod robotiq_ft_sensor::sensor_accessorRequest
  (:init
   (&key
    ((:command_id __command_id) 0)
    ((:command __command) "")
    )
   (send-super :init)
   (setq _command_id (round __command_id))
   (setq _command (string __command))
   self)
  (:command_id
   (&optional __command_id)
   (if __command_id (setq _command_id __command_id)) _command_id)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:serialization-length
   ()
   (+
    ;; uint8 _command_id
    1
    ;; string _command
    4 (length _command)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _command_id
       (write-byte _command_id s)
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _command_id
     (setq _command_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass robotiq_ft_sensor::sensor_accessorResponse
  :super ros::object
  :slots (_success _res ))

(defmethod robotiq_ft_sensor::sensor_accessorResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:res __res) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _res (string __res))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:res
   (&optional __res)
   (if __res (setq _res __res)) _res)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _res
    4 (length _res)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _res
       (write-long (length _res) s) (princ _res s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _res
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _res (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass robotiq_ft_sensor::sensor_accessor
  :super ros::object
  :slots ())

(setf (get robotiq_ft_sensor::sensor_accessor :md5sum-) "c8c699ce907261a9c404410e16c0ef0d")
(setf (get robotiq_ft_sensor::sensor_accessor :datatype-) "robotiq_ft_sensor/sensor_accessor")
(setf (get robotiq_ft_sensor::sensor_accessor :request) robotiq_ft_sensor::sensor_accessorRequest)
(setf (get robotiq_ft_sensor::sensor_accessor :response) robotiq_ft_sensor::sensor_accessorResponse)

(defmethod robotiq_ft_sensor::sensor_accessorRequest
  (:response () (instance robotiq_ft_sensor::sensor_accessorResponse :init)))

(setf (get robotiq_ft_sensor::sensor_accessorRequest :md5sum-) "c8c699ce907261a9c404410e16c0ef0d")
(setf (get robotiq_ft_sensor::sensor_accessorRequest :datatype-) "robotiq_ft_sensor/sensor_accessorRequest")
(setf (get robotiq_ft_sensor::sensor_accessorRequest :definition-)
      "uint8 COMMAND_GET_SERIAL_NUMBER=1
uint8 COMMAND_GET_FIRMWARE_VERSION=2
uint8 COMMAND_GET_PRODUCTION_YEAR=4
uint8 COMMAND_SET_ZERO=8
uint8 command_id
string command  # deprecated, please use command_id with a value of COMMAND_*
---
bool success
string res

")

(setf (get robotiq_ft_sensor::sensor_accessorResponse :md5sum-) "c8c699ce907261a9c404410e16c0ef0d")
(setf (get robotiq_ft_sensor::sensor_accessorResponse :datatype-) "robotiq_ft_sensor/sensor_accessorResponse")
(setf (get robotiq_ft_sensor::sensor_accessorResponse :definition-)
      "uint8 COMMAND_GET_SERIAL_NUMBER=1
uint8 COMMAND_GET_FIRMWARE_VERSION=2
uint8 COMMAND_GET_PRODUCTION_YEAR=4
uint8 COMMAND_SET_ZERO=8
uint8 command_id
string command  # deprecated, please use command_id with a value of COMMAND_*
---
bool success
string res

")



(provide :robotiq_ft_sensor/sensor_accessor "c8c699ce907261a9c404410e16c0ef0d")


