;; Auto-generated. Do not edit!


(when (boundp 'robotiq_ft_sensor::ft_sensor)
  (if (not (find-package "ROBOTIQ_FT_SENSOR"))
    (make-package "ROBOTIQ_FT_SENSOR"))
  (shadow 'ft_sensor (find-package "ROBOTIQ_FT_SENSOR")))
(unless (find-package "ROBOTIQ_FT_SENSOR::FT_SENSOR")
  (make-package "ROBOTIQ_FT_SENSOR::FT_SENSOR"))

(in-package "ROS")
;;//! \htmlinclude ft_sensor.msg.html


(defclass robotiq_ft_sensor::ft_sensor
  :super ros::object
  :slots (_Fx _Fy _Fz _Mx _My _Mz ))

(defmethod robotiq_ft_sensor::ft_sensor
  (:init
   (&key
    ((:Fx __Fx) 0.0)
    ((:Fy __Fy) 0.0)
    ((:Fz __Fz) 0.0)
    ((:Mx __Mx) 0.0)
    ((:My __My) 0.0)
    ((:Mz __Mz) 0.0)
    )
   (send-super :init)
   (setq _Fx (float __Fx))
   (setq _Fy (float __Fy))
   (setq _Fz (float __Fz))
   (setq _Mx (float __Mx))
   (setq _My (float __My))
   (setq _Mz (float __Mz))
   self)
  (:Fx
   (&optional __Fx)
   (if __Fx (setq _Fx __Fx)) _Fx)
  (:Fy
   (&optional __Fy)
   (if __Fy (setq _Fy __Fy)) _Fy)
  (:Fz
   (&optional __Fz)
   (if __Fz (setq _Fz __Fz)) _Fz)
  (:Mx
   (&optional __Mx)
   (if __Mx (setq _Mx __Mx)) _Mx)
  (:My
   (&optional __My)
   (if __My (setq _My __My)) _My)
  (:Mz
   (&optional __Mz)
   (if __Mz (setq _Mz __Mz)) _Mz)
  (:serialization-length
   ()
   (+
    ;; float32 _Fx
    4
    ;; float32 _Fy
    4
    ;; float32 _Fz
    4
    ;; float32 _Mx
    4
    ;; float32 _My
    4
    ;; float32 _Mz
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _Fx
       (sys::poke _Fx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Fy
       (sys::poke _Fy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Fz
       (sys::poke _Fz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Mx
       (sys::poke _Mx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _My
       (sys::poke _My (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Mz
       (sys::poke _Mz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _Fx
     (setq _Fx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Fy
     (setq _Fy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Fz
     (setq _Fz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Mx
     (setq _Mx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _My
     (setq _My (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Mz
     (setq _Mz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robotiq_ft_sensor::ft_sensor :md5sum-) "060fd5fcbaefcff9daec9d5bb2d41714")
(setf (get robotiq_ft_sensor::ft_sensor :datatype-) "robotiq_ft_sensor/ft_sensor")
(setf (get robotiq_ft_sensor::ft_sensor :definition-)
      "float32 Fx
float32 Fy
float32 Fz
float32 Mx
float32 My
float32 Mz

")



(provide :robotiq_ft_sensor/ft_sensor "060fd5fcbaefcff9daec9d5bb2d41714")


