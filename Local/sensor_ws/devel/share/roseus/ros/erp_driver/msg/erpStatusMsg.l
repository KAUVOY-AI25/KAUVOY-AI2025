;; Auto-generated. Do not edit!


(when (boundp 'erp_driver::erpStatusMsg)
  (if (not (find-package "ERP_DRIVER"))
    (make-package "ERP_DRIVER"))
  (shadow 'erpStatusMsg (find-package "ERP_DRIVER")))
(unless (find-package "ERP_DRIVER::ERPSTATUSMSG")
  (make-package "ERP_DRIVER::ERPSTATUSMSG"))

(in-package "ROS")
;;//! \htmlinclude erpStatusMsg.msg.html


(defclass erp_driver::erpStatusMsg
  :super ros::object
  :slots (_control_mode _e_stop _gear _speed _steer _brake _encoder _alive ))

(defmethod erp_driver::erpStatusMsg
  (:init
   (&key
    ((:control_mode __control_mode) 0)
    ((:e_stop __e_stop) nil)
    ((:gear __gear) 0)
    ((:speed __speed) 0)
    ((:steer __steer) 0)
    ((:brake __brake) 0)
    ((:encoder __encoder) 0)
    ((:alive __alive) 0)
    )
   (send-super :init)
   (setq _control_mode (round __control_mode))
   (setq _e_stop __e_stop)
   (setq _gear (round __gear))
   (setq _speed (round __speed))
   (setq _steer (round __steer))
   (setq _brake (round __brake))
   (setq _encoder (round __encoder))
   (setq _alive (round __alive))
   self)
  (:control_mode
   (&optional __control_mode)
   (if __control_mode (setq _control_mode __control_mode)) _control_mode)
  (:e_stop
   (&optional (__e_stop :null))
   (if (not (eq __e_stop :null)) (setq _e_stop __e_stop)) _e_stop)
  (:gear
   (&optional __gear)
   (if __gear (setq _gear __gear)) _gear)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:steer
   (&optional __steer)
   (if __steer (setq _steer __steer)) _steer)
  (:brake
   (&optional __brake)
   (if __brake (setq _brake __brake)) _brake)
  (:encoder
   (&optional __encoder)
   (if __encoder (setq _encoder __encoder)) _encoder)
  (:alive
   (&optional __alive)
   (if __alive (setq _alive __alive)) _alive)
  (:serialization-length
   ()
   (+
    ;; int8 _control_mode
    1
    ;; bool _e_stop
    1
    ;; uint8 _gear
    1
    ;; uint8 _speed
    1
    ;; int32 _steer
    4
    ;; uint8 _brake
    1
    ;; int32 _encoder
    4
    ;; uint8 _alive
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _control_mode
       (write-byte _control_mode s)
     ;; bool _e_stop
       (if _e_stop (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _gear
       (write-byte _gear s)
     ;; uint8 _speed
       (write-byte _speed s)
     ;; int32 _steer
       (write-long _steer s)
     ;; uint8 _brake
       (write-byte _brake s)
     ;; int32 _encoder
       (write-long _encoder s)
     ;; uint8 _alive
       (write-byte _alive s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _control_mode
     (setq _control_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _control_mode 127) (setq _control_mode (- _control_mode 256)))
   ;; bool _e_stop
     (setq _e_stop (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _gear
     (setq _gear (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _speed
     (setq _speed (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int32 _steer
     (setq _steer (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _brake
     (setq _brake (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int32 _encoder
     (setq _encoder (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _alive
     (setq _alive (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get erp_driver::erpStatusMsg :md5sum-) "c9878ccc909e763ca6b607c34a8914ba")
(setf (get erp_driver::erpStatusMsg :datatype-) "erp_driver/erpStatusMsg")
(setf (get erp_driver::erpStatusMsg :definition-)
      "int8 control_mode
bool e_stop
uint8 gear
uint8 speed
int32 steer
uint8 brake
int32 encoder
uint8 alive
")



(provide :erp_driver/erpStatusMsg "c9878ccc909e763ca6b607c34a8914ba")


