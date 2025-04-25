;; Auto-generated. Do not edit!


(when (boundp 'assignment1::SensorFusion)
  (if (not (find-package "ASSIGNMENT1"))
    (make-package "ASSIGNMENT1"))
  (shadow 'SensorFusion (find-package "ASSIGNMENT1")))
(unless (find-package "ASSIGNMENT1::SENSORFUSION")
  (make-package "ASSIGNMENT1::SENSORFUSION"))

(in-package "ROS")
;;//! \htmlinclude SensorFusion.msg.html


(defclass assignment1::SensorFusion
  :super ros::object
  :slots (_FusedData ))

(defmethod assignment1::SensorFusion
  (:init
   (&key
    ((:FusedData __FusedData) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _FusedData __FusedData)
   self)
  (:FusedData
   (&optional __FusedData)
   (if __FusedData (setq _FusedData __FusedData)) _FusedData)
  (:serialization-length
   ()
   (+
    ;; float32[] _FusedData
    (* 4    (length _FusedData)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _FusedData
     (write-long (length _FusedData) s)
     (dotimes (i (length _FusedData))
       (sys::poke (elt _FusedData i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _FusedData
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _FusedData (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _FusedData i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get assignment1::SensorFusion :md5sum-) "370f547decfe0b4a69841f116fc2c0fe")
(setf (get assignment1::SensorFusion :datatype-) "assignment1/SensorFusion")
(setf (get assignment1::SensorFusion :definition-)
      "# SensorFusion.msg

float32[] FusedData
")



(provide :assignment1/SensorFusion "370f547decfe0b4a69841f116fc2c0fe")


