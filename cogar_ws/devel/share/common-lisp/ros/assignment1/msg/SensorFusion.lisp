; Auto-generated. Do not edit!


(cl:in-package assignment1-msg)


;//! \htmlinclude SensorFusion.msg.html

(cl:defclass <SensorFusion> (roslisp-msg-protocol:ros-message)
  ((FusedData
    :reader FusedData
    :initarg :FusedData
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SensorFusion (<SensorFusion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SensorFusion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SensorFusion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assignment1-msg:<SensorFusion> is deprecated: use assignment1-msg:SensorFusion instead.")))

(cl:ensure-generic-function 'FusedData-val :lambda-list '(m))
(cl:defmethod FusedData-val ((m <SensorFusion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment1-msg:FusedData-val is deprecated.  Use assignment1-msg:FusedData instead.")
  (FusedData m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SensorFusion>) ostream)
  "Serializes a message object of type '<SensorFusion>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'FusedData))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'FusedData))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SensorFusion>) istream)
  "Deserializes a message object of type '<SensorFusion>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'FusedData) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'FusedData)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SensorFusion>)))
  "Returns string type for a message object of type '<SensorFusion>"
  "assignment1/SensorFusion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SensorFusion)))
  "Returns string type for a message object of type 'SensorFusion"
  "assignment1/SensorFusion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SensorFusion>)))
  "Returns md5sum for a message object of type '<SensorFusion>"
  "370f547decfe0b4a69841f116fc2c0fe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SensorFusion)))
  "Returns md5sum for a message object of type 'SensorFusion"
  "370f547decfe0b4a69841f116fc2c0fe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SensorFusion>)))
  "Returns full string definition for message of type '<SensorFusion>"
  (cl:format cl:nil "# SensorFusion.msg~%~%float32[] FusedData~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SensorFusion)))
  "Returns full string definition for message of type 'SensorFusion"
  (cl:format cl:nil "# SensorFusion.msg~%~%float32[] FusedData~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SensorFusion>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'FusedData) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SensorFusion>))
  "Converts a ROS message object to a list"
  (cl:list 'SensorFusion
    (cl:cons ':FusedData (FusedData msg))
))
