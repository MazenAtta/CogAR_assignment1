; Auto-generated. Do not edit!


(cl:in-package assignment1-srv)


;//! \htmlinclude Speaker-request.msg.html

(cl:defclass <Speaker-request> (roslisp-msg-protocol:ros-message)
  ((message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass Speaker-request (<Speaker-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Speaker-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Speaker-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assignment1-srv:<Speaker-request> is deprecated: use assignment1-srv:Speaker-request instead.")))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <Speaker-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment1-srv:message-val is deprecated.  Use assignment1-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Speaker-request>) ostream)
  "Serializes a message object of type '<Speaker-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Speaker-request>) istream)
  "Deserializes a message object of type '<Speaker-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Speaker-request>)))
  "Returns string type for a service object of type '<Speaker-request>"
  "assignment1/SpeakerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Speaker-request)))
  "Returns string type for a service object of type 'Speaker-request"
  "assignment1/SpeakerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Speaker-request>)))
  "Returns md5sum for a message object of type '<Speaker-request>"
  "8b7095eb8dcd517ba7c37a0a06dcc50b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Speaker-request)))
  "Returns md5sum for a message object of type 'Speaker-request"
  "8b7095eb8dcd517ba7c37a0a06dcc50b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Speaker-request>)))
  "Returns full string definition for message of type '<Speaker-request>"
  (cl:format cl:nil "# String.srv~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Speaker-request)))
  "Returns full string definition for message of type 'Speaker-request"
  (cl:format cl:nil "# String.srv~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Speaker-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Speaker-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Speaker-request
    (cl:cons ':message (message msg))
))
;//! \htmlinclude Speaker-response.msg.html

(cl:defclass <Speaker-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Speaker-response (<Speaker-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Speaker-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Speaker-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assignment1-srv:<Speaker-response> is deprecated: use assignment1-srv:Speaker-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Speaker-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment1-srv:success-val is deprecated.  Use assignment1-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Speaker-response>) ostream)
  "Serializes a message object of type '<Speaker-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Speaker-response>) istream)
  "Deserializes a message object of type '<Speaker-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Speaker-response>)))
  "Returns string type for a service object of type '<Speaker-response>"
  "assignment1/SpeakerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Speaker-response)))
  "Returns string type for a service object of type 'Speaker-response"
  "assignment1/SpeakerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Speaker-response>)))
  "Returns md5sum for a message object of type '<Speaker-response>"
  "8b7095eb8dcd517ba7c37a0a06dcc50b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Speaker-response)))
  "Returns md5sum for a message object of type 'Speaker-response"
  "8b7095eb8dcd517ba7c37a0a06dcc50b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Speaker-response>)))
  "Returns full string definition for message of type '<Speaker-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Speaker-response)))
  "Returns full string definition for message of type 'Speaker-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Speaker-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Speaker-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Speaker-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Speaker)))
  'Speaker-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Speaker)))
  'Speaker-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Speaker)))
  "Returns string type for a service object of type '<Speaker>"
  "assignment1/Speaker")