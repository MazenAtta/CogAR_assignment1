;; Auto-generated. Do not edit!


(when (boundp 'assignment1::Speaker)
  (if (not (find-package "ASSIGNMENT1"))
    (make-package "ASSIGNMENT1"))
  (shadow 'Speaker (find-package "ASSIGNMENT1")))
(unless (find-package "ASSIGNMENT1::SPEAKER")
  (make-package "ASSIGNMENT1::SPEAKER"))
(unless (find-package "ASSIGNMENT1::SPEAKERREQUEST")
  (make-package "ASSIGNMENT1::SPEAKERREQUEST"))
(unless (find-package "ASSIGNMENT1::SPEAKERRESPONSE")
  (make-package "ASSIGNMENT1::SPEAKERRESPONSE"))

(in-package "ROS")





(defclass assignment1::SpeakerRequest
  :super ros::object
  :slots (_message ))

(defmethod assignment1::SpeakerRequest
  (:init
   (&key
    ((:message __message) "")
    )
   (send-super :init)
   (setq _message (string __message))
   self)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass assignment1::SpeakerResponse
  :super ros::object
  :slots (_success ))

(defmethod assignment1::SpeakerResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass assignment1::Speaker
  :super ros::object
  :slots ())

(setf (get assignment1::Speaker :md5sum-) "8b7095eb8dcd517ba7c37a0a06dcc50b")
(setf (get assignment1::Speaker :datatype-) "assignment1/Speaker")
(setf (get assignment1::Speaker :request) assignment1::SpeakerRequest)
(setf (get assignment1::Speaker :response) assignment1::SpeakerResponse)

(defmethod assignment1::SpeakerRequest
  (:response () (instance assignment1::SpeakerResponse :init)))

(setf (get assignment1::SpeakerRequest :md5sum-) "8b7095eb8dcd517ba7c37a0a06dcc50b")
(setf (get assignment1::SpeakerRequest :datatype-) "assignment1/SpeakerRequest")
(setf (get assignment1::SpeakerRequest :definition-)
      "# String.srv
string message

---
bool success
")

(setf (get assignment1::SpeakerResponse :md5sum-) "8b7095eb8dcd517ba7c37a0a06dcc50b")
(setf (get assignment1::SpeakerResponse :datatype-) "assignment1/SpeakerResponse")
(setf (get assignment1::SpeakerResponse :definition-)
      "# String.srv
string message

---
bool success
")



(provide :assignment1/Speaker "8b7095eb8dcd517ba7c37a0a06dcc50b")


