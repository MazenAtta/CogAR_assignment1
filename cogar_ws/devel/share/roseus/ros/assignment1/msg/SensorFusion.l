;; Auto-generated. Do not edit!


(when (boundp 'assignment1::SensorFusion)
  (if (not (find-package "ASSIGNMENT1"))
    (make-package "ASSIGNMENT1"))
  (shadow 'SensorFusion (find-package "ASSIGNMENT1")))
(unless (find-package "ASSIGNMENT1::SENSORFUSION")
  (make-package "ASSIGNMENT1::SENSORFUSION"))

(in-package "ROS")
;;//! \htmlinclude SensorFusion.msg.html
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass assignment1::SensorFusion
  :super ros::object
  :slots (_header _pointcloud _image _odom _imu_orientation _imu_angular_velocity _imu_linear_acceleration ))

(defmethod assignment1::SensorFusion
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pointcloud __pointcloud) (instance sensor_msgs::PointCloud2 :init))
    ((:image __image) (instance sensor_msgs::Image :init))
    ((:odom __odom) (instance nav_msgs::Odometry :init))
    ((:imu_orientation __imu_orientation) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:imu_angular_velocity __imu_angular_velocity) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:imu_linear_acceleration __imu_linear_acceleration) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _pointcloud __pointcloud)
   (setq _image __image)
   (setq _odom __odom)
   (setq _imu_orientation __imu_orientation)
   (setq _imu_angular_velocity __imu_angular_velocity)
   (setq _imu_linear_acceleration __imu_linear_acceleration)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pointcloud
   (&rest __pointcloud)
   (if (keywordp (car __pointcloud))
       (send* _pointcloud __pointcloud)
     (progn
       (if __pointcloud (setq _pointcloud (car __pointcloud)))
       _pointcloud)))
  (:image
   (&rest __image)
   (if (keywordp (car __image))
       (send* _image __image)
     (progn
       (if __image (setq _image (car __image)))
       _image)))
  (:odom
   (&rest __odom)
   (if (keywordp (car __odom))
       (send* _odom __odom)
     (progn
       (if __odom (setq _odom (car __odom)))
       _odom)))
  (:imu_orientation
   (&optional __imu_orientation)
   (if __imu_orientation (setq _imu_orientation __imu_orientation)) _imu_orientation)
  (:imu_angular_velocity
   (&optional __imu_angular_velocity)
   (if __imu_angular_velocity (setq _imu_angular_velocity __imu_angular_velocity)) _imu_angular_velocity)
  (:imu_linear_acceleration
   (&optional __imu_linear_acceleration)
   (if __imu_linear_acceleration (setq _imu_linear_acceleration __imu_linear_acceleration)) _imu_linear_acceleration)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; sensor_msgs/PointCloud2 _pointcloud
    (send _pointcloud :serialization-length)
    ;; sensor_msgs/Image _image
    (send _image :serialization-length)
    ;; nav_msgs/Odometry _odom
    (send _odom :serialization-length)
    ;; float64[4] _imu_orientation
    (* 8    4)
    ;; float64[3] _imu_angular_velocity
    (* 8    3)
    ;; float64[3] _imu_linear_acceleration
    (* 8    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; sensor_msgs/PointCloud2 _pointcloud
       (send _pointcloud :serialize s)
     ;; sensor_msgs/Image _image
       (send _image :serialize s)
     ;; nav_msgs/Odometry _odom
       (send _odom :serialize s)
     ;; float64[4] _imu_orientation
     (dotimes (i 4)
       (sys::poke (elt _imu_orientation i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _imu_angular_velocity
     (dotimes (i 3)
       (sys::poke (elt _imu_angular_velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _imu_linear_acceleration
     (dotimes (i 3)
       (sys::poke (elt _imu_linear_acceleration i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; sensor_msgs/PointCloud2 _pointcloud
     (send _pointcloud :deserialize buf ptr-) (incf ptr- (send _pointcloud :serialization-length))
   ;; sensor_msgs/Image _image
     (send _image :deserialize buf ptr-) (incf ptr- (send _image :serialization-length))
   ;; nav_msgs/Odometry _odom
     (send _odom :deserialize buf ptr-) (incf ptr- (send _odom :serialization-length))
   ;; float64[4] _imu_orientation
   (dotimes (i (length _imu_orientation))
     (setf (elt _imu_orientation i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _imu_angular_velocity
   (dotimes (i (length _imu_angular_velocity))
     (setf (elt _imu_angular_velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _imu_linear_acceleration
   (dotimes (i (length _imu_linear_acceleration))
     (setf (elt _imu_linear_acceleration i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get assignment1::SensorFusion :md5sum-) "c4f5e90b537eb296772ed90404b0484f")
(setf (get assignment1::SensorFusion :datatype-) "assignment1/SensorFusion")
(setf (get assignment1::SensorFusion :definition-)
      "std_msgs/Header header

# Sensors
sensor_msgs/PointCloud2 pointcloud
sensor_msgs/Image image
nav_msgs/Odometry odom

# IMU simulated values
float64[4] imu_orientation          # Quaternion (x, y, z, w)
float64[3] imu_angular_velocity      # Angular velocity (x, y, z)
float64[3] imu_linear_acceleration   # Linear acceleration (x, y, z)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: nav_msgs/Odometry
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :assignment1/SensorFusion "c4f5e90b537eb296772ed90404b0484f")


