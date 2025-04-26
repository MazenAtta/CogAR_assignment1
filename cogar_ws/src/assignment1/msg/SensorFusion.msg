std_msgs/Header header

# Sensors
sensor_msgs/PointCloud2 pointcloud
sensor_msgs/Image image
nav_msgs/Odometry odom

# IMU simulated values
float64[4] imu_orientation          # Quaternion (x, y, z, w)
float64[3] imu_angular_velocity      # Angular velocity (x, y, z)
float64[3] imu_linear_acceleration   # Linear acceleration (x, y, z)
