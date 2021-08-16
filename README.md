# nav_odom_publisher

Overview
--
Publish odometry information using GPS localization. </br>

Package Dependency
--

* Necessary
  * geonav_transform: https://github.com/bsb808/geonav_transform.git

* Recommended
  * ebimu_odometry: https://github.com/doyle34/ebimu_odometry.git
  * nmea_navsat_driver: https://github.com/ros-drivers/nmea_navsat_driver.git
  
Launch File
--
* ```nav_odom_pub_tf.launch``` : Run nav_odom_publisher.py and geonav_transform_node

Nodes
--
### nav_odom_publisher.py

* Subscribed Topics
  * ```/fix``` (sensor_msgs/NavSatFix) : current position of robot from GPS
  * ```/vel``` (geometry_msgs/TwistStamped) : current linear velocity of robot from GPS
  * ```/imu_data``` (sensors_msg/Imu) : IMU data
* Published Topics
  * ```/init_fix``` (sensor_msgs/NavSatFix) : initial GPS coordinate. For compatibility with geonav_transform package.
  * ```nav_odom``` (nav_msgs/Odometry) : Odometry data from GPS and IMU. Position and linear velocity are provided by GPS,
  while orientation and angular velocity are provided by IMU.
* Parameters
  * nan
* Provided tf Transforms
  * ```base_link_GPS``` -> ```base_link``` : zero vector transformation between two base_link frames.
