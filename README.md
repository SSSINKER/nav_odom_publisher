# nav_odom_publisher

Overview
--
Publish odometry information using GPS localization. </br>

Package Dependency
--

* Necessary
  * geonav_transform: https://github.com/bsb808/geonav_transform.git

* Recommended
  * ebimu_odometry: https://github.com/SSSINKER/ebimu_odometry.git
  * nmea_navsat_driver: https://github.com/ros-drivers/nmea_navsat_driver.git
  
Launch File
--
* ```nav_odom_pub_tf.launch``` : Run nav_odom_publisher.py and geonav_transform_node. Use ```odom_GPS``` and ```base_link_GPS``` frame. Provide ```base_link_GPS``` to ```base_link``` tf.
* ```nav_odom_pub_tf_2.launch``` : Run nav_odom_publisher.py and geonav_transform_node. Use odom and base_link frame

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
  * ```separated_frame_id``` (```bool```, default=True) : if True, use ```odom_GPS``` for ```odom_frame_id``` and ```base_link_GPS``` for ```base_frame_id```.
* Provided tf Transforms
  * ```base_link_GPS``` -> ```base_link``` : zero vector transformation between two base_link frames. Provided only if ```separated_frame_id``` param is set to True.
