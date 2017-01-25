# Autoland package 
This is a ROS package created during my bachelor thesis (paper included).
This package is used to provide an extandable sensor method to land with centimeter accuracy in varrying conditions (fog, night etc..).This is achieved by retrieving position information using an RTK GPS and controlling the quadrotor velocity with a PID filter. The software is ran on a companion computer (Raspberry Pi) connected to a pixhawk.

The paper is included in the repository, for any additional questions you can contact me or create an github issue.

### Dependencies
ROS indigo
#### ROS packages
* mavros [doc](http://wiki.ros.org/mavros)
* swiftnav [doc](http://wiki.ros.org/swiftnav)
* PID [doc](http://wiki.ros.org/pid)



