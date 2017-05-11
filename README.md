# Vision-Based Control for a ROV

# Installation:

You will need:
* [ViSP](https://github.com/lagadic/visp)  You can clone it and build it from source, see [here](http://visp-doc.inria.fr/doxygen/visp-daily/tutorial-install-ubuntu.html)
* [vision_visp](https://github.com/lagadic/vision_visp) clone it in your catkin_workspace and build it indicating the path to the build folder of ViSP with   
`catkin_make -DVISP_DIR=/home/jokla/Software/visp3/visp/build/`

# Subscribers:
* Intrinsic camera parameters `sensor_msgs::CameraInfo`
* Pose of the target by vision: `geometry_msgs::PoseStamped` (not used here)
* Pose of the rov: `geometry_msgs::PoseStamped`
* Status of the detection: `std_msgs::Int8`

# Publisher:
* Command velocities in `std_msgs::Float32MultiArray.h`


The PID controllers generate 3 forces (fx,fy and fz) so that the target stays in the center of the camera, at a 2 meters distance in the Z axis.



To launch:
`$ roslaunch rov_vs rov_vs.launch `
