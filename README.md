# Vision-Based Control for a ROV

# Installation:

You will need:
* [ViSP](https://github.com/lagadic/visp)  
* [vision_visp](https://github.com/lagadic/vision_visp)

# Subscribers:
* Intrinsic camera parameters `sensor_msgs::CameraInfo`
* Pose of the target: `geometry_msgs::Polygon`
* Status of the detection: `std_msgs::Int8`

# Publisher:
* Command velocities in `std_msgs::Float32MultiArray.h`

A Posed-Based Visual Servoing is implemented to follow an object. 


The degrees of freedom that are controlled are Vx, Vy and Wz.

For the Visual Servoing I used ViSP:
https://visp.inria.fr/



To launch:
`$ roslaunch rov_vs rov_vs.launch `
