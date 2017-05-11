#include <visp3/gui/vpDisplayX.h>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Polygon.h>
#include <std_msgs/Int8.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpQuaternionVector.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>
#include <visp/vpFeatureTranslation.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpServo.h>

#include <visp_bridge/3dpose.h>
#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>


class rov_vs
{
public:

    rov_vs(ros::NodeHandle &nh);
    ~rov_vs();
    void initializationVS();
    bool computeBaseTLDControlLaw();
    void spin();
    void getCameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg);
    void getPoseTargetCb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void getPoseRovCb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void getStatusTargetCb(const std_msgs::Int8::ConstPtr  &status);
    void publishCmdVel();
    void publishCmd();
    void publishCmdVelStop();
    void waitforPose();


protected:

    //Display
    vpImage<unsigned char> I;
    vpDisplayX d;
    int m_width;
    int m_height;

    vpCameraParameters m_cam;

    // ROS
    ros::NodeHandle n;
    std::string cmdVelTopicName;
    std::string m_cameraInfoName;
    std::string m_statusTargetTopicName;
    std::string m_poseTargetTopicName;
    std::string m_poseRovTopicName;
    std::string  m_cmdVelTopicName;
    std::string  m_cmdTopicName;
    ros::Subscriber m_cameraInfoSub;
    ros::Subscriber m_poseTargetSub;
    ros::Subscriber m_statusTargetSub;
    ros::Subscriber m_poseRovSub;
    ros::Publisher m_cmdPub;
    ros::Publisher m_cmdVelPub;
    int m_freq;

    // Servo Base PBVS
    bool m_pbvs_base;
    vpServo m_base_task;
    vpFeatureTranslation m_t;
    vpFeatureThetaU m_tu;
    vpFeatureTranslation m_s_star_t;
    vpFeatureThetaU m_s_star_tu;
    vpHomogeneousMatrix m_cMdt;
    vpHomogeneousMatrix m_cMt;
    vpHomogeneousMatrix m_wMr;
    vpHomogeneousMatrix m_wMt;
    double m_servo_time_init;


    // Servo Base to track an object
    int m_status_target;

    vpMatrix m_eJe;
    vpHomogeneousMatrix m_cMb;
    vpColVector m_base_vel;
    vpColVector m_effort_cmd;


    //conditions
    bool m_servo_enabled;
    bool m_camInfoIsInitialized;
    bool m_cMt_isInitialized;
    bool m_wMr_isInitialized;

};
