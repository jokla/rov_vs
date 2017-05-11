#include <iostream>
#include <vector>
#include <algorithm>

#include <visp/vpFeatureBuilder.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpPlot.h>
#include <visp/vpTime.h>


#include "pid_controller.h"

#include "rov_vs.h"

rov_vs::rov_vs(ros::NodeHandle &nh): m_cam(),  m_camInfoIsInitialized(false), m_width(800), m_height(600),  m_effort_cmd(4)
{
    // read in config options
    n = nh;

    m_servo_time_init = 0;
    m_cMt_isInitialized = false;

    n.param( "frequency", m_freq, 20);
    n.param<std::string>("cameraInfoName", m_cameraInfoName, "/camera/camera_info");
    n.param<std::string>("targetTopicName", m_poseTargetTopicName, "/vision/pose");
    n.param<std::string>("RovTopicName", m_poseRovTopicName, "/pose_estimation");
    n.param<std::string>("cmdTopicName", m_cmdTopicName, "/command");
    n.param<std::string>("cmdVelTopicName", m_cmdVelTopicName, "/cmd_vel");
    n.param<std::string>("statusTargetTopicName", m_statusTargetTopicName, "/vision/status");

    m_cameraInfoSub = n.subscribe( m_cameraInfoName, 1, (boost::function < void(const sensor_msgs::CameraInfoConstPtr & )>) boost::bind( &rov_vs::getCameraInfoCb, this, _1 ));
    m_statusTargetSub = n.subscribe( m_statusTargetTopicName, 1, (boost::function < void(const std_msgs::Int8::ConstPtr & )>) boost::bind( &rov_vs::getStatusTargetCb, this, _1 ));
    m_poseTargetSub = n.subscribe( m_poseTargetTopicName, 1, (boost::function < void(const geometry_msgs::PoseStamped::ConstPtr & )>) boost::bind( &rov_vs::getPoseTargetCb, this, _1 ));
    m_poseRovSub = n.subscribe( m_poseRovTopicName, 1, (boost::function < void(const geometry_msgs::PoseStamped::ConstPtr & )>) boost::bind( &rov_vs::getPoseRovCb, this, _1 ));

    m_cmdPub  = n.advertise<std_msgs::Float32MultiArray>(m_cmdTopicName, 1000);
    m_cmdVelPub  = n.advertise<geometry_msgs::TwistStamped>(m_cmdVelTopicName, 1000);

}

rov_vs::~rov_vs(){

}

void  rov_vs::initializationVS()
{

    // PBVS Visual Servoing
    m_t.setFeatureTranslationType(vpFeatureTranslation::cdMc);
    // m_tu.setFeatureThetaURotationType(vpFeatureThetaU::cdRc);

    // Build the desired visual
    m_s_star_t.setFeatureTranslationType(vpFeatureTranslation::cdMc); // Default initialization to zero
    //m_s_star_tu.setFeatureThetaURotationType(vpFeatureThetaU::cdRc);// Default initialization to zero

    // We want to see a point on a point
    m_base_task.addFeature(m_t, m_s_star_t) ;   // 3D translation
    //m_base_task.addFeature(m_tu, m_s_star_tu) ; // 3D rotation
    m_base_task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    // Interaction matrix is computed with the desired visual features sd
    m_base_task.setInteractionMatrixType(vpServo::CURRENT);
    vpAdaptiveGain lambda_base_poly(1.0, 0.4, 2.5);//(1.2, 1.0, 10); // 2.3, 0.7, 15
    m_base_task.setLambda(lambda_base_poly);

    // Define desired pose
    m_cMdt.eye();
    m_cMdt[0][3] = 0.0; // desired tx
    m_cMdt[1][3] = 0.0; // desired ty
    m_cMdt[2][3] = 2.0; // desired tz

    // Jacobian, we control only vx,vy and vz
    m_eJe.resize(6,3,true);
    m_eJe[0][0] = 1;
    m_eJe[1][1] = 1;
    m_eJe[2][2] = 1;

    std::cout << "JAC: " << std::endl << m_eJe << std::endl;

    // Homogeneous transformation from the base to the camera
    vpHomogeneousMatrix bMc;
    bMc[0][0] = 0.;
    bMc[1][1] = 0.;
    bMc[2][2] = -1.;
    bMc[1][0] = -1.;
    bMc[0][1] = -1.;

    bMc[0][3] = 1.25;
    bMc[1][3] = 0;
    bMc[2][3] = -0.2;

    m_cMb = bMc.inverse();
    std::cout << "Camera to base frame cMb  :" << std::endl << m_cMb << std::endl;
    std::cout << "Visual Servoing initialized" << std::endl;

    // Homogeneous matrix between world and target
    m_wMt[0][3] = -92.24;
    m_wMt[1][3] = -193.2;
    m_wMt[2][3] = -60.54;

}

void rov_vs::spin()
{
    ros::Rate loop_rate(m_freq);

    // Initialize Visual servoing variables
    this->initializationVS();

    PID px(8, 46, 250, 0.0001, 2000);
    PID py(8 , 46, 250, 0.0001, 2000);
    PID pz(7, 60, 270, 0.0001, 2000);



    // Create graph
    // Create a window (700 by 700) at position (100, 200) with one graph
    vpPlot A(1, 700, 700, 100, 200, "Translations");
    // The first graphic contains 1 curve and the second graphic contains 2 curves
    A.initGraph(0,3);
    A.setLegend(0,0,"x");
    A.setLegend(0,1,"y");
    A.setLegend(0,2,"z");

    int count = 0;
    double time_prev = vpTime::measureTimeSecond();

    while(ros::ok()){
        // vpDisplay::display(I);

        double time_now = vpTime::measureTimeSecond();

        // Test without camera
        vpHomogeneousMatrix rMt = m_wMr.inverse()*m_wMt;

        std::cout << "t:" << std::endl << rMt.getTranslationVector() << std::endl;
        double dt = time_now - time_prev;
        std::cout << "dt = " << dt << std::endl;


        m_effort_cmd[0] = -px.calculateOutput(rMt[0][3], 0.0, dt);
        m_effort_cmd[1] = -py.calculateOutput(rMt[1][3], 0.0, dt);
        m_effort_cmd[2] = -pz.calculateOutput(rMt[2][3],-2.0, dt);

        if(count>10)
            A.plot(0,count,rMt.getTranslationVector());
        // std::cout << "m_effort_cmd" << std::endl << m_effort_cmd << std::endl;
        publishCmd();


        ros::spinOnce();
        count++;

        loop_rate.sleep();
        time_prev = time_now;
    }

}

void rov_vs::getStatusTargetCb(const std_msgs::Int8::ConstPtr  &status)
{
    m_status_target = status->data;
}

void rov_vs::getCameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg)
{
    std::cout << "Received Camera INFO"<<std::endl;
    // Convert the paramenter in the visp format
    m_cam = visp_bridge::toVispCameraParameters(*msg);
    m_cam.printParameters();

    m_width = msg->width;
    m_height = msg->height;

    // Stop the subscriber (we don't need it anymore)
    this->m_cameraInfoSub.shutdown();

    m_camInfoIsInitialized = 1;
}

void rov_vs::getPoseTargetCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    m_cMt = visp_bridge::toVispHomogeneousMatrix(msg->pose);

    if (!m_cMt.isAnHomogeneousMatrix())
        exit(0);

    if ( !m_cMt_isInitialized )
    {
        ROS_INFO("DesiredPose received");
        m_cMt_isInitialized = true;
    }
}


void rov_vs::getPoseRovCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    m_wMr = visp_bridge::toVispHomogeneousMatrix(msg->pose);

    if (!m_wMr.isAnHomogeneousMatrix())
        exit(0);

    if ( !m_wMr_isInitialized )
    {
        ROS_INFO("DesiredPose received");
        m_wMr_isInitialized = true;
    }
}

void rov_vs::publishCmd()
{
    std_msgs::Float32MultiArray msg;
    msg.data.resize(4);
    msg.data[0] = m_effort_cmd[0];
    msg.data[1] = m_effort_cmd[1];
    msg.data[2] = m_effort_cmd[2];
    msg.data[3] = 0.0;
    m_cmdPub.publish(msg);

}

void rov_vs::publishCmdVel()
{
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.twist.linear.x = m_base_vel[0];
    msg.twist.linear.y = m_base_vel[1];
    msg.twist.linear.z = m_base_vel[2];
    msg.twist.angular.x = 0.;
    msg.twist.angular.y = 0.;
    msg.twist.angular.z = 0.;

    m_cmdVelPub.publish(msg);
}

void rov_vs::publishCmdVelStop()
{
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.twist.linear.x = 0.;
    msg.twist.linear.y = 0.;
    msg.twist.linear.z = 0.;
    msg.twist.angular.x = 0.;
    msg.twist.angular.y = 0.;
    msg.twist.angular.z = 0.;

    m_cmdVelPub.publish(msg);
}

