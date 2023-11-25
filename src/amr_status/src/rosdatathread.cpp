#include "rosdatathread.h"

RosDataThread::RosDataThread(QObject *parent)
    : QThread{parent}
{
    pose_sub = nh.subscribe<gnd_msgs::msg_pose2d_stamped>("pose_particle_localizer",100,&RosDataThread::poseCallback,this);
    pointCloud_sub = nh.subscribe<sensor_msgs::PointCloud>("coordinate/scan",100,&RosDataThread::pointCloudCallback,this);
    status_sub = nh.subscribe<gnd_msgs::msg_vehicle_status>("icartmini_sbtp/vehicle_status",10,&RosDataThread::statusCallback,this);
    particle_sub = nh.subscribe<gnd_particle_localizer::msg_particles_pose2d_stamped>("particles_particle_localizer",10,&RosDataThread::particleCallback,this);
    coordid_gl = -1;
    coordid_map = -1;
    coordid_rbt = -1 ;
    pose_buff.resize(MAX_SIZE);

    // define coordinate
    ROS_INFO("   => define coordinate");
    QGenericMatrix<4,4,double> coordinate_matrix;
    // initialize coordinate matrix
    coordinate_matrix.setToIdentity();
    // set global coordinate
    coordid_gl = coordinate_tree.add_node("global", "root", &coordinate_matrix);
    // set robot coordinate
    coordid_rbt = coordinate_tree.add_node("robot","global", &coordinate_matrix);
    // set map coordinate
    gnd::matrix::coordinate_converter(&coordinate_matrix,
        0.0, 0.0, 0.0, //map-origin[0~2]
        1.0, 0.0, 0.0, //map-x-axis[0~1]
        0.0, 0.0, 1.0  //fixed:0.0, 0.0, 1.0
    );
    coordid_map = coordinate_tree.add_node("map", "global", &coordinate_matrix);
}

RosDataThread::~RosDataThread()
{

}

void RosDataThread::run()
{

    ros::spin();

}

void RosDataThread::poseCallback(const gnd_msgs::msg_pose2d_stamped::ConstPtr& msg)
{    
    pose_buff.push(*msg);
}

void RosDataThread::pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    if(pose_buff.copy_at_time(&poseMsg, msg->header.stamp.toSec()) < 0)
    {
        return; //get pose data error
    }

    QGenericMatrix<4,4,double> mat_coordtf_rbt2gl;
    QGenericMatrix<4,4,double> mat_coordtf_rbt2map;
    sensor_msgs::PointCloud	    msg_tmp;
    sensor_msgs::PointCloud::_points_type::value_type msg_value;
    msg_tmp.channels.resize(1);
    msg_tmp.channels[0].values.resize(msg->points.size());

    // calculate coordinate transform matrix
    gnd::matrix::coordinate_converter(&mat_coordtf_rbt2gl,
        poseMsg.x,          poseMsg.y,          0,
        cos(poseMsg.theta), sin(poseMsg.theta), 0,
        0,                  0,                  1.0);

    coordinate_tree.set_coordinate(coordid_rbt, &mat_coordtf_rbt2gl);
    coordinate_tree.get_convert_matrix(coordid_rbt, coordid_map, &mat_coordtf_rbt2map);

    // scanning loop (point cloud data)
    for(uint i = 0; i < msg->points.size(); i++)
    {
        QGenericMatrix<4,1,double> point_src, point_map;
        // coordinate transform
        point_src(0,0) = msg->points[i].x;
        point_src(0,1) = msg->points[i].y;
        point_src(0,2) = msg->points[i].z;
        point_src(0,3) = 1;

        point_map = point_src * mat_coordtf_rbt2map;

        //update pointCloud data
        msg_value.x = point_map(0,0);
        msg_value.y = point_map(0,1);
        msg_value.z = point_map(0,2);
        msg_tmp.points.push_back(msg_value);
        msg_tmp.channels[0].values[i] = msg->channels.at(0).values[i];

    }
    msg_tmp.header.seq = msg->header.seq;
    msg_tmp.header.stamp = msg->header.stamp;

    emit robotDataReceived(msg_tmp, poseMsg);
}

void RosDataThread::statusCallback(const gnd_msgs::msg_vehicle_status::ConstPtr& msg)
{
    emit statusReceived(msg->status);
}

void RosDataThread::particleCallback(const gnd_particle_localizer::msg_particles_pose2d_stamped::ConstPtr& msg)
{
    emit particlesReceived(*msg);
}
