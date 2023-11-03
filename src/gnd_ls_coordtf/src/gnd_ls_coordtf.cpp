/*
 * gnd_ls_coordtf_config.hpp
 *
 *  Created on: 2014/09/05
 *      Author: tyamada
 *  Updated by ryu, 2023/5/24
 *  .Use Qt v5.14 lib to update config file
 */
#include <QString>
#include <QGenericMatrix>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "gnd_ls_coordtf_config.h"
#include "gnd_rosutil.h"
#include "gnd-coord-tree.h"
#include "gnd-matrix-coordinate.h"

ros::Publisher				      pointcloud_pub;
sensor_msgs::PointCloud	    pointcloud_msg;
QGenericMatrix<4,4,double>  mat_coordtf;
QFile        log_file;
QTextStream  log_out;

void laserscan_Callback(const sensor_msgs::LaserScan::ConstPtr& laserscan_msg)
{
    uint i = 0;
    uint j = 0;
    uint k = 0;
    sensor_msgs::PointCloud::_points_type::value_type ws_point;  // work space

    pointcloud_msg.points.clear();
    pointcloud_msg.channels[0].values.clear();
    QGenericMatrix<4,1,double> point_src, point_dest; //<N,M> - N:column, M:row

    // ---> coordinate transform and set value
    for( i = 0; i < (signed)laserscan_msg->ranges.size(); i++ )
    {
        // error
        if( laserscan_msg->ranges[i] <= laserscan_msg->range_min ||
            laserscan_msg->ranges[i] >= laserscan_msg->range_max)
        {
          continue;
        }

        // source coordinate point
        point_src(0,0) = laserscan_msg->ranges[i] * cos( laserscan_msg->angle_min + laserscan_msg->angle_increment * i );
        point_src(0,1) = laserscan_msg->ranges[i] * sin( laserscan_msg->angle_min + laserscan_msg->angle_increment * i );
        point_src(0,2) = 0;
        point_src(0,3) = 1;

        // coordinate transform
        point_dest = point_src * mat_coordtf ;

        // set destination coordinate point
        ws_point.x = point_dest(0,0);
        ws_point.y = point_dest(0,1);
        ws_point.z = point_dest(0,2);
        pointcloud_msg.points.push_back(ws_point);

        // set intensity
        if( laserscan_msg->intensities.size() > i )
        {
            pointcloud_msg.channels[0].values.push_back(laserscan_msg->intensities[i]);
        }

        if( log_file.isOpen() )
        {
            log_out << QString::asprintf("%d %lf %lf %lf %lf %lf \r\n",
                                         pointcloud_msg.header.seq + 1,
                                         pointcloud_msg.points[pointcloud_msg.points.size() - 1].x,
                                         pointcloud_msg.points[pointcloud_msg.points.size() - 1].y,
                                         point_src(0,0),
                                         point_src(0,1),
                                         laserscan_msg->ranges[i]);
        }
    }

    // set header
    pointcloud_msg.header.seq++;
    pointcloud_msg.header.stamp = laserscan_msg->header.stamp;

    // publish
    pointcloud_pub.publish(pointcloud_msg);

}

int main(int argc, char **argv)
{
    gnd::ls_coordtf::node_config  node_config;

    // start up, read configuration file
    if( argc > 1 )
    {
        ROS_INFO("=> read configuration file");
        if( gnd::ls_coordtf::fread_node_config( QString(argv[1]), &node_config ) < 0 )
        {
            ROS_INFO("... Error: fail to read config file %s", argv[1]);
            QString fname = QString(argv[1]) + ".tmp";
            // file out config file
            if( gnd::ls_coordtf::fwrite_node_config( fname, &node_config ) >= 0 )
            {
                ROS_INFO("      : output sample config file %s", fname.toStdString().c_str());
            }
            return -1;
        }
        else
        {
            ROS_INFO("   ... ok, read config file %s", argv[1]);
        }
    }

    // initialize ros
    if( node_config.node_name.value.at(0).size() > 0 )
    {
        ros::init(argc, argv, node_config.node_name.value.at(0).toStdString());
    }
    else
    {
        std::cout << "   ... Error: node name is null, you must specify the name of this node via config item"
                  << node_config.node_name.item.toStdString() << "\n";
        return -1;
    }
    ROS_INFO(" node: %s ",node_config.node_name.value.at(0).toStdString().c_str());


    ros::NodeHandle	nh;
    ros::Subscriber	laserscan_sub;
    int             phase = 0;

    // ---> initialize
    ROS_INFO("---------- initialize ----------");

    // show initialize phase task
    if( ros::ok() )
    {
        ROS_INFO(" initialization task");
        ROS_INFO("   %d. calculate coordinate transform matrix", ++phase);
        ROS_INFO("   %d. make laser-scan topic subscriber", ++phase);
        ROS_INFO("   %d. make point-cloud topic publisher", ++phase);
        if( node_config.text_log.value.at(0).size() > 0 )
        {
            ROS_INFO("   %d. create text log file", ++phase);
        }
    }

    // calculate coordinate transform matrix
    if( ros::ok() )
    {
        ROS_INFO(" => calculate coordinate transform matrix");
        ROS_INFO("    ... defined coordinate" );
        ROS_INFO("        origin: %.03lf, %.03lf, %.03lf",
            node_config.coordinate_origin.value[0], node_config.coordinate_origin.value[1], node_config.coordinate_origin.value[2]);
        ROS_INFO("         front: %.03lf, %.03lf, %.03lf",
            node_config.axis_vector_front.value[0], node_config.axis_vector_front.value[1], node_config.axis_vector_front.value[2]);
        ROS_INFO("        upside: %.03lf, %.03lf, %.03lf",
            node_config.axis_vector_upside.value[0], node_config.axis_vector_upside.value[1], node_config.axis_vector_upside.value[2]);

        gnd::matrix::coordinate_converter(&mat_coordtf,
            node_config.coordinate_origin.value[0], node_config.coordinate_origin.value[1], node_config.coordinate_origin.value[2],
            node_config.axis_vector_front.value[0], node_config.axis_vector_front.value[1], node_config.axis_vector_front.value[2],
            node_config.axis_vector_upside.value[0], node_config.axis_vector_upside.value[1], node_config.axis_vector_upside.value[2]);
    }

    // make laserscan subscriber
    if( ros::ok() )
    {
        ROS_INFO(" => make laser-scan topic subscriber");

        if( node_config.topic_name_laserscan.value.at(0).size() == 0 )
        {
            ROS_ERROR("    ... error: laser scan topic name is null");
            ROS_ERROR("        usage: fill \"%s\" item in configuration file\n", node_config.topic_name_laserscan.item.toStdString().c_str());
            ros::shutdown();
        }
        else
        {
            ROS_INFO("    ... topic name is \"%s\"\n", node_config.topic_name_laserscan.value.at(0).toStdString().c_str());
            // subscribe
            laserscan_sub = nh.subscribe(node_config.topic_name_laserscan.value.at(0).toStdString(), 40, laserscan_Callback);
            ROS_INFO("    ... ok");
        }
    }


    // ---> make pointcloud publisher
    if( ros::ok() )
    {
        ROS_INFO(" => make point-cloud topic publisher");

        if( node_config.topic_name_pointcloud.value[0].size() == 0 )
        {
            ROS_ERROR("    ... error: point-cloud on coordinate topic name is null");
            ROS_ERROR("        usage: fill \"%s\" item in configuration file", node_config.topic_name_pointcloud.item.toStdString().c_str());
            ros::shutdown();
        }
        else
        {
            pointcloud_pub = nh.advertise<sensor_msgs::PointCloud>(node_config.topic_name_pointcloud.value.at(0).toStdString(), 40);

            pointcloud_msg.header.seq = 0;
            pointcloud_msg.header.stamp = ros::Time::now();
            pointcloud_msg.header.frame_id = node_config.topic_name_pointcloud.value.at(0).toStdString();

            pointcloud_msg.points.clear();
            pointcloud_msg.channels.resize(1);
            pointcloud_msg.channels[0].name = "intensity";
            pointcloud_msg.channels[0].values.clear();

            ROS_INFO("    ... ok");
        }
    }

    // init text log
    if( ros::ok() && node_config.text_log.value.at(0).size() > 0 )
    {
        ROS_INFO("   %d. create text log file\n", ++phase);

        log_file.setFileName(node_config.text_log.value.at(0));
        if(log_file.open(QIODevice::WriteOnly  | QIODevice::Text))
        {
            log_out.setDevice(&log_file);
            log_out << "#[1. sequence id] [2. x] [3. y]\n";
            ROS_INFO("    ... ok, create file \"%s\"\n", node_config.text_log.value.at(0).toStdString().c_str() );
        }
        else
        {
            ROS_ERROR("    ... error: fail to open \"%s\"", node_config.text_log.value.at(0).toStdString().c_str());
            ros::shutdown();
        }

    }

    ros::Rate loop_rate(1000);

    while( ros::ok() )
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    // finalize
    ROS_INFO("---------- ls_coordtf finalize ----------");
    if( log_file.isOpen() )
    {
        log_file.close();
    }

    return 0;
}

