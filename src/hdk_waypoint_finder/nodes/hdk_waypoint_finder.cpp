/*
 * @file hdk_waypoint_finder.cpp
 * @brief ロボットの近くにあるwaypointの名前を返すプログラム．
 *        ロボットの自己位置を受け取り，waypoint名を返す．
 *        ROSサービスで実装．
 * @author Hidekazu TAKAHASHI
 * @date 2017/11/07
 * Updated by ryu, 2023/10/18
 *  .Use Qt v5.14 lib to update config file
 */

#include <cstdio>
#include <cstdlib>

#include <QtGlobal>
#include <QString>
#include <QtMath>

#include "ros/ros.h"
#include "gnd_msgs/msg_pose2d_stamped.h"
#include "gnd-path-io.h"
#include "gnd_msgs/msg_path_area_and_speed_limited.h"
#include "hdk_waypoint_finder.h"

gnd_msgs::msg_pose2d_stamped	             msg_pose;
gnd_msgs::msg_path_area_and_speed_limited  path;
gnd::path::path_net_area_and_speed_limited path_net;

void pose_callback(const gnd_msgs::msg_pose2d_stamped::ConstPtr& msg)
{
    /* update pose */
    msg_pose = *msg;

}

bool srv_find_callback(hdk_waypoint_finder::srv_find_waypointRequest &request,
                       hdk_waypoint_finder::srv_find_waypointResponse &response)
{
    char waypoint_name[128];

    /* check if the robot is in movable area */
    if ( !hdk::waypoint_finder::is_in_path_net( path_net, msg_pose.x, msg_pose.y, 0.0 ) )
    {
        /* out of area */
        return false;
    }

    /* find */
    bool ret_value;
    ret_value = hdk::waypoint_finder::find_waypoint( path_net, msg_pose.x, msg_pose.y, 0.0,
            request.waypoint_name_destination.c_str(), waypoint_name );
    response.waypoint_name = std::string(waypoint_name);

    return ret_value;
}

bool srv_travelable_callback(hdk_waypoint_finder::srv_is_in_travelable_areaRequest &request,
                             hdk_waypoint_finder::srv_is_in_travelable_areaResponse &response)
{
    /* check if the robot is in movable area */
    if ( !hdk::waypoint_finder::is_in_path_net( path_net, msg_pose.x, msg_pose.y, 0.0 ) )
    {
        /* out of area */
        response.ret = false;
    }
    else
    {
        response.ret = true;
    }

    return true;
}

int main(int argc, char *argv[])
{
    hdk::waypoint_finder::node_config node_config;

    // start up, read configuration file
    if (argc > 1)
    {
        if (hdk::waypoint_finder::fread_node_config(argv[1], &node_config) < 0)
        {
            ROS_ERROR("   ... Error: fail to read config file \"%s\"", argv[1]);

            QString fname = QString(argv[1]) + ".tmp";
            // file out configuration file
            if( hdk::waypoint_finder::fwrite_node_config( fname, &node_config ) >= 0 )
            {
                ROS_INFO("  : output sample configuration file %s", fname.toStdString().c_str());
            }

            return -1;
        }
        else
        {
            ROS_INFO("   ... read config file \"%s\"", argv[1]);
        }
    }

    // initialize ros
    ROS_INFO(" => initialize ros");
    if (node_config.node_name.value.at(0).size() > 0)
    {
        ros::init(argc, argv, node_config.node_name.value.at(0).toStdString());
        if( ros::isInitialized() )
        {
            // nothing to do
            ROS_INFO("    ... ok");
        }
        else
        {
            ROS_ERROR("   ... Error: fail ROS initialization");
            return -1;
        }
    }
    else
    {
        ROS_ERROR("   ... Error: node name is null, you must specify the name of this node via config item \"%s\"", node_config.node_name.item.toStdString().c_str());
        return -1;
		}

    // load route data file
    if( ros::ok() && node_config.path_map_file.value.at(0).size() > 0)
    {
        ROS_INFO(" => load route data file");
        ROS_INFO("    file path is \"%s\"", node_config.path_map_file.value.at(0).toStdString().c_str());

        if( gnd::path::fread(node_config.path_map_file.value.at(0).toStdString().c_str(), &path_net) < 0 )
        {
            ROS_ERROR("    ... error : fail to read route file");
            ros::shutdown();
        }
        else
        {
            ROS_INFO("    ... ok");
        }
    }

    // ros communication object
    ros::NodeHandle			   nh;
    /* pose subscriber */
    ros::Subscriber			   pose_sub;
    /* find waypoint server */
    ros::ServiceServer		 srvserv_find_waypoint;
    /* is in travelable area server */
    ros::ServiceServer		 srvserv_is_in_travelable_area;

    // ---> make pose subscriber
    if (ros::ok())
    {
        ROS_INFO("   => make pose subscriber");
        if( node_config.topic_name_pose.value.at(0).isEmpty() )
        {
            ROS_ERROR("    ... error: pose topic name is null");
            ROS_ERROR("      usage: fill \"%s\" item in configuration file", node_config.topic_name_pose.item.toStdString().c_str() );
            ros::shutdown();
        }
        else
        {
            ROS_INFO("    ... topic name is \"%s\"", node_config.topic_name_pose.value.at(0).toStdString().c_str());

            // subscribe
            pose_sub = nh.subscribe(node_config.topic_name_pose.value.at(0).toStdString(), 100, pose_callback);

            ROS_INFO("    ... ok");
        }
    } // <--- make pose subscriber


    // ---> make service find waypoint server
    ROS_INFO(" => make service server to find waypoint");
    if( node_config.service_name_find_waypoint.value.at(0).isEmpty() )
    {
        ROS_ERROR("    ... error: invalid service name");
        ros::shutdown();
    }
    else
    {
        ROS_INFO("    ... service name is \"%s\"", node_config.service_name_find_waypoint.value.at(0).toStdString().c_str() );
        srvserv_find_waypoint = nh.advertiseService(node_config.service_name_find_waypoint.value.at(0).toStdString().c_str(), srv_find_callback);
		}

    // ---> make service is in travelable area server
    ROS_INFO(" => make service server to check specified position is in travelable area");
    if( node_config.service_name_is_in_travelable_area.value.at(0).isEmpty() )
    {
        ROS_ERROR("    ... error: invalid service name");
				ros::shutdown();
    }
    else
    {
        ROS_INFO("    ... service name is \"%s\"", node_config.service_name_is_in_travelable_area.value.at(0).toStdString().c_str() );
        srvserv_is_in_travelable_area = nh.advertiseService(node_config.service_name_is_in_travelable_area.value.at(0).toStdString(), srv_travelable_callback);
		}

    // ---> operation
    if ( ros::ok() )
    {
        ros::Rate loop_rate(100.0);
        //main loop
        ROS_INFO(" => main loop start");
        while ( ros::ok() )
        {
            /* blocking */
            loop_rate.sleep();

            /* spin */
            ros::spinOnce();

        }
    } // <--- operation

    return 0;
}
