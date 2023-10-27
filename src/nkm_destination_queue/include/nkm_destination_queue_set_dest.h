/*
 * @file nkm_destination_queue_set_dest.h
 * @brief wait and manage target waypoint from user reuqest
 * @author Hidekazu TAKAHASHI
 * @date 2017/11/07
 * Updated by ryu, 2023/10/19
 *  .Use Qt v5.14 lib to update config file
 */

#ifndef NKM_DESTINATION_QUEUE_SET_DEST_H
#define NKM_DESTINATION_QUEUE_SET_DEST_H

#include <QtGlobal>
#include <QString>
#include <QtMath>

#include "ros/ros.h"
#include "gnd-path.h"
#include "gnd-path-io.h"
#include "gnd_msgs/msg_path_area_and_speed_limited.h"
#include "icartmini_sbtp/srv_set_navigation_path.h"
#include "hdk_waypoint_finder/srv_reset_pose.h"
#include "hdk_waypoint_finder/srv_get_waypoint.h"
#include "hdk_waypoint_finder/srv_is_in_travelable_area.h"
#include "hdk_waypoint_finder/srv_find_waypoint.h"
#include "nkm_destination_queue_set_dest_config.h"

namespace nkm {

class NKMSetDest
{
public:
    int defAttemptLimits;

private:
    gnd_msgs::msg_path_area_and_speed_limited               path;
    gnd::path::path_net_area_and_speed_limited              path_net;

    ros::ServiceClient                                      srv_client_set_navigation_path;
    icartmini_sbtp::srv_set_navigation_path                 srv_set_navigation_path;
    ros::ServiceClient                                      srv_client_reset_pose;
    hdk_waypoint_finder::srv_reset_pose                     srv_reset_pose;
    ros::ServiceClient                                      srv_client_get_waypoint;
    hdk_waypoint_finder::srv_get_waypoint                   srv_get_waypoint;
    ros::ServiceClient                                      srv_client_is_in_travelable_area;
    hdk_waypoint_finder::srv_is_in_travelable_area          srv_is_in_travelable_area;
    ros::ServiceClient                                      srv_client_find_waypoint;
    hdk_waypoint_finder::srv_find_waypoint                  srv_find_waypoint;
    
public:
    int Initialize(node_config *nd_config);
    int SetDestination(const char* nameDestination);
    int GetWaypointPosition(const char *name, double *x, double *y);
    bool IsCorrectWaypoint(const char *name);
    int DeleteDestination();
    bool path_plan( gnd::path::path_net_area_and_speed_limited *path_net,
                    int start,
                    int destination,
                    gnd_msgs::msg_path_area_and_speed_limited *path );

};

}

namespace nkm {

int NKMSetDest::Initialize(node_config *nd_config)
{
    ros::NodeHandle nh;

    defAttemptLimits                 = nd_config->attemp_limits.value.at(0);
    srv_client_set_navigation_path   = nh.serviceClient<icartmini_sbtp::srv_set_navigation_path>(nd_config->client_name_set_navi_path.value.at(0).toStdString());
    srv_client_reset_pose            = nh.serviceClient<hdk_waypoint_finder::srv_reset_pose>(nd_config->client_name_reset_pose.value.at(0).toStdString());
    srv_client_get_waypoint          = nh.serviceClient<hdk_waypoint_finder::srv_get_waypoint>(nd_config->client_name_get_waypoint.value.at(0).toStdString());
    srv_client_is_in_travelable_area = nh.serviceClient<hdk_waypoint_finder::srv_is_in_travelable_area>(nd_config->client_name_is_intravelable_area.value.at(0).toStdString());
    srv_client_find_waypoint         = nh.serviceClient<hdk_waypoint_finder::srv_find_waypoint>(nd_config->client_name_find_waypoint.value.at(0).toStdString());

    /* read path file */
    if ( gnd::path::fread( nd_config->path_file.value.at(0).toStdString().c_str(), &path_net ) < 0 )
    {
        ROS_ERROR("Cannot read specified path file \"%s\"", nd_config->path_file.value.at(0).toStdString().c_str());
        return -1;
    }

    return 0;
}

int NKMSetDest::SetDestination(const char* nameDestination)
{
    char start_str[128];
    int start = -1;
    int destination = -1;

    for(int i = 0 ; i < defAttemptLimits; i++)
    {
        srv_find_waypoint.request.waypoint_name_destination = std::string(nameDestination);

        if( srv_client_find_waypoint.call( srv_find_waypoint ) )
        {
            //get waypoint
            strcpy(start_str, srv_find_waypoint.response.waypoint_name.c_str());

            // find index
            start = path_net.index_waypoint( start_str );
            if ( start < 0 )
            {
                ROS_ERROR("There is not specified waypoint (Start) \"%s\"", start_str);
                continue;
            }
            destination = path_net.index_waypoint(nameDestination);
            if ( destination < 0 )
            {
                ROS_ERROR("There is not specified waypoint (Dest)\"%s\"", nameDestination);
                continue;
            }

            if ( start == destination )
            {
                ROS_ERROR("The destination is the same with the start");
                return 0;
            }

            // set path
            path_plan( &path_net, start, destination, &path );
            srv_set_navigation_path.request.path = path;
            srv_client_set_navigation_path.call(srv_set_navigation_path);

            return 0;
        }
        else
        {
            ROS_ERROR("[Error] Could not find a waypoint nearby");
            continue;
        }
    }
    return -1;
}

int NKMSetDest::GetWaypointPosition(const char *name, double *x, double *y)
{
    return path_net.get_waypoint(name, x, y);
}

bool NKMSetDest::IsCorrectWaypoint(const char *name)
{
    double x, y;
    return (path_net.get_waypoint(name, &x, &y) >= 0) ? true : false;
}

int NKMSetDest::DeleteDestination()
{
    // Call Blank Path.
    if(path_net.n_waypoints() != 0)
    {
        srv_set_navigation_path.request.path.start.name = path_net[0].waypoint.name;
        srv_set_navigation_path.request.path.start.x = path_net[0].waypoint.x;
        srv_set_navigation_path.request.path.start.y = path_net[0].waypoint.y;
    }
    srv_set_navigation_path.request.path.path.clear();
    srv_client_set_navigation_path.call(srv_set_navigation_path);

    return 0;
}

/*
 * @brief Returns path from start to destination.
 * @param[in] path_net the path net, path is extracted ftom this
 * @param[in] start the index of start waypoint
 * @param[in] destination the index of destination waypoint
 * @param[out] path the path from start to destiation
 * @return true if succeed. false, otherwise.
 */
bool NKMSetDest::path_plan( gnd::path::path_net_area_and_speed_limited *path_net, int start, int destination, gnd_msgs::msg_path_area_and_speed_limited *path )
{
    gnd::path::path_net_area_and_speed_limited::path_t ws;
    int ret;

    /* input check */
    if ( (start < 0) || (start >= path_net->n_waypoints()) )
    {
        return false;
    }
    if ( (destination < 0) || (destination >= path_net->n_waypoints()) )
    {
        return false;
    }

    /* clear */
    path->path.clear();

    // start
    path->start.name = (*path_net)[start].waypoint.name;
    path->start.x = (*path_net)[start].waypoint.x;
    path->start.y = (*path_net)[start].waypoint.y;
    path->start.theta = 0;

    // find path
    ret = (*path_net).find_path_dijkstra( &ws, (*path_net)[start].waypoint.name, (*path_net)[destination].waypoint.name );
    if ( ret < 0 )
    {
        path->start.name = "";
        path->path.clear();
        return false;
    }
    for ( int i=0; i<(signed)ws.path.size(); i++ )
    {
        gnd_msgs::msg_path_unit_area_and_speed_limited unit;

        unit.end.name = ws.path[i].end.name;
        unit.end.x = ws.path[i].end.x;
        unit.end.y = ws.path[i].end.y;
        unit.end.theta = ws.path[i].end.theta;

        unit.curvature = ws.path[i].curvature;
        unit.limit_translate = ws.path[i].prop.limit_translate;
        unit.limit_rotate = ws.path[i].prop.limit_rotate;
        unit.start_extend = ws.path[i].prop.start_extend;
        unit.end_extend = ws.path[i].prop.end_extend;
        unit.left_width = ws.path[i].prop.left_width;
        unit.right_width = ws.path[i].prop.right_width;
        path->path.push_back(unit);
    }

    return true;
}

}

#endif //NKM_DESTINATION_QUEUE_SET_DEST_H
