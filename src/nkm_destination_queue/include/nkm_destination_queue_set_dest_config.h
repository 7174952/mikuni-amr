/*
 * @file nkm_destination_queue_set_dest.h
 * @brief wait and manage target waypoint from user reuqest
 * @author Hidekazu TAKAHASHI
 * @date 2017/11/07
 * Updated by ryu, 2023/10/19
 *  .Use Qt v5.14 lib to update config file
 */

#ifndef NKM_DESTINATION_QUEUE_SET_DEST_CONFIG_H
#define NKM_DESTINATION_QUEUE_SET_DEST_CONFIG_H

#include <QString>
#include <QtMath>
#include "gnd-configfile.h"
#include "gnd_rosutil.h"

// ---> const variables definition
namespace nkm {
    // ---> ros commnication
    static const gnd::Param<QString> Default_node_name = {
        "node-name",
        {"nkm_destination_queue"},
        "ros-node name"
    };

    static const gnd::Param<QString> Default_path_file = {
        "path-file",
        {"maps.path"},
        "path map file name"
    };

    static const gnd::Param<QString> Default_srv_name_add_destination = {
        "srv-name-add-destination",
        {"nkm_destination_queue/add_destination"},
        "service name to add destinationt (server)"
    };

    static const gnd::Param<QString> Default_srv_name_delete_destination = {
        "srv-name-delete-destination",
        {"nkm_destination_queue/delete_destination"},
        "service name to destination (server)"
    };

    static const gnd::Param<QString> Default_srv_name_show_queue = {
        "srv-name-show-queue",
        {"nkm_destination_queue/show_queue"},
        "service name to show queue (server)"
    };

    static const gnd::Param<QString> Default_client_name_set_navi_path = {
        "client-name-set-navi-path",
        {"set_navigation_path"},
        "service name to set navigation path (client)"
    };

    static const gnd::Param<QString> Default_client_name_reset_pose = {
        "client-name-reset-pose",
        {"reset_pose"},
        "service name to reset pose (client)"
    };

    static const gnd::Param<QString> Default_client_name_get_waypoint = {
        "client-name-get-waypoint",
        {"get_waypoint"},
        "service name to get waypoint (client)"
    };

    static const gnd::Param<QString> Default_client_name_is_intravelable_area = {
        "client-name-is-intravelable-area",
        {"is_in_travelable_area"},
        "service name to intravelable area (client)"
    };

    static const gnd::Param<QString> Default_client_name_find_waypoint = {
        "client-name-find-waypoint",
        {"find_waypoint"},
        "service name to find waypoint (client)"
    };

    static const gnd::Param<QString> Default_topic_name_pose = {
        "topic-name-pose",
        {"pose_particle_localizer"},
        "topic name to pose"
    };

    static const gnd::Param<QString> Default_topic_name_status = {
        "topic-name-status",
        {"vehicle_status"},
        "topic name to status"
    };

    static const gnd::Param<QString> Default_topic_name_queue_info = {
        "topic-name-queue-info",
        {"nkm_destination_queue/info"},
        "topic name to queue info"
    };

    static const gnd::Param<QString> Default_topic_name_pub_destination = {
        "topic-name-pub-destination",
        {"nkm_destination_queue/next_dest"},
        "topic name to publish destination"
    };
    // <--- ros commnication

    static const gnd::Param<double> Default_arrive_decision_distance = {
        "arrive-decision-distance",
        {1.0},
        "arrive decision distance"
    };

    static const gnd::Param<double> Default_attemp_limits = {
        "attempt-limits",
        {10.0},
        "attempt limits"
    };

    // ---> function declaration
    /**
     * @brief configuration parameter for node
     */
    struct node_config
    {
        node_config();
        // ros communication option
        gnd::Param<QString> node_name;							              //< ros-node name
        gnd::Param<QString> path_file;						                //< topic name(subscribe)
        gnd::Param<QString> srv_name_add_destination;				      //< service name(server)
        gnd::Param<QString> srv_name_delete_destination;	        //< service name(server)
        gnd::Param<QString> srv_name_show_queue;						      //< path map file
        gnd::Param<QString> client_name_set_navi_path;						//< path map file
        gnd::Param<QString> client_name_reset_pose;						    //< path map file
        gnd::Param<QString> client_name_get_waypoint;						  //< path map file
        gnd::Param<QString> client_name_is_intravelable_area;		  //< path map file
        gnd::Param<QString> client_name_find_waypoint;						//< path map file
        gnd::Param<QString> topic_name_pose;						          //< path map file
        gnd::Param<QString> topic_name_status;						        //< path map file
        gnd::Param<QString> topic_name_queue_info;						    //< path map file
        gnd::Param<QString> topic_name_pub_destination;						//< path map file
        gnd::Param<double> arrive_decision_distance;						  //< path map file
        gnd::Param<double> attemp_limits;						              //< path map file

    };

    /**
     * @brief initialize configure to default parameter
     * @param [out] p : node_config
     */
    int init_node_config(node_config *p);

    /**
     * @brief config file read
     * @param [in] fname : file name
     * @param [out] dest : configuration parameter
     */
    int fread_node_config( const QString& fname, node_config *dest );
    /**
     * @brief get config parameter from description
     * @param [out] dest : configuration parameter
     * @param [in]  src  : configuration description
     */
    int get_node_config( node_config *dest, gnd::ConfigFile *src );

    /**
     * @brief config file write
     * @param [in] fname : file name
     * @param [in] src   : configuration parameter
     */
    int fwrite_node_config( const QString& fname, node_config *src );
    /**
     * @brief set config description
     * @param [out] dest : description
     * @param [in]   src : parameter
     */
    int set_node_config( gnd::ConfigFile *dest, node_config *src );
    // <--- function declaration

}
// <--- const variables definition

// ---> function definition
namespace nkm {
    node_config::node_config()
    {
        init_node_config(this);
    }

    /*
     * @brief initialize configuration parameter
     * @param [out] p : node_config
     */
    int init_node_config( node_config *p )
    {
        Q_ASSERT_X(p, "invalid", "invalid null pointer argument\n" );

        // ros communication parameter
        p->node_name                        = Default_node_name;
        p->path_file                        = Default_path_file;
        p->srv_name_add_destination         = Default_srv_name_add_destination;
        p->srv_name_delete_destination      = Default_srv_name_delete_destination;
        p->srv_name_show_queue              = Default_srv_name_show_queue;

        p->client_name_set_navi_path        = Default_client_name_set_navi_path;
        p->client_name_reset_pose           = Default_client_name_reset_pose;
        p->client_name_get_waypoint         = Default_client_name_get_waypoint;
        p->client_name_is_intravelable_area = Default_client_name_is_intravelable_area;
        p->client_name_find_waypoint        = Default_client_name_find_waypoint;

        p->topic_name_pose                  = Default_topic_name_pose;
        p->topic_name_status                = Default_topic_name_status;
        p->topic_name_queue_info            = Default_topic_name_queue_info;
        p->topic_name_pub_destination       = Default_topic_name_pub_destination;

        p->arrive_decision_distance         = Default_arrive_decision_distance;
        p->attemp_limits                    = Default_attemp_limits;

        return 0;
    }

    /*
     * @brief config file read
     * @param [in] fname : file name
     * @param [out] dest : configuration parameter
     */
    int fread_node_config( const QString& fname, node_config *dest )
    {
        Q_ASSERT_X(fname.size() > 0, "invalid", "invalid null pointer argument\n" );
        Q_ASSERT_X(dest, "invalid", "invalid null pointer argument\n" );

        int ret;
        gnd::ConfigFile fs;
        // configuration file read
        if( (ret = fs.read_from_file(fname)) < 0 )    return ret;

        return get_node_config(dest, &fs);
    }

    /*
     * @brief get config parameter from description
     * @param [out] dest : configuration parameter
     * @param [in]  src  : configuration description
     */
    int get_node_config( node_config *dest, gnd::ConfigFile *src )
    {
        Q_ASSERT_X(dest, "invalid", "invalid null pointer argument\n" );
        Q_ASSERT_X(src, "invalid", "invalid null pointer argument\n" );

        // ros communication parameter
        src->get_param( &dest->node_name );
        src->get_param( &dest->path_file );
        src->get_param( &dest->srv_name_add_destination );
        src->get_param( &dest->srv_name_delete_destination );
        src->get_param( &dest->srv_name_show_queue );

        src->get_param( &dest->client_name_set_navi_path );
        src->get_param( &dest->client_name_reset_pose );
        src->get_param( &dest->client_name_get_waypoint );
        src->get_param( &dest->client_name_is_intravelable_area );
        src->get_param( &dest->client_name_find_waypoint );

        src->get_param( &dest->topic_name_pose );
        src->get_param( &dest->topic_name_status );
        src->get_param( &dest->topic_name_queue_info );
        src->get_param( &dest->topic_name_pub_destination );

        src->get_param( &dest->arrive_decision_distance );
        src->get_param( &dest->attemp_limits );

        return 0;
    }

    /*
     * @brief config file write
     * @param [in] fname : file name
     * @param [in] src   : configuration parameter
     */
    int fwrite_node_config( const QString& fname, node_config *src )
    {
        Q_ASSERT_X(fname.size() > 0, "invalid", "invalid null pointer argument\n" );
        Q_ASSERT_X(src, "invalid", "invalid null pointer argument\n" );

        int ret;
        gnd::ConfigFile fs;
        // convert configuration declaration
        if( (ret = set_node_config(&fs, src)) < 0 ) return ret;

        return fs.write_to_file(fname);
    }

    /**
     * @brief set config description
     * @param [out] dest : description
     * @param [in]   src : parameter
     */
    int set_node_config( gnd::ConfigFile *dest, node_config *src )
    {
        Q_ASSERT_X(dest, "invalid", "invalid null pointer argument\n" );
        Q_ASSERT_X(src, "invalid", "invalid null pointer argument\n" );

        // ros communication option
        dest->set_param( src->node_name );
        dest->set_param( src->path_file );
        dest->set_param( src->srv_name_add_destination );
        dest->set_param( src->srv_name_delete_destination );
        dest->set_param( src->srv_name_show_queue );

        dest->set_param( src->client_name_set_navi_path );
        dest->set_param( src->client_name_reset_pose );
        dest->set_param( src->client_name_get_waypoint );
        dest->set_param( src->client_name_is_intravelable_area );
        dest->set_param( src->client_name_find_waypoint );

        dest->set_param( src->topic_name_pose );
        dest->set_param( src->topic_name_status );
        dest->set_param( src->topic_name_queue_info );
        dest->set_param( src->topic_name_pub_destination );
        dest->set_param( src->arrive_decision_distance );
        dest->set_param( src->attemp_limits );

        return 0;
    }
}
// <--- function definition
#endif //NKM_DESTINATION_QUEUE_SET_DEST_CONFIG_H
