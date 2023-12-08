/*
 * @file hdk_pose_evaluator_config.hpp
 * @author Hidekazu TAKAHASHI
 * @date 2017/10/25
 *  *  Updated by ryu, 2023/10/18
 *  .Use Qt v5.14 lib to update config file
 */
#ifndef HDK_WAYPOINT_FINDER_CONFIG_H
#define HDK_WAYPOINT_FINDER_CONFIG_H

#include <QString>
#include <QtMath>
#include "gnd-configfile.h"
#include "gnd_rosutil.h"

// ---> const variables definition
namespace hdk {
	namespace waypoint_finder {

		// ---> ros commnication
        static const gnd::Param<QString> Default_node_name = {
                "node-name",
                {"waypoint_finder"},
                "ros-node name"
		};

        static const gnd::Param<QString> Default_topic_name_pose = {
				"topic-name-pose",
                {"pose"},
				"pose topic name, (subscribe, type:gnd_geometry2d_msgs/msg_pose2d_stamped)"
		};

        static const gnd::Param<QString> Default_service_name_find_waypoint = {
				"service-name-find-waypoint",
                {"find_waypoint"},
				"service name to find waypoint (server)"
		};

        static const gnd::Param<QString> Default_service_name_is_in_travelable_area = {
				"service-name-is-in-travelable-area",
                {"is_in_travelable_area"},
				"service name to check specified position is in travelable area or not (server)"
		};
		// <--- ros commnication


		// ---> path file option
        static const gnd::Param<QString> Default_path_map_file = {
				"path-map-file",
                {""},
				"path map file name"
		};
		// <--- path file option

    // ---> function declaration
    /**
     * @brief configuration parameter for node
     */
    struct node_config
    {
        node_config();
        // ros communication option
        gnd::Param<QString> node_name;							              //< ros-node name
        gnd::Param<QString> topic_name_pose;						          //< topic name(subscribe)
        gnd::Param<QString> service_name_find_waypoint;				    //< service name(server)
        gnd::Param<QString> service_name_is_in_travelable_area;	  //< service name(server)
        gnd::Param<QString> path_map_file;						            //< path map file
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
}
// <--- const variables definition

// ---> function definition
namespace hdk {
	namespace waypoint_finder {

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
        p->node_name                           = Default_node_name;
        p->topic_name_pose                     = Default_topic_name_pose;
        p->service_name_find_waypoint          = Default_service_name_find_waypoint;
        p->service_name_is_in_travelable_area  = Default_service_name_is_in_travelable_area;
        // path file option
        p->path_map_file                       = Default_path_map_file;

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
        src->get_param( &dest->topic_name_pose );
        src->get_param( &dest->service_name_find_waypoint );
        src->get_param( &dest->service_name_is_in_travelable_area );
        // map file option
        src->get_param( &dest->path_map_file );

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
        dest->set_param( src->topic_name_pose );
        dest->set_param( src->service_name_find_waypoint );
        dest->set_param( src->service_name_is_in_travelable_area );
        // map file option
        dest->set_param( src->path_map_file );

        return 0;
    }
	}
}
// <--- function definition

#endif
