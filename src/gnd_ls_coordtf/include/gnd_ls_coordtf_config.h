/*
 * gnd_ls_coordtf_config.hpp
 *
 *  Created on: 2014/09/05
 *      Author: tyamada
 *  Updated by ryu, 2023/5/24
 *  .Use Qt v5.14 lib to update config file
 */
#ifndef GND_LS_COORDTF_CONFIG_H
#define GND_LS_COORDTF_CONFIG_H

#include <QString>
#include "gnd-configfile.h"

namespace gnd {
namespace ls_coordtf {

// ---> ros communicate option
static const Param<QString> Default_node_name = {
    "node-name",
    {"gnd_ls_coordtf"},
    "ros-node name"
};

static const Param<QString> Default_topic_name_laserscan = {
    "topic-name-laserscan",
    {"scan"},
    "laser scan topic name"
};

static const Param<QString> Default_topic_name_pointclouds_on_coordinate = {
    "topic-name-pointcloud-on-coordinate",
    {"coordinate/scan"},
    "ros topic name laser-scan's reflection point-cloud on defined coordinate (publish, sensor_msgs::PointCloud)"
};
// <--- ros communicate option

// ---> coordinate option
static const Param<double> Default_coordinate_origin = {
    "coordinate-origin",
    {0, 0, 0},
    "the source coordinate origin position on the destination coordinate"
};

static const Param<double> Default_axis_vector_front = {
    "coordinate-axis-vector-front",
    {1.0, 0.0, 0.0},
    "the source front axis on the destination coordinate (right hand system)"
};

static const Param<double> Default_axis_vector_upside = {
    "coordinate-axis-vector-upside",
    {0.0, 0.0, 1.0},
    "the source front axis on the destination coordinate (right hand system)"
};

// ---> debug option
static const Param<double> Default_period_cui_status_display = {
    "period-cui-status-display",
    {0.0},
    "display the node status in terminal. [note] it need ansi color code"
};

static const Param<QString> Default_text_log = {
    "text-log",
    {""},
    "text log file name"
};
// <--- debug option

/**
 * @brief configuration parameter for gnd_urg_proxy node
 */
struct node_config
{
    node_config();

    // ros communication
    Param<QString>  node_name;					    ///< node name
    Param<QString>	topic_name_laserscan;		///< topic name of laser scan (publish)
    Param<QString>	topic_name_pointcloud;			  ///< coordinate name

    // coordinate option
    Param<double>   coordinate_origin;			///< coordinate origin
    Param<double>		axis_vector_front;			///< front axis
    Param<double>		axis_vector_upside;			///< upside axis

    // debug option
    Param<double>	  period_cui_status_display;	///< cui status display mode
    Param<QString>	text_log;					          ///< text log
};

int init_node_config( node_config *p );
int fread_node_config( const QString &fname, node_config *dest );
int get_node_config( node_config *dest, ConfigFile *src );
int fwrite_node_config( const QString &fname, node_config *src );
int set_node_config( ConfigFile *dest, node_config *src );

} //End of namespace

} //End of namespace

namespace gnd {
namespace ls_coordtf {

// function definition
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
    p->node_name = Default_node_name;
    p->topic_name_laserscan = Default_topic_name_laserscan;
    p->topic_name_pointcloud = Default_topic_name_pointclouds_on_coordinate;
    // hard-ware parameter
    p->coordinate_origin = Default_coordinate_origin;
    p->axis_vector_front = Default_axis_vector_front;
    p->axis_vector_upside = Default_axis_vector_upside;
    // debug option
    p->period_cui_status_display = Default_period_cui_status_display;
    p->text_log = Default_text_log;

    return 0;
}

/*
 * @brief config file read
 * @param [in] fname : file name
 * @param [out] dest : configuration parameter
 */
int fread_node_config( const QString &fname, node_config *dest )
{
    Q_ASSERT_X(!fname.isEmpty(), "invalid", "invalid null pointer argument\n" );
    Q_ASSERT_X(dest, "invalid", "invalid null pointer argument\n" );

    int ret;
    ConfigFile fs;

    // configuration file read
    if( (ret = fs.read_from_file(fname)) < 0 )    return ret;

    return get_node_config(dest, &fs);
}

/*
 * @brief get config parameter from description
 * @param [out] dest : configuration parameter
 * @param [in]  src  : configuration description
 */
int get_node_config( node_config *dest, ConfigFile *src )
{
    Q_ASSERT_X(dest, "invalid", "invalid null pointer argument\n" );
    Q_ASSERT_X(src, "invalid", "invalid null pointer argument\n" );

    // ros communication parameter
    src->get_param( &dest->node_name );
    src->get_param( &dest->topic_name_laserscan );
    src->get_param( &dest->topic_name_pointcloud );
    // coordinate option
    src->get_param( &dest->coordinate_origin );
    src->get_param( &dest->axis_vector_front );
    src->get_param( &dest->axis_vector_upside );

    // debug option
    src->get_param( &dest->period_cui_status_display );
    src->get_param( &dest->text_log );

    return 0;
}

/*
 * @brief config file write
 * @param [in] fname : file name
 * @param [in] src   : configuration parameter
 */
int fwrite_node_config( const QString &fname, node_config *src )
{
    Q_ASSERT_X(!fname.isEmpty(), "invalid", "invalid null pointer argument\n" );
    Q_ASSERT_X(src, "invalid", "invalid null pointer argument\n" );

    int ret;
    ConfigFile fs;
    // convert configuration declaration
    if( (ret = set_node_config(&fs, src)) < 0 ) return ret;

    return fs.write_to_file(fname);
}

/*
 * @brief set config description
 * @param [out] dest : description
 * @param [in]   src : parameter
 */
int set_node_config( ConfigFile *dest, node_config *src )
{
  Q_ASSERT_X(dest, "invalid", "invalid null pointer argument\n" );
  Q_ASSERT_X(src, "invalid", "invalid null pointer argument\n" );

  // ros communication parameter
  dest->set_param(src->node_name );
  dest->set_param(src->topic_name_laserscan );
  dest->set_param(src->topic_name_pointcloud );
  // coordinate option
  dest->set_param(src->coordinate_origin );
  dest->set_param(src->axis_vector_front );
  dest->set_param(src->axis_vector_upside );

  // debug option
  dest->set_param(src->period_cui_status_display );
  dest->set_param(src->text_log );

  return 0;
}


} //End of namespace

} //End of namespace


#endif // GND_LS_COORDTF_CONFIG_H
