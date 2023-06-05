/*
 * gnd_lssmap_maker_config.h
 *
 *  Created on: 2014/09/05
 *      Author: tyamada
 *       Brief: Laser Scan Statistics MAP MAKER node CONFIG definition
 *  Updated by ryu, 2023/5/25
 *  .Use Qt v5.14 lib to update config file
 */
#ifndef GND_LSSMAP_MAKER_CONFIG_H
#define GND_LSSMAP_MAKER_CONFIG_H

#include <QString>
#include <QtMath>
#include "gnd-configfile.h"

namespace gnd {
namespace lssmap_maker {

#define gnd_unit_cm2m_scale		(0.01)
#define gnd_unit_m2cm_scale		(100.0)
#define gnd_cm2m(x)				((x) * gnd_unit_cm2m_scale)
#define gnd_m2cm(x)				((x) * gnd_unit_m2cm_scale)

// ---> ros communication
static const Param<QString> Default_node_name = {
    "node-name",
    {"lssmap_maker"},
    "ros-node name"
};

static const Param<QString> Default_topic_name_pose = {
    "topic-pose",
    {"gyrodometry"},
    "global pose topic (subscribe)"
};

static const Param<QString> Default_topic_name_pointcloud = {
    "topic-laserscan-point",
    {"coordinate/scan"},
    "laser scan point topic, type PointCloud on robot coordinate (subscribe)"
};
// <--- ros communication


// ---> map option
static const Param<double> Default_map_origin = {
    "map-origin",
    {0.0, 0.0, 0.0},
    "map origin position"
};

static const Param<double> Default_map_x_axis = {
    "map-x-axis",
    {1.0, 0.0},
    "The vector that represent the x axis direction of the map"
};

static const Param<QString> Default_initial_counting_map = {
    "initial-counting-map-directory",
    {""},
    "directory that include counting map statistics files. this option is to merge sensor data that is obtained in other time"
};


static const Param<double> Default_counting_map_cell_size = {
    "counting-map-cell-size",
    {gnd_cm2m(80)},
    "cell size for counting to calculate variance and means (m)"
};

static const Param<double> Default_image_map_pixel_size = {
    "image-map-pixel-size",
    {gnd_cm2m(10)},
    "image map pixel size (m)"
};

static const Param<double> Default_additional_smoothing_parameter = {
    "additional-smoothing-parameter",
    {100},
    "additional smoothing parameter"
};

static const Param<double> Default_sensor_range = {
    "sensor-range",
    {30},
    "sensor range (m)"
};
// <--- map option



// ---> data collect option
static const Param<double> Default_collect_condition_ignore_range_lower= {
    "collect-condition-ignore-range-lower",
    {gnd_cm2m(20)},
    "ignore laser scan data (m)"
};

static const Param<double> Default_collect_condition_ignore_range_upper = {
    "collect-condition-ignore-range-upper",
    {-1},
    "ignore laser scan data (m)"
};

static const Param<double> Default_collect_condition_culling_distance = {
    "collect-condition-culling-distance",
    {gnd_cm2m(5)},
    "distance condition to ignore data in laser scan data counting (m)"
};

static const Param<double> Default_collect_condition_moving_distance = {
    "collect-condition-moving-distance",
    {gnd_cm2m(5)},
    "data collect condition of moving distance(m), [note] if this parameter is <=0, not collect data under the moving distance condition"
};

static const Param<double> Default_collect_condition_moving_angle = {
    "collect-condition-moving-angle",
    {qDegreesToRadians(90.0)},
    "data collect condition of moving angle(deg), [note] if this parameter is <=0, not collect data under the moving angle condition"
};

static const Param<double> Default_collect_condition_time = {
    "collect-condition-time",
    {0},
    "data collect condition (sec), [note] if this parameter is <=0, not collect data under the time condition"
};
// <--- data collect option



// ---> debug condition
static const Param<double> Default_cycle_status_display = {
    "period-cui-status-display",
    {1},
    "period to display the node status in standard output [msec]. [note] it need ansi color code. [note] if this value is less than or equal 0, don't display"
};

static const Param<QString> Default_text_log_pointcloud = {
    "text-log-pointcloud",
    {"point_cloud.txt"},
    "text log file name. this file is point cloud data. [note] if this parameter is null, the file is not created."
};

static const Param<QString> Default_text_log_trajecotry = {
    "text-log-trajectory",
    {"trajectory.txt"},
    "text log file name. this file is robot trajectory data. [note] if this parameter is null, the file is not created."
};

static const Param<QString> Default_map_folder_path = {
    "map-folder-path",
    {"/home/mikuni/catkin_ws/src/amr_ros/maps"},
    "set saving map folder path."
};

/**
 * @brief configuration parameter for gnd_urg_proxy node
 */
struct node_config
{
  node_config();
  // ros communication option
  Param<QString> node_name;							                //< node name for ros communication
  Param<QString> topic_name_pose;						            //< pose topic name for ros communication
  Param<QString> topic_name_pointcloud;				          //< pointcloud topic name for ros communication
  // map make option
  Param<double>  map_origin;						                //< map origin position
  Param<double>  map_x_axis;						                //< the vector that represent the x axis direction of the map
  Param<QString> initial_counting_map;				          //< initial counting map
  Param<double>  counting_map_cell_size;				        //< counting cell size
  Param<double>  image_map_pixel_size;				          //< image map pixel size
  Param<double>  additional_smoothing_parameter;		    //< additional smoothing parameter
  Param<double>  sensor_range;						              //< sensor range
  // data collect option
  Param<double>  collect_condition_ignore_range_lower;  //< ignore range
  Param<double>  collect_condition_ignore_range_upper;  //< ignore upper
  Param<double>  collect_condition_culling_distance;	  //< culling distance
  Param<double>  collect_condition_moving_distance;	    //< data collect condition (moving distance)
  Param<double>  collect_condition_moving_angle;		    //< data collect condition (moving angle)
  Param<double>  collect_condition_time;				        //< data collect condition (time)
  // debug option
  Param<double>  period_cui_status_display;			        //< cui status display mode
  Param<QString> text_log_pointcloud;							      //< text log file name
  Param<QString> text_log_trajectory;							      //< text log file name
  Param<QString> map_folder_path;							          //< map folder paht
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
int get_node_config( node_config *dest, ConfigFile *src );

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
int set_node_config( ConfigFile *dest, node_config *src );

} //End of namespace

} //End of namespace

namespace gnd {
namespace lssmap_maker {

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

    // ros communication option
    p->node_name                            = Default_node_name;
    p->topic_name_pose                      = Default_topic_name_pose;
    p->topic_name_pointcloud                = Default_topic_name_pointcloud;
    // map make option
    p->map_origin                           = Default_map_origin;
    p->map_x_axis                           = Default_map_x_axis;
    p->initial_counting_map                 = Default_initial_counting_map;
    p->counting_map_cell_size               = Default_counting_map_cell_size;
    p->image_map_pixel_size                 = Default_image_map_pixel_size;
    p->additional_smoothing_parameter       = Default_additional_smoothing_parameter;
    p->sensor_range                         = Default_sensor_range;
    p->collect_condition_ignore_range_lower = Default_collect_condition_ignore_range_lower;
    p->collect_condition_ignore_range_upper = Default_collect_condition_ignore_range_upper;
    p->collect_condition_culling_distance   = Default_collect_condition_culling_distance;
    p->collect_condition_moving_distance    = Default_collect_condition_moving_distance;
    p->collect_condition_moving_angle       = Default_collect_condition_moving_angle;
    p->collect_condition_time               = Default_collect_condition_time;
    // debug option
    p->period_cui_status_display            = Default_cycle_status_display;
    p->text_log_pointcloud                  = Default_text_log_pointcloud;
    p->text_log_trajectory                  = Default_text_log_trajecotry;
    p->map_folder_path                      = Default_map_folder_path;

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

    // ros communication option
    src->get_param( &dest->node_name );
    src->get_param( &dest->topic_name_pose );
    src->get_param( &dest->topic_name_pointcloud );
    // map maker option
    src->get_param( &dest->map_origin );
    src->get_param( &dest->map_x_axis );
    src->get_param( &dest->initial_counting_map );
    src->get_param( &dest->counting_map_cell_size );
    src->get_param( &dest->image_map_pixel_size );
    src->get_param( &dest->additional_smoothing_parameter );
    src->get_param( &dest->sensor_range );
    // data collect option
    src->get_param( &dest->collect_condition_ignore_range_lower );
    src->get_param( &dest->collect_condition_ignore_range_upper );
    src->get_param( &dest->collect_condition_culling_distance );
    src->get_param( &dest->collect_condition_moving_distance );
    if( src->get_param( &dest->collect_condition_moving_angle ) >= 0)
    {
        dest->collect_condition_moving_angle.value[0] = qDegreesToRadians(dest->collect_condition_moving_angle.value[0]);
    }
    src->get_param( &dest->collect_condition_time );
    // debug option
    src->get_param( &dest->period_cui_status_display );
    src->get_param( &dest->text_log_pointcloud );
    src->get_param( &dest->text_log_trajectory );
    src->get_param( &dest->map_folder_path );

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
    ConfigFile fs;
    // convert configuration declaration
    if( (ret = set_node_config(&fs, src)) < 0 ) return ret;

    return fs.write_to_file(fname);
}

/**
 * @brief set config description
 * @param [out] dest : description
 * @param [in]   src : parameter
 */
int set_node_config( ConfigFile *dest, node_config *src )
{
    Q_ASSERT_X(dest, "invalid", "invalid null pointer argument\n" );
    Q_ASSERT_X(src, "invalid", "invalid null pointer argument\n" );

    // ros communication option
    dest->set_param( src->node_name );
    dest->set_param( src->topic_name_pose );
    dest->set_param( src->topic_name_pointcloud );
    // map maker option
    dest->set_param( src->map_origin );
    dest->set_param( src->map_x_axis );
    dest->set_param( src->initial_counting_map );
    dest->set_param( src->counting_map_cell_size );
    dest->set_param( src->image_map_pixel_size );
    dest->set_param( src->additional_smoothing_parameter );
    dest->set_param( src->sensor_range );
    // data collect option
    dest->set_param( src->collect_condition_ignore_range_lower );
    dest->set_param( src->collect_condition_ignore_range_upper );
    dest->set_param( src->collect_condition_culling_distance );
    dest->set_param( src->collect_condition_moving_distance );
    src->collect_condition_moving_angle.value[0] = qRadiansToDegrees(src->collect_condition_moving_angle.value[0]);
    dest->set_param( src->collect_condition_moving_angle );
    src->collect_condition_moving_angle.value[0] = qDegreesToRadians(src->collect_condition_moving_angle.value[0]);

    dest->set_param( src->collect_condition_time );
    // debug option
    dest->set_param( src->period_cui_status_display );
    dest->set_param( src->text_log_pointcloud );
    dest->set_param( src->text_log_trajectory );
    dest->set_param( src->map_folder_path );

    return 0;
}


} //End of namespace

} //End of namespace

#endif // GND_LSSMAP_MAKER_CONFIG_H
