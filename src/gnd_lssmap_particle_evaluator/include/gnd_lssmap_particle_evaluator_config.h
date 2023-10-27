/*
 * gnd_lssmap_particle_evaluator_config.h
 *
 *  Created on: 2014/09/05
 *      Author: tyamada
 *       Brief: Calculate particles evaluate and compared on map
 *  Updated by ryu, 2023/9/19
 *  .Use Qt v5.14 lib to update config file
 */
#ifndef GND_LSSMAP_PARTICLE_EVALUTOR_CONFIG_H
#define GND_LSSMAP_PARTICLE_EVALUTOR_CONFIG_H

#include <QString>
#include <QtMath>
#include "gnd-configfile.h"
#include "gnd_rosutil.h"


namespace gnd {
namespace lssmap_particle_evaluator {

// ---> map file option
static const Param<QString> Default_bmp_map = {
    "bmp-map-file",
    {""},
    "bmp map file path"
};

static const Param<QString> Default_bmp_map_origin = {
    "bmp-map-origin-file",
    {""},
    "bmp map origin file path"
};
// <--- map file option

// ---> ros communication
static const Param<QString> Default_node_name = {
    "node-name",
    {"lssmap_particle_evaluator"},
    "ros-node name"
};

static const Param<QString> Default_topic_name_particles = {
    "topic-particles",
    {"localization_particles"},
    "particles topic name, (subscribe, type:gnd_particle_localiser/msg_pose2darray_stamped)"
};

static const Param<QString> Default_topic_name_particle_weight = {
    "topic-particle-weights",
    {"localization_particles/weights"},
    "particle weights topic name, (publish, gnd_particle_localiser/msg_localization_particle_weight)"
};

static const Param<QString> Default_topic_name_pointcloud = {
    "topic-point-cloud",
    {"point_cloud"},
    "point-cloud topic name, [note] this node deal the point cloud data that is 2d laser-scanner data and it's coordinate origin should be robot position (subscribe, type:sensor_msgs::PointCloud)"
};
// <--- ros communication

// ---> time synchronization
static const Param<QString> Default_service_name_get_synctimer = {
    "service-name-get-synctime",
    {"get_synctime"},
    "service name to get time for synchronization"
};

static const Param<double> Default_period_time_synchronize = {
    "period-time-synchronize",
    {10.0},
    "period for time synchronization"
};
// <--- time synchronization

// ---> operating option
static const Param<double> Default_period = {
    "period",
    {gnd_msec2sec(1000)},
    "period time [sec]"
};

static const Param<double> Default_points_select_condition_height_range = {
    "points-select-condition-height-range",
    {0.0, -1.0},
    "height range to select points that use particles evaluation. #1 range lower, #2 range upper. [note] if lower > upper, this item is ignored."
};

static const Param<double> Default_points_select_condition_ignore_horizontal_range_upper = {
    "points-select-condition-ignore-horizontal-range-upper",
    {-1},
    "horizontal distance range to select points that use particles evaluation. if a point is farther than this value, it is ignored. [note] if this value < 0, this item is ignored."
};

static const Param<double> Default_points_select_condition_ignore_horizontal_range_lower = {
    "points-select-condition-ignore-horizontal-range-lower",
    {0.02},
    "horizontal distance range to select points that use particles evaluation. if a point is nearer than this value, it is ignored. [note] if this value < 0, this item is ignored."
};

static const Param<double> Default_points_select_condition_culling_distance = {
    "points-select-condition-culling-distance",
    {gnd_cm2m(5)},
    "if contiguous points are nearer than this value, these point is ignored. [note] if this value < 0, this item is ignored."
};
// <--- operating option


/**
 * @brief configuration parameter for node
 */
struct node_config
{
    node_config();
    // map file option
    Param<QString> bmp_map;							                                      //< map file (bitmap)
    Param<QString> bmp_map_origin;						                                //< map origin file
    Param<QString> node_name;				                                          //< ros-node name
    // ros communication option
    Param<QString>  topic_name_particles;						                          //< particles topic name
    Param<QString>  topic_name_particle_weight;						                    //< particle weights topic name
    Param<QString> topic_name_pointcloud;				                              //< pointcloud topic name
    // time synchronization
    Param<QString>  service_name_get_synctimer;
    Param<double>  period_time_synchronize;
    // operating option
    Param<double>  period;		                                                //< cycle to resampling
    Param<double>  points_select_condition_height_range;                      //< point select condition
    Param<double>  points_select_condition_ignore_horizontal_range_upper;     //< point select condition
    Param<double>  points_select_condition_ignore_horizontal_range_lower;     //< point select condition
    Param<double>  points_select_condition_culling_distance;	                //< point select condition
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
namespace lssmap_particle_evaluator {

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

    // map file option
    p->bmp_map                                               = Default_bmp_map;
    p->bmp_map_origin                                        = Default_bmp_map_origin;
    // ros communication option
    p->node_name                                             = Default_node_name;
    p->topic_name_particles                                  = Default_topic_name_particles;
    p->topic_name_particle_weight                            = Default_topic_name_particle_weight;
    p->topic_name_pointcloud                                 = Default_topic_name_pointcloud;
    // time synchronization
    p->service_name_get_synctimer                            = Default_service_name_get_synctimer;
    p->period_time_synchronize                               = Default_period_time_synchronize;
    // operating option
    p->period                                                = Default_period;
    p->points_select_condition_height_range                  = Default_points_select_condition_height_range;
    p->points_select_condition_ignore_horizontal_range_lower = Default_points_select_condition_ignore_horizontal_range_lower;
    p->points_select_condition_ignore_horizontal_range_upper = Default_points_select_condition_ignore_horizontal_range_upper;
    p->points_select_condition_culling_distance              = Default_points_select_condition_culling_distance;

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

    // map file option
    src->get_param( &dest->bmp_map );
    src->get_param( &dest->bmp_map_origin );
    // ros communication option
    src->get_param( &dest->node_name );
    src->get_param( &dest->topic_name_particles );
    src->get_param( &dest->topic_name_particle_weight );
    src->get_param( &dest->topic_name_pointcloud );
    // time synchronization
    src->get_param( &dest->service_name_get_synctimer );
    src->get_param( &dest->period_time_synchronize );
    // operating option
    src->get_param( &dest->period );
    src->get_param( &dest->points_select_condition_height_range );
    src->get_param( &dest->points_select_condition_ignore_horizontal_range_lower );
    src->get_param( &dest->points_select_condition_ignore_horizontal_range_upper );
    src->get_param( &dest->points_select_condition_culling_distance );

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

    // map file option
    dest->set_param( src->bmp_map );
    dest->set_param( src->bmp_map_origin );
    // ros communication option
    dest->set_param( src->node_name );
    dest->set_param( src->topic_name_particles );
    dest->set_param( src->topic_name_particle_weight );
    dest->set_param( src->topic_name_pointcloud );
    // time synchronization
    dest->set_param( src->service_name_get_synctimer );
    dest->set_param( src->period_time_synchronize );
    // operating option
    dest->set_param( src->period );
    dest->set_param( src->points_select_condition_height_range );
    dest->set_param( src->points_select_condition_ignore_horizontal_range_lower );
    dest->set_param( src->points_select_condition_ignore_horizontal_range_upper );
    dest->set_param( src->points_select_condition_culling_distance );

    return 0;
}


} //End of namespace

} //End of namespace

#endif // GND_LSSMAP_MAKER_CONFIG_H
