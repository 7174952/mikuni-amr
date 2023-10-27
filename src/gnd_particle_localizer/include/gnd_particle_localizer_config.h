/*
 * particle_localizer_config.h
 *
 *  Created on: 2014/09/05
 *      Author: tyamada
 *       Brief: generate particles and check self location
 *  Updated by ryu, 2023/9/21
 *  .Use Qt v5.14 lib to update config file
 */
#ifndef GND_PARTICLE_LOCALIZER_CONFIG_H
#define GND_PARTICLE_LOCALIZER_CONFIG_H

#include <QString>
#include <QtMath>
#include "gnd-configfile.h"
#include "gnd_rosutil.h"

namespace gnd {
namespace particle_localizer {

// ---> ros communication
static const Param<QString> Default_node_name = {
    "node-name",
    {"particle_localizer"},
    "ros-node name"
};

static const Param<QString> Default_topic_name_pose = {
    "topic-pose",
    {"pose"},
    "estimated pose topic name, (publish, type:gnd_geometry2d_msgs/msg_pose2d_stamped)"
};

// ---> ros communication
static const Param<QString> Default_topic_name_particles = {
    "topic-particles",
    {"localization_particles"},
    "particles topic name, (publish, type:gnd_particle_localiser/msg_pose2darray_stamped)"
};

static const Param<QString> Default_topic_name_particle_weight = {
    "topic-particle-weights",
    {"localization_particles/weights"},
    "particle weights topic name, (subscribe, gnd_particle_localiser/msg_localization_particle_weight)"
};

static const Param<QString> Default_topic_name_motion = {
    "topic-motion",
    {"velocity"},
    "motion model topic name, (subscribe, type:gnd_geometry2d_msgs/msg_velocity2d_with_covariance_stamped)"
};

static const Param<QString> Default_service_name_reset_particles_nd = {
    "service-reset-particles-normal-distribution",
    {"reset_particles_nd"},
    "service name, (gnd_particle_localizer/srv_reset_particles_normal_distribution)"
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

// ---> initial parameter
static const Param<double> Default_initial_pose = {
    "initial-pose",
    {0.0, 0.0, gnd_ang2deg(0.0)},
    "initial pose"
};

static const Param<double> Default_error_covariance_initial_pose = {
    "standard-error-initial-pose",
    {0.3 * 0.3, 0.0,       0.0,
     0.0,       0.3 * 0.3, 0.0,
     0.0,       0.0,       gnd_deg2ang(10.0) * gnd_deg2ang(10.0)},
    "standard error of initial pose"
};
// <--- initial parameter

// ---> particle filter parameter
static const Param<double> Default_number_of_particles = {
    "number-of-particles",
    {100},
    "the number of particles"
};

static const Param<double> Default_period_resampling = {
    "period-resampling",
    {gnd_sec2time(1.0)},
    "period to resampling"
};

static const Param<double> Default_standard_systematic_motion_error = {
    "standard-systematic-motion-error",
    {0.025, 0.01, gnd_deg2ang(0.75)},
    "standard systematic error per translate 1.0 m"
};

static const Param<double> Default_probability_change_systematic_motion_error = {
    "probability-change-systematic-motion-error",
    {0.5},
    "probability of change systematic error parameter in resmapling"
};
// <--- particle filter parameter

/**
 * @brief configuration parameter for node
 */
struct node_config
{
    node_config();
    // ros communication option
    Param<QString> node_name;							                     //< ros-node name
    Param<QString> topic_name_pose;						                 //< estimated pose topic name
    Param<QString> topic_name_particles;				               //< particles topic name
    Param<QString>  topic_name_particle_weights;						   //< particle weights topic name
    Param<QString>  topic_name_motion;						             //< motion model topic name
    Param<QString> service_name_reset_particles_nd;				     //< reset particle service name
    // initial position option
    Param<double>  initial_pose;                               //< mean of estimated initial pose
    Param<double>  error_covariance_initial_pose;              //< error co-variance of initial pose
    // particle filter parameter
    Param<double>  number_of_particles;		                     //< cthe number of particles
    Param<double>  period_resampling;                          //< cycle to resampling
    Param<double>  standard_systematic_motion_error;
    Param<double>  probability_change_systematic_motion_error;
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
namespace particle_localizer {

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
    p->node_name                                  = Default_node_name;
    p->topic_name_pose                            = Default_topic_name_pose;
    p->topic_name_particles                       = Default_topic_name_particles;
    p->topic_name_particle_weights                = Default_topic_name_particle_weight;
    p->topic_name_motion                          = Default_topic_name_motion;
    p->service_name_reset_particles_nd            = Default_service_name_reset_particles_nd;
    // initial pose option
    p->initial_pose                               = Default_initial_pose;
    p->error_covariance_initial_pose              = Default_error_covariance_initial_pose;
    // particle filter parameter
    p->number_of_particles                        = Default_number_of_particles;
    p->period_resampling                          = Default_period_resampling;
    p->standard_systematic_motion_error           = Default_standard_systematic_motion_error;
    p->probability_change_systematic_motion_error = Default_probability_change_systematic_motion_error;

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

    // ros communication parameter
    src->get_param( &dest->node_name );
    src->get_param( &dest->topic_name_pose );
    src->get_param( &dest->topic_name_particles );
    src->get_param( &dest->topic_name_particle_weights );
    src->get_param( &dest->topic_name_motion );
    src->get_param( &dest->service_name_reset_particles_nd );
    // initial pose option
    src->get_param( &dest->initial_pose );
    src->get_param( &dest->error_covariance_initial_pose );
    if( src->get_param(&dest->standard_systematic_motion_error ) >= 3)
    {
        dest->standard_systematic_motion_error.value[2] = gnd_deg2ang(dest->standard_systematic_motion_error.value.at(2));
    }
    src->get_param( &dest->probability_change_systematic_motion_error );
    // particle filter parameter
    src->get_param( &dest->number_of_particles );
    src->get_param( &dest->period_resampling );

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
    dest->set_param( src->topic_name_particles );
    dest->set_param( src->topic_name_particle_weights );
    dest->set_param( src->topic_name_motion );
    dest->set_param( src->service_name_reset_particles_nd );
    // initial pose option
    dest->set_param( src->initial_pose );
    dest->set_param( src->error_covariance_initial_pose );
    // particle filter parameter
    dest->set_param( src->number_of_particles );
    dest->set_param( src->period_resampling );
    src->standard_systematic_motion_error.value[2] = gnd_ang2deg(src->standard_systematic_motion_error.value.at(2));
    dest->set_param( src->standard_systematic_motion_error );
    src->standard_systematic_motion_error.value[2] = gnd_deg2ang(src->standard_systematic_motion_error.value.at(2));
    dest->set_param( src->probability_change_systematic_motion_error );

    return 0;
}


} //End of namespace

} //End of namespace

#endif // GND_LSSMAP_MAKER_CONFIG_H
