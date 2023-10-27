/*
 * icartmini_sbtp_config.h
 *
 *  Created on: 2014/09/05
 *      Author: tyamada
 *       Brief: deterimine running route
 *  Updated by ryu, 2023/9/27
 *  .Use Qt v5.14 lib to update config file
 */
#ifndef ICARTMINI_SBTP_CONFIG_H
#define ICARTMINI_SBTP_CONFIG_H

#include <QString>
#include <QtMath>
#include "gnd-configfile.h"
#include "gnd_rosutil.h"

namespace gnd {
namespace icartmini_sbtp {

// ---> ros communication
static const Param<QString> Default_route_file = {
    "route-file",
    {"/home/mikuni/catkin_ws/src/amr_ros/maps/maps.path"},
    "route data file path"
};

static const Param<QString> Default_start_node_name = {
    "start",
    {""},
    "start path-node name"
};

// ---> ros communication
static const Param<QString> Default_destination_node_name = {
    "destination",
    {""},
    "destination path-node name"
};

static const Param<QString> Default_node_name = {
    "node-name",
    {"icartmini_sbtp"},
    "ros-node name"
};

static const Param<QString> Default_topic_name_pose = {
    "topic-pose",
    {"pose_particle_localizer"},
    "estimated pose topic name, (subscribe, type: gnd_msgs/msg_pose2d_stamped)"
};

static const Param<QString> Default_topic_names_pointcloud = {
    "topic-point-cloud",
    {"coordinate/scan"},
    "point cloud topic name for obstacle detection, (subscribe, gnd_particle_localiser/msg_localization_particle_weight)"
};
// <--- ros communication

static const Param<QString> Default_topic_name_vehicle_vel = {
    "topic-vehicle-velocity",
    {"vehicle_vel"},
    "vehicle velocity topic name for control motion, (subscribe, gnd_msgs/msg_velocity2d_with_covariance_stamped)"
};

static const Param<QString> Default_topic_name_planned_path = {
    "topic-planned-path",
    {"planned_path"},
    "planned path topic name, (publish, gnd_msgs/msg_paths_speed_limited)"
};

static const Param<QString> Default_topic_name_vehicle_ctrl = {
    "topic-vehiclectrl",
    {"vehicle_ctrl"},
    "vehicle control command topic name (publish jwvehicle_proxy/msg_ctrlcmd)"
};

static const Param<QString> Default_topic_name_trajectory_target = {
    "topic-trajectory-target",
    {"target_trajectory"},
    "current target trajectory topic name (publish gnd_msgs/msg_trajectory)"
};

static const Param<QString> Default_topic_name_trajectory_actual = {
    "topic-trajectory-actual",
    {"actual_trajectory"},
    "current actual trajectory topic name (publish gnd_msgs/msg_trajectory)"
};

static const Param<double> Default_period_publish_planning = {
    "period-publish-target-trajectory",
    {1.0},
    "period of trajectory publish"
};

static const Param<QString> Default_topic_name_vehicle_status = {
    "topic-vehicle-status",
    {"icartmini_sbtp/vehicle_status"},
    "current vehicle status topic name (publish gnd_msgs/msg_vehicle_status)"
};

static const Param<QString> Default_service_name_set_navigation_path = {
    "service-set-navigation-path",
    {"set_navigation_path"},
    "set navigation path service name (jwvehicle_sbtp/srv_set_navigation_path)"
};

static const Param<QString> Default_topic_name_external_controll_velocity = {
    "topic-external-control-velocity",
    {"external_limit_velocity"},
    "external controll velocity topic name (subscribe std_msgs/Float64)"
};
// <--- ros communication


// ---> vehicle size
static const Param<double> Default_vehicle_length_front = {
    "vehicle-length-front",
    {0.12},
    "vehicle size"
};
static const Param<double> Default_vehicle_length_back = {
    "vehicle-length-back",
    {0.4},
    "vehicle size"
};
static const Param<double> Default_vehicle_width_left = {
    "vehicle-length-left",
    {0.2},
    "vehicle size"
};
static const Param<double> Default_vehicle_width_right = {
    "vehicle-length-right",
    {0.2},
    "vehicle size"
};
// <--- vehicle size


// ---> vehicle velocity option
static const Param<double> Default_max_velocity = {
    "max-velocity",
    {0.5},
    "maximum velocity"
};

static const Param<double> Default_max_angular_velocity = {
    "max-angular-velocity",
    {gnd_deg2ang(60)},
    "maximum angular velocity"
};

static const Param<double> Default_acceleration = {
    "acceleration",
    {0.01},
    "maximum acceleration of target velocity"
};

static const Param<double> Default_deceleration = {
    "deceleration",
    {0.3},
    "maximum deceleration of target velocity except in the case of collision avoidance"
};
// <--- vehicle velocity option


// ---> navigation option
static const Param<double> Default_pathend_margin = {
    "pathend-margin",
    {0.1},
    "when the robot is closer than this value to the path-end, the robot figure out that it arrive at the path-end."
};

static const Param<double> Default_period_planning = {
    "period-planning",
    {0.05},
    "motion planning period(sec)"
};

static const Param<double> Default_period_reselect_trajectory = {
    "period-reselect-trajectory",
    {3.0},
    "trajectory reselect period(sec)"
};

static const Param<double> Default_clearance_required = {
    "clearance-required",
    {0.10},
    "required clearance"
};

static const Param<double> Default_clearance_margin = {
    "clearance-margin",
    {0.15},
    "if the trajectory have clearance of required and margin clearance, the trajectory have enough clearance"
};

static const Param<double> Default_discretized_interval_of_target_trajectory = {
    "discretized-interval-of-target-trajectory",
    {0.05},
    "the discretized interval of target trajectories for its selection"
};

static const Param<double> Default_distance_threshold_mediate_trajectory = {
    "distance-threshold-mediate-trajectory",
    {0.15},
    "the distance threshold for planning the mediate trajectory"
};

static const Param<double> Default_depth_for_trajectory_selection = {
    "depth-for-trajectory-selection",
    {5.0},
    "the maximum of depth for the target trajectory selection"
};

static const Param<double> Default_depth_for_slow_down = {
    "depth-for-slow-down",
    {2.0},
    "when the depth is lower than this value, the robot will be slow down."
};

static const Param<double> Default_depth_for_stop = {
    "depth-for-stop",
    {1.0},
    "when the depth is lower than this value, the robot will stop."
};


static const Param<double> Default_target_trajecotry_weight_depth = {
    "target-trajectory-weight-depth",
    {0.4},
    "the weight of depth for the target trajectory selection. the depth means how long the trajectory can proceed."
};


static const Param<double> Default_target_trajecotry_weight_clearance = {
    "target-trajectory-weight-clearance",
    {0.4},
    "the weight of clearance for the target trajectory selection."
};

static const Param<double> Default_target_trajecotry_weight_distance = {
    "target-trajectory-weight-distance",
    {0.2},
    "the weight of distance for the target trajectory selection. the distance is how long from the robot to the trajectory."
};

static const Param<double> Default_mediate_trajectory_search_anguler_size = {
    "mediate-trajectory-search-angular-size",
    {gnd_deg2ang(1.0)},
    "the angular size for mediate trajectory search [degree]"
};

static const Param<double> Default_mediate_trajectory_serch_anguler_size_for_avoidance = {
    "mediate-trajectory-search-angular-size-for-avoidance",
    {gnd_deg2ang(90.0)},
    "the angular size for avoidance mediate trajectory search [degree]"
};

static const Param<double> Default_mediate_trajectory_search_angular_range = {
    "mediate-trajectory-search-angular-range",
    {gnd_deg2ang(30.0)},
    "the limit of angular range for mediate trajectory search [degree]"
};

static const Param<double> Default_mediate_trajectory_search_range = {
    "mediate-trajectory-search-range",
    {1.0},
    "the limit of range for mediate trajectory search [m]"
};

static const Param<double> Default_mediate_trajectory_weight_clearance = {
    "mediate-trajectory-weight-clearance",
    {0.6},
    "the weight of clearance for the mediate trajectory selection"
};

static const Param<double> Default_mediate_trajectory_weight_direction = {
    "mediate-trajectory-weight-direction",
    {0.4},
    "the weight of direction for the mediate trajectory selection"
};
// <--- icartmini sbtp parameter

/**
 * @brief configuration parameter for node
 */
struct node_config
{
    node_config();
    // ros communication option
    Param<QString> path_file;							                 //path file
    Param<QString> start_node;						                 //start node name
    Param<QString> dest_node;				                       //destination node name

    Param<QString>  node_name;						                 //node name
    Param<QString>  topic_name_pose;						           //pose topic
    Param<QString> topic_names_pointcloud;				         //point cloud topic
    // initial position option
    Param<QString>  topic_name_vehicle_vel;                //vehicle velocity topic
    // particle filter parameter
    Param<QString>  topic_name_planned_path;		           //planed path
    Param<QString>  topic_name_vehicle_ctrl;               //vehicle ctrl topic
    Param<QString>  topic_name_trajectory_target;          //target trajectory
    Param<QString>  topic_name_trajectory_actual;          //actual trajectory
    Param<double>   period_publish_planning;
    Param<QString>  topic_name_vehicle_status;             //vehicle status
    Param<QString>  topic_name_external_controll_velocity; //external_control_velocity
    Param<QString>  service_name_set_navigation_path;      //set navigation path service
    // vehicle size
    Param<double>  vehicle_length_front;
    Param<double>  vehicle_length_back;
    Param<double>  vehicle_width_left;
    Param<double>  vehicle_width_right;
    // velocity option
    Param<double>  max_velocity;
    Param<double>  max_angular_velocity;
    Param<double>  acceleration;
    Param<double>  deceleration;
    // navigation option
    Param<double>  period_planning;
    Param<double>  period_reselect_trajectory;
    Param<double>  clearance_required;
    Param<double>  clearance_margin;
    Param<double>  pathend_margin;
    Param<double>  discretized_interval_of_target_trajectory;
    Param<double>  distance_threshold_intermediate;
    Param<double>  depth_for_trajectory_selection;
    Param<double>  depth_for_slow_down;
    Param<double>  depth_for_stop;
    Param<double>  target_trajectory_weight_depth;
    Param<double>  target_trajecotry_weight_clearance;
    Param<double>  target_trajecotry_weight_distance;
    // accompany option
    Param<double>  distance_to_slowdown_for_companion;
    Param<double>  distance_to_wait_for_companion;
    Param<double>  mediate_trajectory_search_angular_size;
    Param<double>  mediate_trajectory_search_angular_size_for_avoidance;
    Param<double>  mediate_trajectory_search_angular_range;
    Param<double>  mediate_trajectory_search_range;
    Param<double>  mediate_trajectory_weight_clearance;
    Param<double>  mediate_trajectory_weight_direction;
};

/**
 * @brief initialize configure to default parameter
 * @param [out] p : node_config
 */
int init_node_config(struct node_config *p);

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
namespace icartmini_sbtp {

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
    p->path_file                             = Default_route_file;
    p->start_node                            = Default_start_node_name;
    p->dest_node                             = Default_destination_node_name;
    p->node_name                             = Default_node_name;
    p->topic_name_pose                       = Default_topic_name_pose;
    p->topic_names_pointcloud                = Default_topic_names_pointcloud;
    // initial pose option
    p->topic_name_vehicle_vel                = Default_topic_name_vehicle_vel;
    p->topic_name_vehicle_ctrl               = Default_topic_name_vehicle_ctrl;
    // particle filter parameter
    p->topic_name_planned_path               = Default_topic_name_planned_path;
    p->topic_name_trajectory_target          = Default_topic_name_trajectory_target;
    p->topic_name_trajectory_actual          = Default_topic_name_trajectory_actual;
    p->period_publish_planning               = Default_period_publish_planning;
    p->topic_name_vehicle_status             = Default_topic_name_vehicle_status;
    p->topic_name_external_controll_velocity = Default_topic_name_external_controll_velocity;
    p->service_name_set_navigation_path      = Default_service_name_set_navigation_path;
    // vehicle size
    p->vehicle_length_front                  = Default_vehicle_length_front;
    p->vehicle_length_back                   = Default_vehicle_length_back;
    p->vehicle_width_left                    = Default_vehicle_width_left;
    p->vehicle_width_right                   = Default_vehicle_width_right;
    // velocity option
    p->max_velocity                          = Default_max_velocity;
    p->max_angular_velocity                  = Default_max_angular_velocity;
    p->acceleration                          = Default_acceleration;
    p->deceleration                          = Default_deceleration;
    // navigation option
    p->period_planning                       = Default_period_planning;
    p->period_reselect_trajectory            = Default_period_reselect_trajectory;
    p->clearance_required                    = Default_clearance_required;
    p->clearance_margin                      = Default_clearance_margin;
    p->discretized_interval_of_target_trajectory            = Default_discretized_interval_of_target_trajectory;
    p->distance_threshold_intermediate                      = Default_distance_threshold_mediate_trajectory;
    p->depth_for_trajectory_selection                       = Default_depth_for_trajectory_selection;
    p->depth_for_slow_down                                  = Default_depth_for_slow_down;
    p->depth_for_stop                                       = Default_depth_for_stop;
    p->pathend_margin                                       = Default_pathend_margin;
    p->target_trajectory_weight_depth                       = Default_target_trajecotry_weight_depth;
    p->target_trajecotry_weight_clearance                   = Default_target_trajecotry_weight_clearance;
    p->target_trajecotry_weight_distance                    = Default_target_trajecotry_weight_distance;
    p->mediate_trajectory_search_angular_size               = Default_mediate_trajectory_search_anguler_size;
    p->mediate_trajectory_search_angular_size_for_avoidance = Default_mediate_trajectory_serch_anguler_size_for_avoidance;
    p->mediate_trajectory_search_angular_range              = Default_mediate_trajectory_search_angular_range;
    p->mediate_trajectory_search_range                      = Default_mediate_trajectory_search_range;
    p->mediate_trajectory_weight_clearance                  = Default_mediate_trajectory_weight_clearance;
    p->mediate_trajectory_weight_direction                  = Default_mediate_trajectory_weight_direction;

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
    src->get_param( &dest->path_file );
    src->get_param( &dest->start_node );
    src->get_param( &dest->dest_node );
    src->get_param( &dest->node_name );
    src->get_param( &dest->topic_name_pose );
    src->get_param( &dest->topic_names_pointcloud );
    // initial pose option
    src->get_param( &dest->topic_name_vehicle_vel );
    src->get_param( &dest->topic_name_planned_path );

    src->get_param( &dest->topic_name_vehicle_ctrl );

    src->get_param( &dest->topic_name_trajectory_target );
    src->get_param( &dest->topic_name_trajectory_actual );
    src->get_param( &dest->period_publish_planning );
    src->get_param( &dest->topic_name_vehicle_status );
    src->get_param( &dest->topic_name_external_controll_velocity );
    src->get_param( &dest->service_name_set_navigation_path );
    src->get_param( &dest->vehicle_length_front );
    src->get_param( &dest->vehicle_length_back );
    src->get_param( &dest->vehicle_width_left );
    src->get_param( &dest->vehicle_width_right );
    // velocity option
    src->get_param( &dest->max_velocity );
    src->get_param( &dest->max_angular_velocity );
    if(src->get_param( &dest->max_angular_velocity ) >= 0)
    {
        dest->max_angular_velocity.value = {gnd_deg2ang(dest->max_angular_velocity.value.at(0))};
    }
    src->get_param( &dest->acceleration );
    src->get_param( &dest->deceleration );
    // navigation option
    src->get_param( &dest->pathend_margin );
    src->get_param( &dest->period_planning );
    src->get_param( &dest->period_reselect_trajectory );
    src->get_param( &dest->clearance_required );
    src->get_param( &dest->clearance_margin );
    src->get_param( &dest->discretized_interval_of_target_trajectory );
    src->get_param( &dest->distance_threshold_intermediate );
    src->get_param( &dest->depth_for_trajectory_selection );
    src->get_param( &dest->depth_for_slow_down );
    src->get_param( &dest->depth_for_stop );
    src->get_param( &dest->target_trajectory_weight_depth );
    src->get_param( &dest->target_trajecotry_weight_clearance );
    src->get_param( &dest->target_trajecotry_weight_distance );

    if(src->get_param( &dest->mediate_trajectory_search_angular_size ) >= 0)
    {
        dest->mediate_trajectory_search_angular_size.value = {gnd_deg2ang(dest->mediate_trajectory_search_angular_size.value.at(0))};
    }
    if(src->get_param( &dest->mediate_trajectory_search_angular_size_for_avoidance ) >= 0)
    {
        dest->mediate_trajectory_search_angular_size_for_avoidance.value = {gnd_deg2ang(dest->mediate_trajectory_search_angular_size_for_avoidance.value.at(0))};
    }
    if(src->get_param( &dest->mediate_trajectory_search_angular_range) >= 0 )
    {
        dest->mediate_trajectory_search_angular_range.value = {gnd_deg2ang(dest->mediate_trajectory_search_angular_range.value.at(0))};
    }
    src->get_param( &dest->mediate_trajectory_search_range );
    src->get_param( &dest->mediate_trajectory_weight_clearance );
    src->get_param( &dest->mediate_trajectory_weight_direction );



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
    dest->set_param( src->path_file );
    dest->set_param( src->start_node );
    dest->set_param( src->dest_node );
    dest->set_param( src->node_name );
    dest->set_param( src->topic_name_pose );

    dest->set_param( src->topic_name_vehicle_vel );
    // initial pose option
    dest->set_param( src->topic_name_planned_path );
    dest->set_param( src->topic_name_vehicle_ctrl );
    // particle filter parameter
    dest->set_param( src->topic_name_trajectory_target );
    dest->set_param( src->topic_name_trajectory_actual );
    dest->set_param( src->period_publish_planning );
    dest->set_param( src->topic_name_vehicle_status );
    dest->set_param( src->topic_name_external_controll_velocity );
    dest->set_param( src->service_name_set_navigation_path );
    dest->set_param( src->vehicle_length_front );
    dest->set_param( src->vehicle_length_back );
    dest->set_param( src->vehicle_width_left );
    dest->set_param( src->vehicle_width_right );
    dest->set_param( src->max_velocity );
    src->max_angular_velocity.value = {gnd_ang2deg(src->max_angular_velocity.value.at(0))};
    dest->set_param( src->max_angular_velocity );
    src->max_angular_velocity.value = {gnd_deg2ang(src->max_angular_velocity.value.at(0))};
    dest->set_param( src->acceleration );
    dest->set_param( src->deceleration );
    // navigation option
    dest->set_param( src->period_planning );
    dest->set_param( src->period_reselect_trajectory );
    dest->set_param( src->clearance_required );
    dest->set_param( src->clearance_margin );
    dest->set_param( src->pathend_margin );
    dest->set_param( src->discretized_interval_of_target_trajectory );
    dest->set_param( src->distance_threshold_intermediate );
    dest->set_param( src->depth_for_trajectory_selection );
    dest->set_param( src->depth_for_slow_down );
    dest->set_param( src->depth_for_stop );
    dest->set_param( src->target_trajectory_weight_depth );
    dest->set_param( src->target_trajecotry_weight_clearance );
    dest->set_param( src->target_trajecotry_weight_distance );
    src->mediate_trajectory_search_angular_size.value = {gnd_ang2deg(src->mediate_trajectory_search_angular_size.value.at(0))};
    dest->set_param( src->mediate_trajectory_search_angular_size );
    src->mediate_trajectory_search_angular_size.value = {gnd_deg2ang(src->mediate_trajectory_search_angular_size.value.at(0))};
    src->mediate_trajectory_search_angular_size_for_avoidance.value = {gnd_ang2deg(src->mediate_trajectory_search_angular_size_for_avoidance.value.at(0))};
    dest->set_param( src->mediate_trajectory_search_angular_size_for_avoidance );
    src->mediate_trajectory_search_angular_size_for_avoidance.value = {gnd_deg2ang(src->mediate_trajectory_search_angular_size_for_avoidance.value.at(0))};
    src->mediate_trajectory_search_angular_range.value = {gnd_ang2deg(src->mediate_trajectory_search_angular_range.value.at(0))};
    dest->set_param( src->mediate_trajectory_search_angular_range );
    src->mediate_trajectory_search_angular_range.value = {gnd_deg2ang(src->mediate_trajectory_search_angular_range.value.at(0))};
    dest->set_param( src->mediate_trajectory_search_range );
    dest->set_param( src->mediate_trajectory_weight_clearance );
    dest->set_param( src->mediate_trajectory_weight_direction );

    return 0;
}

} //End of namespace

} //End of namespace

#endif // ICARTMINI_SBTP_CONFIG_H


