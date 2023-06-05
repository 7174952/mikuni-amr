/*
 * gnd_gyrodometry_conf.hpp
 *
 *  Created on: 2014/08/01
 *      Author: tyamada
 *  Updated by ryu, 2023/5/18
 *  .Use Qt v5.14 lib to update config file
 */

#ifndef GND_GYRODOMETOR_CONF_H
#define GND_GYRODOMETOR_CONF_H

#include <QString>
#include "gnd-configfile.h"

namespace gnd {
namespace gyrodometor {

// ---> const variables definition
static const Param<QString> Default_node_name = {
    "node-name",
    {"gyrodometor"},
    "ros-node name"
};

static const Param<QString> Default_topic_name_imu = {
    "topic-name-imu",
    {"imu"},
    "imu topic name (subscribe)"
};

static const Param<QString> Default_topic_name_vel2d = {
    "topic-name-velocity",
    {"vehiclevel"},
    "2d velocity topic name (subscribe)"
};

static const Param<QString> Default_topic_name_gyrodometry = {
    "topic-name-gyrodometry",
    {"gyrodometry"},
    "gyrodometry(pose2d) topic name (publish)"
};

static const Param<double> Default_period = {
    "period",
    {0.001*10},
    "position estimation cycle [sec]"
};

static const Param<double> Default_offset_calibration_default = {
    "offset-calibration-default",
    {0.0},
    "offset calibration default"
};

static const Param<double> Default_offset_calibration_factor = {
    "offset-calibration-factor",
    {0.005},
    "offset calibration factor"
};

static const Param<double> Default_offset_calibration_threshold = {
    "offset-calibration-threshold",
    {0.5},
    "if gyro-sensor readings(rate absolute value) is greater than this, offset calibration ignore the readings"
};

static const Param<double> Default_offset_calibration_time_margin = {
    "offset-calibration-time-margin",
    {0.1},
    "offset calibration start after this time from robot stopped"
};

static const Param<double> Default_period_cui_status_display = {
    "period-cui-status-display",
    {0},
    "period to display the node status in standard output [msec]. [note] it need ansi color code. [note] if this value is less than or equal 0, don't display"
};
// ---> debug condition


static const Param<QString> Default_gyrodometry_log = {
    "gyrodometry-log",
    {""},
    "gyrodometry log(text file)"
};

/**
 * \brief configuration parameter for gnd_gyrodometry node
 */
struct node_config
{
    node_config();

    // ros communication config
    Param<QString>	node_name;							          ///< node name
    Param<QString>	topic_name_imu;						        ///< imu topic name
    Param<QString>	topic_name_vel2d;					        ///< vel2d topic name
    Param<QString>	topic_name_gyrodom;					      ///< gyrodometory topic name

    // position estimation config
    Param<double>	  offset_calibration_default;			  ///< offset calibration option
    Param<double>	  offset_calibration_time_margin;		///< offset calibration option
    Param<double>	  offset_calibration_factor;			  ///< offset calibration option
    Param<double>   offset_calibration_threshold;		  ///< offset calibration threshold

    // debug option
    Param<QString>	gyrodometry_log;					        ///< gyrodometory topic name
    Param<double>   period_cui_status_display;			  ///< cui status display mode
};

int init_node_config( node_config *p );
int fread_node_config( const QString &fname, node_config *dest );
int get_node_config( node_config *dest, ConfigFile *src );
int fwrite_node_config( const QString &fname, node_config *src );
int set_node_config( ConfigFile *dest, node_config *src );

// <--- struct node_config

}//End of namespace

}//End of namespace

namespace gnd {
namespace gyrodometor {

// ---> function definition
node_config::node_config()
{
    init_node_config(this);
}

/*
 * \brief initialize configuration parameter
 * @param [out] p : node_config
 */
int init_node_config( node_config *p )
{
    Q_ASSERT_X(p, "invalid", "invalid null pointer argument\n" );

    // ros communication
    p->node_name = Default_node_name;
    p->topic_name_imu = Default_topic_name_imu;
    p->topic_name_vel2d = Default_topic_name_vel2d;
    p->topic_name_gyrodom = Default_topic_name_gyrodometry;
    // position estimation
    p->offset_calibration_default = Default_offset_calibration_default;
    p->offset_calibration_factor = Default_offset_calibration_factor;
    p->offset_calibration_time_margin = Default_offset_calibration_time_margin;
    p->offset_calibration_threshold = Default_offset_calibration_threshold;

    // debug option
    p->gyrodometry_log = Default_gyrodometry_log;
    p->period_cui_status_display = Default_period_cui_status_display;

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

    // ros communication
    src->get_param(&dest->node_name );
    src->get_param(&dest->topic_name_imu );
    src->get_param(&dest->topic_name_vel2d );
    src->get_param(&dest->topic_name_gyrodom );

    // position estimation
    src->get_param(&dest->offset_calibration_default );
    src->get_param(&dest->offset_calibration_factor );
    src->get_param(&dest->offset_calibration_time_margin );
    src->get_param(&dest->offset_calibration_threshold );
    // debug option
    src->get_param(&dest->gyrodometry_log );
    src->get_param(&dest->period_cui_status_display );

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

    // ros communication
    dest->set_param(src->node_name);
    dest->set_param(src->topic_name_imu );
    dest->set_param(src->topic_name_vel2d );
    dest->set_param(src->topic_name_gyrodom );

    // position estimation
    dest->set_param(src->offset_calibration_default );
    dest->set_param(src->offset_calibration_factor );
    dest->set_param(src->offset_calibration_time_margin );
    dest->set_param(src->offset_calibration_threshold );

    // debug option
    dest->set_param(src->gyrodometry_log );
    dest->set_param(src->period_cui_status_display );

    return 0;
}

}//End of namespace

}//End of namespace

#endif // GND_GYRODOMETOR_CONF_H
