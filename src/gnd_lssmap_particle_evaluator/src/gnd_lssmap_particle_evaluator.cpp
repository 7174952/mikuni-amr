/*
 * gnd_lssmap_particle_evaluator.cpp
 *
 *  Created on: 2014/09/05
 *      Author: tyamada
 *       Brief: Calculate particles evaluate and compared on map
 *  Updated by ryu, 2023/9/19
 *  .Use Qt v5.14 lib to update self location compared on map
 */

#include <QString>
#include <QFile>
#include <QTextStream>
#include <QGenericMatrix>
#include <QtMath>

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/rate.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud.h"
#include "float.h"

#include "gnd_particle_localizer/msg_particles_pose2d_stamped.h"
#include "gnd_particle_localizer/msg_particle_weights_stamped.h"
#include "gnd_lssmap_particle_evaluator_config.h"
#include "gnd-bmp.h"
#include "gnd-random.h"

struct
{
    double height_range_lower;
    double height_range_upper;
    double square_hrange_lower;
    double square_hrange_upper;
    double square_range_culling;
} condition_point_select;

const uint16_t MAX_SIZE = 1000;
gnd::data_buff<gnd_particle_localizer::msg_particles_pose2d_stamped>             particle_buff(MAX_SIZE);
gnd_particle_localizer::msg_particle_weights_stamped::_weights_type::value_type  log_likelihood_max;
gnd_particle_localizer::msg_particles_pose2d_stamped	                           msg_particle;
gnd_particle_localizer::msg_particle_weights_stamped                             msg_particle_weights;

gnd::lssmap_particle_evaluator::node_config node_config;
ros::Publisher particle_weight_pub;
gnd::bmp32_t map;
double value_out_of_map;


void particle_Callback(const gnd_particle_localizer::msg_particles_pose2d_stamped::ConstPtr& msg)
{
    particle_buff.push(*msg);
}

void pointcloud_Callback(const sensor_msgs::PointCloud::ConstPtr& msg_pointcloud)
{
    if(particle_buff.copy_at_time(&msg_particle, msg_pointcloud->header.stamp.toSec()) < 0)
    {
        return; //get particle data error
    }
    else if(msg_particle.poses.size() == 0)
    {
        return; //particles datais invalid (no particle)
    }
    else // ---> evaluate particles weights
    {
        log_likelihood_max = -FLT_MAX;

        // resize particle weights
        msg_particle_weights.weights.resize(msg_particle.poses.size());

        // ---> particles scanning loop ( calculate log likelihood  )
        for(uint i = 0; i < msg_particle.poses.size(); i++)
        {
            double cosv, sinv;
            struct
            {
                double x;
                double y;
            } prev_point;

            // initialize
            msg_particle_weights.weights[i] = 0;
            cosv = qCos(msg_particle.poses[i].theta);
            sinv = qSin(msg_particle.poses[i].theta);

            // initialize ( large value as it is impossible )
            prev_point.x = 10000;
            prev_point.y = 10000;

            // ---> point-cloud scanning loop ( calculate log likelihood  )
            for( uint j = 0; j < msg_pointcloud->points.size(); j++)
            {
                double x_gl, y_gl;
                double square_range;

                // point select
                // check height range
                if((node_config.points_select_condition_height_range.value.at(0) < node_config.points_select_condition_height_range.value.at(1))
                   && (   (msg_pointcloud->points[j].z < condition_point_select.height_range_lower)
                       || (msg_pointcloud->points[j].z > condition_point_select.height_range_upper)))
                {
                    // ignore
                    continue;
                }

                // check horizontal range
                square_range = gnd_square( msg_pointcloud->points[j].x ) + gnd_square( msg_pointcloud->points[j].y );
                if( condition_point_select.square_hrange_lower >= 0 	// if lower horizontal range is configured
                    && square_range < condition_point_select.square_hrange_lower)
                {
                    continue;
                }
                else if( square_range < condition_point_select.square_hrange_upper)
                {
                    continue;
                }

                // check distance from contiguous points
                square_range = gnd_square( msg_pointcloud->points[j].x - prev_point.x ) + gnd_square( msg_pointcloud->points[j].y - prev_point.y );
                if( square_range < condition_point_select.square_range_culling )
                {
                    continue;
                }

                // update previous selected point
                prev_point.x = msg_pointcloud->points[j].x;
                prev_point.y = msg_pointcloud->points[j].y;

                // ---> coordinate transformation
                x_gl = msg_pointcloud->points[j].x * cosv - msg_pointcloud->points[j].y * sinv + msg_particle.poses[i].x;
                y_gl = msg_pointcloud->points[j].x * sinv + msg_pointcloud->points[j].y * cosv + msg_particle.poses[i].y;

                // ---> refer map to get likelihood and calculate the particle log likelihood
                if( !map.ppointer(x_gl, y_gl) )
                {
                    // out of map range
                    // deal as minimum value except zero
                    msg_particle_weights.weights[i] += qLn( (float) value_out_of_map + 1.0 );
                }
                else
                {
                    msg_particle_weights.weights[i] += qLn( (float) map.pvalue(x_gl, y_gl) + 1.0 );
                }
            }

            // save maximum log likelihood
            log_likelihood_max = msg_particle_weights.weights[i] > log_likelihood_max ? msg_particle_weights.weights[i] : log_likelihood_max;
        }// <--- particles scanning loop ( calculate log likelihood  )

        for(uint i = 0; i < msg_particle_weights.weights.size(); i++)
        {
            msg_particle_weights.weights[i] = qExp(msg_particle_weights.weights[i] - log_likelihood_max) + FLT_EPSILON;
        }

        // set evaluated particles sequence id
        msg_particle_weights.seq_particles = msg_particle.header.seq;

        // set header
        msg_particle_weights.header.seq++;
        msg_particle_weights.header.stamp = msg_particle.header.stamp;

        // publish
        particle_weight_pub.publish( msg_particle_weights );

    }

}

int main(int argc, char **argv)
{
    // start up, read configuration file
    if( argc > 1 )
    {
        if( gnd::lssmap_particle_evaluator::fread_node_config( argv[1], &node_config ) < 0 )
        {
            ROS_ERROR("   ... Error: fail to read config file %s", argv[1]);
            QString fname = QString(argv[1]) + ".tmp";
            // file out configuration file
            if( gnd::lssmap_particle_evaluator::fwrite_node_config( fname, &node_config ) >= 0 )
            {
              ROS_INFO(" : output sample config file %s", fname.toStdString().c_str());
            }
            return -1;
        }
        else
        {
            ROS_INFO("   ... read config file %s", argv[1]);
        }
    }

    // initialize ros
    if( node_config.node_name.value.at(0).size() > 0 )
    {
        ros::init(argc, argv, node_config.node_name.value.at(0).toStdString());
    }
    else
    {
        ROS_ERROR("   ... Error: node name is null, you must specify the name of this node via config item \"%s\"\n",
                  node_config.node_name.item.toStdString().c_str());
        return -1;
    }
    ROS_INFO(" node: \"%s\"\n", node_config.node_name.value.at(0).toStdString().c_str());

    { // ---> initialize point select condition parameter
        condition_point_select.height_range_lower = node_config.points_select_condition_height_range.value.at(0);
        condition_point_select.height_range_upper = node_config.points_select_condition_height_range.value.at(1);

        if( node_config.points_select_condition_ignore_horizontal_range_lower.value.at(0) < 0 )
        {
            condition_point_select.square_hrange_lower = -1;
        }
        else
        {
            condition_point_select.square_hrange_lower = gnd_square(node_config.points_select_condition_ignore_horizontal_range_lower.value.at(0));
        }

        if( node_config.points_select_condition_ignore_horizontal_range_upper.value.at(0) < 0 )
        {
            condition_point_select.square_hrange_upper = -1;
        }
        else
        {
            condition_point_select.square_hrange_upper = gnd_square(node_config.points_select_condition_ignore_horizontal_range_upper.value.at(0));
        }

        if( node_config.points_select_condition_culling_distance.value.at(0) < 0)
        {
            condition_point_select.square_range_culling = -1;
        }
        else
        {
            condition_point_select.square_range_culling = gnd_square(node_config.points_select_condition_culling_distance.value.at(0));
        }
    } // <--- initialize point select condition parameter

    // ---> variables
    ros::NodeHandle			           nh;					           // ros nodehandle
    ros::Subscriber			           pointcloud_sub;		     // point-cloud subscriber
    ros::Subscriber			           particle_sub;		       // particle subscriber

    int phase = 0;
    ROS_INFO("---------- initialization ----------");

    // show initialize phase task
    if( ros::ok() )
    {
        ROS_INFO(" => initialization task");
        ROS_INFO("   %d. load map", ++phase);
        ROS_INFO("   %d. make particles subscriber", ++phase);
        ROS_INFO("   %d. make point-cloud subscriber", ++phase);
        ROS_INFO("   %d. make particle weights publisher", ++phase);
    }

    phase = 0;
    // ---> load map
    if(ros::ok())
    {
        int16_t ret;
        QFile qFile(node_config.bmp_map_origin.value.at(0));

        ROS_INFO(" => %d. load map", ++phase);
        ROS_INFO("    ... file path \"%s\"",node_config.bmp_map.value.at(0).toStdString().c_str());
        ROS_INFO("        origin file path \"%s\"",node_config.bmp_map_origin.value.at(0).toStdString().c_str());

        if(node_config.bmp_map.value.at(0).isEmpty())
        {
            ROS_ERROR("    ... error: bmp_map file path is null");
            ROS_ERROR("        usage: file \"%s\" item in configuration file", node_config.bmp_map.item.toStdString().c_str());
            ros::shutdown();
        }
        else if((ret = gnd::read(node_config.bmp_map.value.at(0),&map)) < 0)
        {
            ROS_ERROR("      ... error: fail to read bmp_map file \"%s\"",node_config.bmp_map.value.at(0).toStdString().c_str());
            ros::shutdown();
        }
        else if(!qFile.open(QFile::ReadOnly|QFile::Text))
        {
            ROS_ERROR("     ... error: fail to read bmp_map origin file \"%s\"",node_config.bmp_map_origin.value.at(0).toStdString().c_str());
            ros::shutdown();
        }
        else
        {
            double x = 0.0, y = 0.0;
            QTextStream in_file(&qFile);

            QStringList strList = in_file.readLine().trimmed().split(" ");
            if(strList.size() < 2)
            {
                ROS_ERROR("   ... error: fail to read bmp_map origin file \"%s\"",node_config.bmp_map_origin.value.at(0).toStdString().c_str());
                ros::shutdown();
            }
            else
            {
                x = strList.at(0).toDouble();
                y = strList.at(1).toDouble();

                map.pset_origin(x, y);
                gnd::write("./load.bmp",&map);

                // set out of map value
                value_out_of_map = 0;
                for(uint i = 0; i < 100; i++)
                {
                    int j;
                    //sampling from end of map
                    j = (map.row() * (1.0 - DBL_EPSILON)) * gnd::random_uniform();
                    value_out_of_map += *map.pointer(j,0);

                    j = (map.row() * (1.0 - DBL_EPSILON)) * gnd::random_uniform();
                    value_out_of_map += *map.pointer(j, map.column() - 1);

                    j = (map.column() * (1.0 - DBL_EPSILON)) * gnd::random_uniform();
                    value_out_of_map += *map.pointer(0, j);

                    j = (map.column() * (1.0 - DBL_EPSILON)) * gnd::random_uniform();
                    value_out_of_map += *map.pointer(map.row() - 1, j);

                }
                value_out_of_map /= (4 * 100);
                ROS_INFO("   ... ok, output \"load.bmp\" to confirm the load map");

            }
            qFile.close();
        }
    } // <--- load map

    // ---> make particles subscriber
    if( ros::ok() )
    {
        ROS_INFO("   => %d. make particles subscriber", ++phase);
        if( node_config.topic_name_particles.value.at(0).isEmpty() )
        {
            ROS_ERROR("    ... error: particles topic name is null");
            ROS_ERROR("        usage: fill \"%s\" item in configuration file", node_config.topic_name_particles.item.toStdString().c_str());
            ros::shutdown();
        }
        else
        {
            ROS_INFO("    ... topic name is \"%s\"\n", node_config.topic_name_particles.value.at(0).toStdString().c_str());
            // make subscriber
            particle_sub = nh.subscribe(node_config.topic_name_particles.value.at(0).toStdString(), 400, particle_Callback);
            ROS_INFO("    ... ok");
        }
    } // <--- make particles subscriber

    // ---> make point-cloud subscriber
    if( ros::ok() )
    {
        ROS_INFO("    => %d. make point-cloud subscriber", ++phase);

        if( node_config.topic_name_pointcloud.value.at(0).isEmpty() )
        {
            ROS_ERROR("    ... error: point-cloud topic name is null");
            ROS_ERROR("        usage: fill \"%s\" item in configuration file", node_config.topic_name_pointcloud.item.toStdString().c_str());
            ros::shutdown();
        }
        else
        {
            ROS_INFO("    ... topic name is \"%s\"\n", node_config.topic_name_pointcloud.value.at(0).toStdString().c_str());
            // make subscriber
            pointcloud_sub = nh.subscribe(node_config.topic_name_pointcloud.value.at(0).toStdString(), 100, pointcloud_Callback);
            ROS_INFO("    ... ok");
        }
    } // <--- make point-cloud subscriber

    // ---> make particle weights publisher
    if( ros::ok() )
    {
        ROS_INFO("   => %d. make particle weights publisher", ++phase);

        if(node_config.topic_name_particle_weight.value.at(0).isEmpty() )
        {
            ROS_ERROR("    ... error: laser scan topic name is null");
            ROS_ERROR("        usage: fill \"%s\" item in configuration file", node_config.topic_name_particle_weight.item.toStdString().c_str() );
            ros::shutdown();
        }
        else
        {
            ROS_INFO("    ... topic name is \"%s\"", node_config.topic_name_particle_weight.value.at(0).toStdString().c_str() );
            particle_weight_pub = nh.advertise<gnd_particle_localizer::msg_particle_weights_stamped>(node_config.topic_name_particle_weight.value.at(0).toStdString(), 1000 );
            ROS_INFO("    ... ok");
        }
    } // <--- make particle weights publisher

    // --->main loop
    ros::Rate loop_rate(1000);
    while(ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    { // ---> finalize
      ROS_INFO("----------finalize----------");

      ROS_INFO("   ... end");
    } // <--- finalize

    return 0;
}

