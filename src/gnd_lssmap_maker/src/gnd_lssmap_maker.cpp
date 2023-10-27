/*
 * gnd_lssmap_maker_config.h
 *
 *  Created on: 2014/09/05
 *      Author: tyamada
 *       Brief: Laser Scan Statistics MAP maker
 *  Updated by ryu, 2023/5/26
 *  .Use Qt v5.14 lib to generate maps
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

#include "gnd_msgs/msg_pose2d_stamped.h"
#include "gnd_lssmap_maker_config.h"
#include "gnd_rosutil.h"
#include "gnd-lssmap-base.h"
#include "gnd-matrix-coordinate.h"
#include "gnd-coord-tree.h"
#include "gnd-bmp.h"

const uint16_t MAX_SIZE = 1000;
gnd::data_buff<gnd_msgs::msg_pose2d_stamped> pose_buff(MAX_SIZE);
gnd_msgs::msg_pose2d_stamped  pose_msg_prevcollect;
gnd_msgs::msg_pose2d_stamped	pose_msg;				  // pose message reader and storage
gnd::coord_tree 		          coordinate_tree;		// coordinate tree
gnd::cmap_t					          lssmap_counting;		// counting map of laser scan statistics


double collect_condition_time;
double collect_condition_moving_distance;
double collect_condition_moving_angle;
int coordid_gl = -1;
int coordid_map = -1;
int coordid_rbt = -1 ;
double collect_condition_ignore_range_lower;
double collect_condition_ignore_range_upper;
double collect_condition_culling_distance;


// debug file stream
QFile fp_txtlog_pointcloud;
QFile fp_txtlog_trajectory;
QTextStream out_pc;
QTextStream out_tj;

double time_watch_dog;
bool is_init_on;

void pose_Callback(const gnd_msgs::msg_pose2d_stamped::ConstPtr& msg)
{
    pose_buff.push(*msg);
}

void pointcloud_Callback(const sensor_msgs::PointCloud::ConstPtr& msg_pointcloud)
{
    //update time
    time_watch_dog = ros::Time::now().toSec();
    is_init_on = false;

    if(pose_buff.copy_at_time(&pose_msg, msg_pointcloud->header.stamp.toSec()) < 0)
    {
        return; //get pose data error
    }

    bool flg_collect = false;
    double time = pose_msg.header.stamp.toSec() - pose_msg_prevcollect.header.stamp.toSec();
    double sqdist = (pose_msg.x - pose_msg_prevcollect.x) * (pose_msg.x - pose_msg_prevcollect.x)
                  + (pose_msg.y - pose_msg_prevcollect.y) * (pose_msg.y - pose_msg_prevcollect.y);
    double angle = qFabs( gnd::rad_normalize(pose_msg.theta - pose_msg_prevcollect.theta ) );

    // check data collect condition
    flg_collect = flg_collect
        || (    (collect_condition_time > 0)
             && (time >= collect_condition_time));
    flg_collect = flg_collect
        || (    (collect_condition_moving_distance > 0)
             && (sqdist > collect_condition_moving_distance * collect_condition_moving_distance));
    flg_collect = flg_collect
        || (    (collect_condition_moving_angle > 0)
             && (angle > collect_condition_moving_angle));
    flg_collect = flg_collect
             && ( fabs(pose_msg.header.stamp.toSec() - msg_pointcloud->header.stamp.toSec()) < 0.1 );

    if(!flg_collect)
    {
        return; //Not in meeting condition case
    }

    QGenericMatrix<4,4,double> mat_coordtf_rbt2gl;
    QGenericMatrix<4,4,double> mat_coordtf_rbt2map;
    double x_src_prev = 10000;
    double y_src_prev = 10000;

    // calculate coordinate transform matrix
    gnd::matrix::coordinate_converter(&mat_coordtf_rbt2gl,
        pose_msg.x, pose_msg.y, 0,
        cos(pose_msg.theta), sin(pose_msg.theta), 0,
        0, 0, 1.0);
    coordinate_tree.set_coordinate(coordid_rbt, &mat_coordtf_rbt2gl);
    coordinate_tree.get_convert_matrix(coordid_rbt, coordid_map, &mat_coordtf_rbt2map);

    // ---> scanning loop (point cloud data)
    for( uint i = 0; i < (int)msg_pointcloud->points.size(); i++ )
    {
        double sq_dist;
        QGenericMatrix<4,1,double> point_src, point_gl, point_map;

        // ignore
        sq_dist = ( msg_pointcloud->points[i].x) * ( msg_pointcloud->points[i].x) + ( msg_pointcloud->points[i].y ) * ( msg_pointcloud->points[i].y );
        if( (collect_condition_ignore_range_lower >= 0) && (sq_dist < collect_condition_ignore_range_lower * collect_condition_ignore_range_lower))
        {
            continue;
        }
        if( (collect_condition_ignore_range_upper >= 0) && (sq_dist > collect_condition_ignore_range_upper * collect_condition_ignore_range_upper))
        {
            continue;
        }

        // culling
        sq_dist =  ( msg_pointcloud->points[i].x - x_src_prev ) * ( msg_pointcloud->points[i].x - x_src_prev )
                 + ( msg_pointcloud->points[i].y - y_src_prev ) * ( msg_pointcloud->points[i].y - y_src_prev );
        if( sq_dist < collect_condition_culling_distance * collect_condition_culling_distance )
        {
            continue;
        }
        x_src_prev = msg_pointcloud->points[i].x;
        y_src_prev = msg_pointcloud->points[i].y;

        // coordinate transform
        point_src(0,0) = msg_pointcloud->points[i].x;
        point_src(0,1) = msg_pointcloud->points[i].y;
        point_src(0,2) = msg_pointcloud->points[i].z;
        point_src(0,3) = 1;

        point_gl = point_src * mat_coordtf_rbt2gl;
        point_map = point_src * mat_coordtf_rbt2map;

        // counting
        gnd::counting_map(&lssmap_counting, point_map(0,0), point_map(0,1));

        // logging
        if( fp_txtlog_pointcloud.isOpen() )
        {
            out_pc << QString::asprintf("%d %lf %lf %lf %lf \r\n",
                                        msg_pointcloud->header.seq,
                                        point_map(0,0),
                                        point_map(0,1),
                                        point_gl(0,0),
                                        point_gl(0,1));
        }
        if( fp_txtlog_trajectory.isOpen() )
        {
            point_src(0,0) = 0;
            point_src(0,1) = 0;
            point_src(0,2) = 0;
            point_src(0,3) = 1;

            point_gl = point_src * mat_coordtf_rbt2gl;
            point_map = point_src * mat_coordtf_rbt2map;

            out_tj << QString::asprintf("%d %lf %lf %lf %lf \r\n",
                                        pose_msg.header.seq,
                                        point_map(0,0),
                                        point_map(0,1),
                                        point_gl(0,0),
                                        point_gl(0,1));
        }

    }

    pose_msg_prevcollect = pose_msg;

}

int main(int argc, char **argv)
{
    gnd::lssmap_maker::node_config node_config;

    // start up, read configuration file
    if( argc > 1 )
    {
        if( gnd::lssmap_maker::fread_node_config( argv[1], &node_config ) < 0 )
        {
            ROS_ERROR("   ... Error: fail to read config file %s", argv[1]);
            QString fname = QString(argv[1]) + ".tmp";
            // file out configuration file
            if( gnd::lssmap_maker::fwrite_node_config( fname, &node_config ) >= 0 )
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

    // ---> variables
    ros::NodeHandle			           nh;					      // ros nodehandle
    ros::Subscriber			           pointcloud_sub;		// point-cloud subscriber

    ros::Subscriber			           pose_sub;				  // pose subscriber

    int phase = 0;
    ROS_INFO("---------- initialize ----------");

    // show initialize phase task
    if( ros::ok() )
    {
        ROS_INFO(" initialization task");
        ROS_INFO("   %d. initialize global pose topic subscriber\n", ++phase);
        ROS_INFO("   %d. initialize point-cloud topic subscriber\n", ++phase);
        ROS_INFO("   %d. initialize map for laser scan data counting\n", ++phase);
        ROS_INFO("   %d. defin coordinate\n", ++phase);

        if ( node_config.text_log_pointcloud.value.at(0).size() > 0 )
        {
            ROS_INFO("   %d. create pointcloud log file\n", ++phase);
        }
        if ( node_config.text_log_trajectory.value.at(0).size() > 0)
        {
            ROS_INFO("  %d. create trajecotry log file\n", ++phase);
        }
    }


    // ---> initialize robot pose subscriber
    if( ros::ok() )
    {
        ROS_INFO(" => initialize robot pose topic subscriber");
        if( node_config.topic_name_pose.value.at(0).isEmpty() )
        {
            ROS_ERROR("    ... error: laser scan topic name is null");
            ROS_ERROR("        usage: fill \"%s\" item in configuration file\n", node_config.topic_name_pose.item.toStdString().c_str());
            ros::shutdown();
        }
        else
        {
            ROS_INFO("    ... topic name is \"%s\"\n", node_config.topic_name_pose.value.at(0).toStdString().c_str());
            // make subscriber
            pose_sub = nh.subscribe(node_config.topic_name_pose.value.at(0).toStdString(), 1000, pose_Callback);
            ROS_INFO("    ... ok");
        }
    }

    // initialize point-cloud subscriber
    if( ros::ok() )
    {
        ROS_INFO(" => initialize point-cloud topic subscriber");

        if( node_config.topic_name_pointcloud.value.at(0).isEmpty() )
        {
            ROS_ERROR("    ... error: laser scan topic name is null");
            ROS_ERROR("        usage: fill \"%s\" item in configuration file\n", node_config.topic_name_pointcloud.item.toStdString().c_str());
            ros::shutdown();
        }
        else
        {
            ROS_INFO("    ... topic name is \"%s\"\n", node_config.topic_name_pointcloud.value.at(0).toStdString().c_str());
            // make subscriber
            pointcloud_sub = nh.subscribe(node_config.topic_name_pointcloud.value.at(0).toStdString(), 200, pointcloud_Callback);
            ROS_INFO("    ... ok");
        }
    }

    // initialize counting map
    if ( ros::ok() )
    {
        ROS_INFO("   => initialize counting map" );

        if( node_config.initial_counting_map.value.at(0).size() > 0)
        {
            // load counting map
            if( gnd::read_counting_map(&lssmap_counting, node_config.initial_counting_map.value.at(0)) < 0 )
            {
                ros::shutdown();
                ROS_ERROR("    ... error: fail to load counting map in \"%s\" ", node_config.initial_counting_map.value.at(0).toStdString().c_str());
            }
            else
            {
                ROS_INFO("    ... ok: load counting map in \"%s\" ", node_config.initial_counting_map.value.at(0).toStdString().c_str());
            }
        }
        else
        {
            // initialize counting map
            if( gnd::init_counting_map(&lssmap_counting, node_config.counting_map_cell_size.value.at(0), node_config.counting_map_cell_size.value.at(0)) < 0 )
            {
                ros::shutdown();
                ROS_ERROR("    ... error: fail to create map");
            }
            else
            {
                ROS_INFO("    ... ok");
            }
        }
    }

    // define coordinate
    if( ros::ok() )
    {
        QGenericMatrix<4,4,double> coordinate_matrix;

        ROS_INFO("   => define coordinate");

        // initialize coordinate matrix
        coordinate_matrix.setToIdentity();

        // set global coordinate
        coordid_gl = coordinate_tree.add_node("global", "root", &coordinate_matrix);
        // set robot coordinate
        coordid_rbt = coordinate_tree.add_node("robot","global", &coordinate_matrix);

        // set map coordinate
        gnd::matrix::coordinate_converter(&coordinate_matrix,
            node_config.map_origin.value.at(0), node_config.map_origin.value.at(1), node_config.map_origin.value.at(2),
            node_config.map_x_axis.value.at(0), node_config.map_x_axis.value.at(1), 0.0,
            0.0, 0.0, 1.0
        );
        coordid_map = coordinate_tree.add_node("map", "global", &coordinate_matrix);
    }

    // text log file create
    if ( ros::ok() && node_config.text_log_pointcloud.value.at(0).size() > 0 )
    {
        QString fname(node_config.map_folder_path.value.at(0) + "/" + node_config.text_log_pointcloud.value.at(0));

        ROS_INFO("   => create log file \"%s\"\n", fname.toStdString().c_str());
        fp_txtlog_pointcloud.setFileName(fname);
        if(fp_txtlog_pointcloud.open(QIODevice::WriteOnly  | QIODevice::Text))
        {
            out_pc.setDevice(&fp_txtlog_pointcloud);
            out_pc << "#[1. sequence id] [2. x on map] [3. y on map] [4. x on default] [5. y on default]\n";
            ROS_INFO("   ...ok!");
        }
        else
        {
            ROS_ERROR("    ... error: fail to open \"%s\"", fname.toStdString().c_str());
            ros::shutdown();
        }
    }

    // text log file create
    if ( ros::ok() && node_config.text_log_trajectory.value.at(0).size() > 0 )
    {
        QString fname(node_config.map_folder_path.value.at(0) + "/" + node_config.text_log_trajectory.value.at(0));

        ROS_INFO("   => create log file \"%s\"\n", fname.toStdString().c_str());

        fp_txtlog_trajectory.setFileName(fname);
        if(fp_txtlog_trajectory.open(QIODevice::WriteOnly  | QIODevice::Text))
        {
            out_tj.setDevice(&fp_txtlog_trajectory);
            out_tj << "#[1. sequence id] [2. x on map] [3. y on map] [4. x on default] [5. y on default]\n";
            ROS_INFO("   ...ok!");
        }
        else
        {
            ROS_ERROR("    ... error: fail to open \"%s\"", fname.toStdString().c_str());
            ros::shutdown();
        }

    }

    // ---> previous pose
    collect_condition_time            = node_config.collect_condition_time.value.at(0);
    collect_condition_moving_distance = node_config.collect_condition_moving_distance.value.at(0);
    collect_condition_moving_angle    = node_config.collect_condition_moving_angle.value.at(0);
    collect_condition_ignore_range_lower = node_config.collect_condition_ignore_range_lower.value.at(0);
    collect_condition_ignore_range_upper = node_config.collect_condition_ignore_range_upper.value.at(0);
    collect_condition_culling_distance   = node_config.collect_condition_culling_distance.value.at(0);
    pose_msg_prevcollect.x            = qSqrt(DBL_MAX) / 2;
    pose_msg_prevcollect.y            = qSqrt(DBL_MAX) / 2;
    pose_msg_prevcollect.header.seq   = 0;
    pose_msg_prevcollect.header.stamp.fromSec( ros::Time::now().toSec() - node_config.collect_condition_time.value.at(0) );

    //wait to complete
    is_init_on = true;
    time_watch_dog = ros::Time::now().toSec();
    // operate
    ros::Rate loop_rate(1000);
    while( ros::ok() && (is_init_on || (ros::Time::now().toSec() - time_watch_dog) < 1.0) )
    {
        // blocking
        loop_rate.sleep();
        ros::spinOnce();
    }

    // ---> finalize
    {
      if( fp_txtlog_pointcloud.isOpen() )
      {
          fp_txtlog_pointcloud.close();
      }
      if( fp_txtlog_trajectory.isOpen() )
      {
          fp_txtlog_trajectory.close();
      }

      //counting data file out
      gnd::write_counting_map(&lssmap_counting, node_config.map_folder_path.value.at(0)+ "/");
      // build bmp image (to visualize for human)
      gnd::lssmap_t lssmap;
      gnd::bmp8_t bmp;
      gnd::bmp32_t bmp32;
      QString bmp_name;
      QString bmp32_name;

      bmp_name = node_config.map_folder_path.value.at(0) + "/" + "map-image8.bmp";
      bmp32_name = node_config.map_folder_path.value.at(0) + "/" + "map-image32.bmp";

      // build environmental map
      gnd::build_map(&lssmap, &lssmap_counting, node_config.sensor_range.value.at(0), node_config.additional_smoothing_parameter.value.at(0) );
      // make bmp image: it show the likelihood field
      gnd::build_bmp(&bmp, &lssmap, node_config.image_map_pixel_size.value.at(0));
      // make bmp image: it show the likelihood field
      gnd::build_bmp(&bmp32, &lssmap, node_config.image_map_pixel_size.value.at(0));
      // file out
      gnd::write(bmp_name, &bmp);
      // file out
      gnd::write(bmp32_name, &bmp32);
      gnd::destroy_map(&lssmap);
      gnd::destroy_counting_map(&lssmap_counting);

      // set origin to file
      QString fname(node_config.map_folder_path.value.at(0) + "/" + "origin.txt");
      QFile file(fname);
      double x, y;
      QTextStream out_org;

      if(!file.open(QIODevice::WriteOnly  | QIODevice::Text))
      {
          ROS_INFO("fail to open %s",fname.toStdString().c_str());
      }
      else
      {
          out_org.setDevice(&file);
          bmp.pget_origin(&x, &y);
          out_org << QString::asprintf("%lf %lf \n",x,y);
          file.close();
      }

      ROS_INFO("   ... make map image %s finished!\n", "map-image.bmp");
      bmp.deallocate();
      bmp32.deallocate();

    } // <--- finalize

    return 0;
}

