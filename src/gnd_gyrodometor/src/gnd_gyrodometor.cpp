/*
 * gnd_gyrodometry_conf.hpp
 *
 *  Created on: 2014/08/01
 *      Author: tyamada
 *  Updated by ryu, 2023/5/18
 *  .Use Qt v5.14 lib to update config file
 */
#include <QVector>
#include <QString>
#include <QFile>
#include <QTextStream>
#include <QtMath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "gnd_msgs/msg_velocity2d_with_covariance_stamped.h"
#include "gnd_msgs/msg_pose2d_stamped.h"
#include "gnd_rosutil.h"

#include "gnd_gyrodometor_conf.h"

const uint16_t MAX_SIZE = 1000;
double offset_rate;
double time_start;
gnd::data_buff<sensor_msgs::Imu> imu_buff(MAX_SIZE);
gnd_msgs::msg_pose2d_stamped gyrodo_msg;
ros::Publisher gyrodo_pub;

QFile log_file;
QTextStream log_out;

void vel2d_Callback(const gnd_msgs::msg_velocity2d_with_covariance_stamped::ConstPtr& vel_msg)
{
    sensor_msgs::Imu imu_msg;

    if(imu_buff.copy_at_time(&imu_msg, vel_msg->header.stamp.toSec()) < 0)
    {
        return;
    }

    double cosv, sinv;
    double rate;

    cosv = qCos( gyrodo_msg.theta );
    sinv = qSin( gyrodo_msg.theta );
    rate = imu_msg.angular_velocity.z - offset_rate;

    // calculate robot direction
    gyrodo_msg.header.seq++;
    if( qFabs(gyrodo_msg.header.stamp.toSec() - vel_msg->header.stamp.toSec() ) > vel_msg->measuring_period )
    {
        gyrodo_msg.header.stamp = vel_msg->header.stamp;
    }
    else
    {
        gyrodo_msg.header.stamp.fromSec( gyrodo_msg.header.stamp.toSec() + vel_msg->measuring_period );
    }

    gyrodo_msg.x += (vel_msg->vel_x * cosv - vel_msg->vel_y * sinv) * vel_msg->measuring_period;
    gyrodo_msg.y += (vel_msg->vel_x * sinv + vel_msg->vel_y * cosv) * vel_msg->measuring_period;
    gyrodo_msg.theta += rate * vel_msg->measuring_period;

    // publish
    gyrodo_pub.publish(gyrodo_msg);

    if( log_file.isOpen() )
    {
          log_out << QString::asprintf("%lf %lf %lf %lf %lf \r\n",
                                       ros::Time::now().toSec() - time_start,
                                       gyrodo_msg.x,
                                       gyrodo_msg.y,
                                       gyrodo_msg.theta,
                                       offset_rate);
    }

}

void gyro_Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_buff.push(*msg);
}

int main(int argc, char **argv)
{
    gnd::gyrodometor::node_config			node_config;
    // read configuration
    if( argc > 1 )
    {
        ROS_INFO("=> read configuration file");
        if( gnd::gyrodometor::fread_node_config( QString(argv[1]), &node_config ) < 0 )
        {
            ROS_ERROR("... Error: fail to read configuration file %s", argv[1]);
            QString fname = QString(argv[1]) + ".tmp";
            // file out configuration file
            if( gnd::gyrodometor::fwrite_node_config( fname, &node_config ) >= 0 )
            {
                ROS_INFO("     : output sample configuration file %s", fname.toStdString().c_str());
            }
            return -1;
        }
        else
        {
            ROS_INFO("   ... ok, read config file %s", argv[1]);
        }
    }

    // init ros
    ROS_INFO(" => call ros::init()");
    if( node_config.node_name.value.at(0).size() > 0 )
    {
        ros::init(argc, argv, node_config.node_name.value.at(0).toStdString());
    }
    else
    {
        std::cout << "   ... Error: node name is null, you must specify the name of this node via config item" << node_config.node_name.item.toStdString() << "\n";
        return -1;
    }
    ROS_INFO("    ... ok, node name is %s ",node_config.node_name.value.at(0).toStdString().c_str());

    ros::NodeHandle   nh;
    ros::Subscriber		vel2d_sub;			  // velocity by gyrodometry publisher
    ros::Subscriber		imu_sub;				  // imu subscriber

    offset_rate = node_config.offset_calibration_default.value.at(0);
    time_start = ros::Time::now().toSec();

    // ---> initialize
    if( ros::ok() )
    {
        int phase = 0;

        ROS_INFO("---------- initialize ----------");

        // show initialize phase task
        if( ros::ok() )
        {
            ROS_INFO(" initialization task");
            ROS_INFO("   %d. make gyrodometry publisher",++phase);
            ROS_INFO("   %d. make vel2d subscriber",++phase);
            ROS_INFO("   %d. make imu subscriber",++phase);

            if( node_config.gyrodometry_log.value.size() > 0 )
            {
                ROS_INFO("   %d. create text log file",++phase);
            }
        }
        phase = 0;

        // initialize gyrodometry publisher
        if( ros::ok() )
        {
            ROS_INFO(" => %d. make gyrodometry publisher",++phase);
            if( node_config.topic_name_gyrodom.value.at(0).size() == 0 )
            {
                ROS_ERROR("    ... Error: gyrodometry topic name is null, you needs to specify the name via config item \"%s\"\n",
                         node_config.topic_name_gyrodom.item.toStdString().c_str());
                ros::shutdown();
            }
            else
            {
                gyrodo_pub = nh.advertise<gnd_msgs::msg_pose2d_stamped>(node_config.topic_name_gyrodom.value.at(0).toStdString(), 1000);
                gyrodo_msg.x = 0;
                gyrodo_msg.y = 0;
                gyrodo_msg.theta = 0;
                gyrodo_msg.header.stamp = ros::Time::now();
                gyrodo_msg.header.seq = 0;
                gyrodo_msg.header.frame_id = "";
                ROS_INFO("   ...ok");
            }
        }

        // initialize vel2d subscriber
        if( ros::ok() )
        {
            ROS_INFO(" => %d. make velocity subscriber",++phase);
            if( node_config.topic_name_vel2d.value.at(0).size() == 0 )
            {
                ROS_ERROR("   ... Error: velocity topic name is null, you needs to specify the name via config item \"%s\"",
                         node_config.topic_name_vel2d.item.toStdString().c_str());
                ros::shutdown();
            }
            else
            {
                // subscribe
                vel2d_sub = nh.subscribe(node_config.topic_name_vel2d.value.at(0).toStdString(), 1000, vel2d_Callback);
                ROS_INFO("   ...ok");
            }
        }

        // initialize imu subscriber
        if( ros::ok() )
        {
            ROS_INFO(" => %d. make imu subscriber",++phase);
            if( node_config.topic_name_vel2d.value.at(0).size() == 0 )
            {
                ROS_ERROR("   ... Error: imu topic name is null, you needs to specify the name via config item \"%s\"",
                          node_config.topic_name_imu.item.toStdString().c_str());
                ros::shutdown();
            }
            else
            {
                // subscribe
                imu_sub = nh.subscribe(node_config.topic_name_imu.value.at(0).toStdString(), 1000, gyro_Callback);
                ROS_INFO("   ...ok");
            }
        }

        if( ros::ok())
        {
            ROS_INFO(" => %d. make gyrodometor status publisher", ++phase);

        }

        if( ros::ok() && (node_config.gyrodometry_log.value.at(0).size() > 0) )
        {
            ROS_INFO("=> %d. create text log file", ++phase);
            ROS_INFO("    ... try to open \"%s\"", node_config.gyrodometry_log.value.at(0).toStdString().c_str());
            log_file.setFileName(node_config.gyrodometry_log.value.at(0));

            if(log_file.open(QIODevice::WriteOnly  | QIODevice::Text))
            {
                log_out.setDevice(&log_file);
                log_out << "# [1. time] [2. x] [3. y] [4. theta] [5. offset]\n";
                ROS_INFO("   ...ok!");
            }
            else
            {
                ROS_ERROR("    ... error: fail to open \"%s\"", node_config.gyrodometry_log.value.at(0).toStdString().c_str());
                ros::shutdown();
            }
        }

    } // <--- initialize

    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    // finalize
    ROS_INFO("---------- finalize ----------");
    if( log_file.isOpen() )
    {
        log_file.close();
    }

    return 0;
}
