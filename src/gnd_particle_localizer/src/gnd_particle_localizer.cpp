/*
 * gnd_particle_localizer.cpp
 *
 *  Created on: 2014/09/05
 *      Author: tyamada
 *       Brief: evaluate self location
 *  Updated by ryu, 2023/9/21
 *  .Use Qt v5.14 lib to update self location by particles
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
#include "gnd_msgs/msg_velocity2d_with_covariance_stamped.h"
#include "gnd_particle_localizer/msg_particles_pose2d_stamped.h"
#include "gnd_particle_localizer/msg_particle_weights_stamped.h"
#include "gnd_particle_localizer/srv_reset_particles_normal_distribution.h"
#include "gnd_particle_localizer_config.h"
#include "gnd_rosutil.h"
#include "gnd-random.h"


const uint16_t MAX_SIZE = 1000;
gnd::data_buff<gnd_particle_localizer::msg_particles_pose2d_stamped>             particle_buff(MAX_SIZE);
gnd_particle_localizer::msg_particle_weights_stamped::_weights_type::value_type  log_likelihood_max;
gnd_particle_localizer::msg_particles_pose2d_stamped	                           msg_particles;

QVector<QVector<double> > systematic_error_ratio_depened_on_translate;

gnd::particle_localizer::node_config                 node_config;
gnd_particle_localizer::msg_particle_weights_stamped particle_weights_integrated;		// likelihood ratio
gnd_msgs::msg_velocity2d_with_covariance_stamped     msg_motion;

ros::Publisher pose_pub;
ros::Publisher particles_pub;
gnd_msgs::msg_pose2d_stamped msg_pose;
bool flg_resampling;

void motion_Callback(const gnd_msgs::msg_velocity2d_with_covariance_stamped::ConstPtr& msg)
{
    double trans_x, trans_y, rot;
    double cosv, sinv;

    msg_motion = *msg;
    trans_x = msg->vel_x * msg->measuring_period;
    trans_y = msg->vel_y * msg->measuring_period;
    rot = msg->vel_ang * msg->measuring_period;

    // pose
    cosv = cos(msg_pose.theta);
    sinv = sin(msg_pose.theta);
    msg_pose.x += cosv * trans_x - sinv * trans_y;
    msg_pose.y += sinv * trans_x + cosv * trans_y;
    msg_pose.theta += rot;

    { // ---> particles transition
        QGenericMatrix<3,3,double> error_covariance_vel;

        error_covariance_vel(0,0) = msg->covariance[0];
        error_covariance_vel(0,1) = error_covariance_vel(1,0) = msg->covariance[1];
        error_covariance_vel(0,2) = error_covariance_vel(2,0) = msg->covariance[2];
        error_covariance_vel(1,1) = msg->covariance[4];
        error_covariance_vel(1,2) = error_covariance_vel(2,1) = msg->covariance[5];
        error_covariance_vel(2,2) = msg->covariance[8];

        for( unsigned int i = 0; i < msg_particles.poses.size(); i++ )
        {
            QGenericMatrix<3,3,double> cp_error_cov;
            QVector<double> ws(3), rand_err, sys_err(3);

            // random error
            error_covariance_vel.copyDataTo(cp_error_cov.data());
            gnd::random_gaussian_mult<3>(&cp_error_cov, &ws, &rand_err);
            // systematic error
            sys_err[0] = systematic_error_ratio_depened_on_translate[i][0] * msg->vel_x;
            sys_err[1] = systematic_error_ratio_depened_on_translate[i][1] * msg->vel_x;
            sys_err[2] = systematic_error_ratio_depened_on_translate[i][2] * msg->vel_x;


            trans_x = ( msg->vel_x + rand_err[0] + sys_err[0]) * msg->measuring_period;
            trans_y = ( msg->vel_y + rand_err[1] + sys_err[1] ) * msg->measuring_period;
            rot = ( msg->vel_ang + rand_err[2] + sys_err[2] ) * msg->measuring_period;

            // pose calculation
            cosv = cos(msg_particles.poses[i].theta);
            sinv = sin(msg_particles.poses[i].theta);
            msg_particles.poses[i].x += cosv * trans_x - sinv * trans_y;
            msg_particles.poses[i].y += sinv * trans_x + cosv * trans_y;
            msg_particles.poses[i].theta += rot;
        }
    } // <--- particles transition

    // publish
    msg_pose.header.stamp = msg->header.stamp;
    msg_pose.header.seq++;
    pose_pub.publish(msg_pose);

    msg_particles.header.stamp = msg->header.stamp;
    msg_particles.header.seq++;
    particles_pub.publish(msg_particles);
}

void particle_weights_Callback(const gnd_particle_localizer::msg_particle_weights_stamped::ConstPtr& msg_particle_weights)
{
    // ---> update particle weights by measurement model
    if(   (msg_particle_weights->seq_particles > particle_weights_integrated.seq_particles)
       || (   (particle_weights_integrated.seq_particles >= UINT32_MAX / 2)
           && (msg_particle_weights->seq_particles < UINT32_MAX / 2)))
    {
        if(msg_particle_weights->weights.size() == particle_weights_integrated.weights.size())
        {
            // integrate
            gnd_particle_localizer::msg_particle_weights_stamped::_weights_type::value_type sum;
            for( uint i = 0; i < particle_weights_integrated.weights.size(); i++ )
            {
                // integrate a particle weight
                particle_weights_integrated.weights[i] *= msg_particle_weights->weights[i];

                // zero weight exception
                if( particle_weights_integrated.weights[i] < FLT_EPSILON)
                {
                    particle_weights_integrated.weights[i] = FLT_EPSILON;
                }
                // sum
                sum += particle_weights_integrated.weights[i];
            }

            // normalize (make the sum of weights to be equal 1.0) to avoid zero weight due to rounding error
            for( uint i = 0; i < particle_weights_integrated.weights.size(); i++ )
            {
                particle_weights_integrated.weights[i] /= sum;
            }

            // resampling is possible
            flg_resampling = true;
        }
    }

    // ---> resampling according to particle weights
    if(flg_resampling )
    {
        uint i, j;
        gnd_particle_localizer::msg_particles_pose2d_stamped::_poses_type copy_poses = msg_particles.poses;
        QVector<QVector<double> > copy_syserr = systematic_error_ratio_depened_on_translate;
        gnd_particle_localizer::msg_particles_pose2d_stamped::_poses_type::value_type sum_poses;

        sum_poses.x = 0;
        sum_poses.y = 0;
        sum_poses.theta = 0;

        for( i = 0; i < msg_particles.poses.size(); i++ )
        {
            double rand = gnd::random_uniform(); // the sum of weights is equal 1.0
            double sum = 0;

            // select a particle according to weight liner probability
            // scanning loop,
            for( j = 0; j < particle_weights_integrated.weights.size() - 1; j++ )
            {
                sum += particle_weights_integrated.weights[j];
                if( sum > rand ) break;
            }

            { // ---> set a new particle
                msg_particles.poses[i] = copy_poses[j];
                // sum
                sum_poses.x += msg_particles.poses[i].x;
                sum_poses.y += msg_particles.poses[i].y;
                sum_poses.theta += msg_particles.poses[i].theta;
            } // <--- set a new particle

            { // ---> set systematic error ratio
                if( node_config.probability_change_systematic_motion_error.value.at(0) < gnd::random_uniform() )
                {
                    // not change
                    systematic_error_ratio_depened_on_translate[i] = copy_syserr[j];
                }
                else
                {
                    // change
                    systematic_error_ratio_depened_on_translate[i][0] = node_config.standard_systematic_motion_error.value[0] * gnd::random_gaussian(1.0);
                    systematic_error_ratio_depened_on_translate[i][1] = node_config.standard_systematic_motion_error.value[1] * gnd::random_gaussian(1.0);
                    systematic_error_ratio_depened_on_translate[i][2] = node_config.standard_systematic_motion_error.value[2] * gnd::random_gaussian(1.0);
                }
            } // <--- set systematic error ratio
        }

        // correct robot pose ( selected particles average )
        msg_pose.x = sum_poses.x / msg_particles.poses.size();
        msg_pose.y = sum_poses.y / msg_particles.poses.size();
        msg_pose.theta = sum_poses.theta / msg_particles.poses.size();

        // update sequence when resampling
        particle_weights_integrated.seq_particles = msg_particles.header.seq + 1;
        // reset weights
        particle_weights_integrated.weights.clear();
        particle_weights_integrated.weights.resize( msg_particles.poses.size(), (float) 1.0 / msg_particles.poses.size() );
        // have no evaluations
        flg_resampling = false;

        // publish(pose)
        msg_pose.header.stamp = msg_motion.header.stamp;
        msg_pose.header.seq++;
        pose_pub.publish(msg_pose);

        // publish(particles)
        msg_particles.header.stamp = msg_motion.header.stamp;
        msg_particles.header.seq++;
        particles_pub.publish(msg_particles);

    } // <--- resampling

}

bool reset_particle_callback(gnd_particle_localizer::srv_reset_particles_normal_distribution::Request &req,
                             gnd_particle_localizer::srv_reset_particles_normal_distribution::Response &res)
{
    int size;

    ROS_INFO("reset particles");
    // set pose
    msg_pose.x = req.x;
    msg_pose.y = req.y;
    msg_pose.theta = req.theta;

    // resize
    if(req.size > 0)
    {
        msg_particles.poses.resize(req.size);
    }

    { // ---> initialize particles
        QGenericMatrix<3,3,double> cov;
        QVector<double> rand(3), ws(3);


        for (uint i = 0; i < node_config.number_of_particles.value.at(0); i++ )
        {
            // set co-variance
            cov(0,0) = req.covariance[0];
            cov(0,1) = req.covariance[1];
            cov(0,2) = req.covariance[2];
            cov(1,0) = req.covariance[3];
            cov(1,1) = req.covariance[4];
            cov(1,2) = req.covariance[5];
            cov(2,0) = req.covariance[6];
            cov(2,1) = req.covariance[7];
            cov(2,2) = req.covariance[8];

            // create random value according to co-variance
            gnd::random_gaussian_mult<3>(&cov, &ws, &rand);

            // add random value
            msg_particles.poses[i].x = msg_pose.x + rand[0];
            msg_particles.poses[i].y = msg_pose.y + rand[1];
            msg_particles.poses[i].theta = gnd::rad_normalize( msg_pose.theta + rand[2] );
        }
    } // <--- initialize particles

    // publish(pose)
    msg_pose.header.stamp = msg_motion.header.stamp;
    msg_pose.header.seq++;
    pose_pub.publish(msg_pose);

    // publish(particles)
    msg_particles.header.stamp = msg_motion.header.stamp;
    msg_particles.header.seq++;
    particles_pub.publish(msg_particles);

    // weight reset
    particle_weights_integrated.weights.clear();
    particle_weights_integrated.weights.resize( msg_particles.poses.size(), (float) 1.0 / msg_particles.poses.size() );

    ROS_INFO("particle_reset ok");

    res.ret = true;
    return res.ret;
}

int main(int argc, char **argv)
{
    // start up, read configuration file
    if( argc > 1 )
    {
        if( gnd::particle_localizer::fread_node_config( argv[1], &node_config ) < 0 )
        {
            ROS_ERROR("   ... Error: fail to read config file %s", argv[1]);
            QString fname = QString(argv[1]) + ".tmp";
            // file out configuration file
            if( gnd::particle_localizer::fwrite_node_config( fname, &node_config ) >= 0 )
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
        ROS_ERROR("   ... Error: node name is null, you must specify the name of this node via config item \"%s\"",
                  node_config.node_name.item.toStdString().c_str());
        return -1;
    }
    ROS_INFO(" node: \"%s\"", node_config.node_name.value.at(0).toStdString().c_str());

    // ---> variables
    ros::NodeHandle     nh;					                // ros nodehandle
    ros::Subscriber     motion_sub;		              // point-cloud subscriber
    ros::Subscriber     particle_weights_sub;		    // particle subscriber
    ros::ServiceServer  reset_particles_nd_srvserv; //reset particle location server
    ros::Time           time_start = ros::Time::now();

    int phase = 0;
    ROS_INFO("---------- initialization ----------");
    // show initialize phase task
    if( ros::ok() )
    {
        ROS_INFO(" => initialization task");
        ROS_INFO("   %d. make estimated pose publisher", ++phase);
        ROS_INFO("   %d. make motion subscribe", ++phase);
        ROS_INFO("   %d. make particles publisher", ++phase);
        ROS_INFO("   %d. make particle weights subscriber", ++phase);
        ROS_INFO("   %d. make service server to reset particles", ++phase);
    }

    phase = 0;
    // ---> make estimated pose publisher
    if(ros::ok())
    {
        ROS_INFO("  => %d. make estimated pose publisher", ++phase);
        if(node_config.topic_name_pose.value.at(0).isEmpty())
        {
            ros::shutdown();
            ROS_ERROR("   ... error: invalid topic name");
        }
        else
        {
            ROS_INFO("    ... topic name is \"%s\"", node_config.topic_name_pose.value.at(0).toStdString().c_str());
            pose_pub = nh.advertise<gnd_msgs::msg_pose2d_stamped>(node_config.topic_name_pose.value.at(0).toStdString(),10);
            msg_pose.header.stamp = time_start;
            msg_pose.header.seq = 0;
            msg_pose.header.frame_id = "";
            msg_pose.x = node_config.initial_pose.value.at(0);
            msg_pose.y = node_config.initial_pose.value.at(1);
            msg_pose.theta = node_config.initial_pose.value.at(2);
            ROS_INFO("   ... ok");
        }
    } // <--- make estimated pose publisher

    // ---> make motion subscriber
    if( ros::ok() )
    {
        ROS_INFO("   => %d. make motion subscriber", ++phase);
        if( node_config.topic_name_motion.value.at(0).isEmpty() )
        {
            ROS_ERROR("    ... error: invalid topic name");
            ros::shutdown();
        }
        else
        {
            ROS_INFO("    ... topic name is \"%s\"\n", node_config.topic_name_motion.value.at(0).toStdString().c_str());
            // make subscriber
            motion_sub = nh.subscribe(node_config.topic_name_motion.value.at(0).toStdString(), 10, motion_Callback);
            ROS_INFO("    ... ok");
        }
    } // <--- make motion subscriber

    // ---> make particles publisher
    if( ros::ok() )
    {
        ROS_INFO("   => %d. ake estimated pose publisher", ++phase);

        if(node_config.topic_name_particles.value.at(0).isEmpty() )
        {
            ROS_ERROR("    ... error: invalid topic name");
            ros::shutdown();
        }
        else
        {
            ROS_INFO("    ... topic name is \"%s\"", node_config.topic_name_particles.value.at(0).toStdString().c_str() );
            particles_pub = nh.advertise<gnd_particle_localizer::msg_particles_pose2d_stamped>(node_config.topic_name_particles.value.at(0).toStdString(), 1);
            msg_particles.header.stamp = time_start;
            msg_particles.header.seq = 0;
            msg_particles.header.frame_id = node_config.node_name.value.at(0).toStdString();
            msg_particles.poses.resize((uint)node_config.number_of_particles.value.at(0));
            msg_particles.schedule_resampling = time_start;

            // systematic error
            systematic_error_ratio_depened_on_translate.resize((uint)node_config.number_of_particles.value.at(0));

            { // ---> initialize particles
                gnd_particle_localizer::msg_particles_pose2d_stamped::_poses_type::value_type init_particle;
                QGenericMatrix<3,3,double> cov;
                QVector<double> ws(3), pos(3);


                for (uint i = 0; i < node_config.number_of_particles.value.at(0); i++ )
                {
                    // set co-variance
                    cov(0,0) = node_config.error_covariance_initial_pose.value.at(0);
                    cov(0,1) = cov(1,0) = node_config.error_covariance_initial_pose.value.at(1);
                    cov(0,2) = cov(2,0) = node_config.error_covariance_initial_pose.value.at(2);
                    cov(1,1) = node_config.error_covariance_initial_pose.value.at(4);
                    cov(1,2) = cov(2,1) = node_config.error_covariance_initial_pose.value.at(5);
                    cov(2,2) = node_config.error_covariance_initial_pose.value.at(8);
                    // create random value according to co-variance
                    gnd::random_gaussian_mult<3>(&cov, &ws, &pos);
                    // add random value
                    init_particle.x = node_config.initial_pose.value.at(0) + pos[0];
                    init_particle.y = node_config.initial_pose.value.at(1) + pos[1];
                    init_particle.theta = node_config.initial_pose.value.at(2) + pos[2];
                    // set
                    msg_particles.poses[i] = init_particle;
                    systematic_error_ratio_depened_on_translate[i].resize(3);
                    systematic_error_ratio_depened_on_translate[i][0] = 0;
                    systematic_error_ratio_depened_on_translate[i][1] = 0;
                    systematic_error_ratio_depened_on_translate[i][2] = 0;
                }
            } // <--- initialize particles

            ROS_INFO("    ... ok");
        }
    } // <--- make particles publisher

    // ---> make particle weights subscriber
    if( ros::ok() )
    {
        ROS_INFO("    => %d. make particle weights subscriber", ++phase);

        if( node_config.topic_name_particle_weights.value.at(0).isEmpty() )
        {
            ROS_ERROR("    ... error: invalid topic name");
            ros::shutdown();
        }
        else
        {
            ROS_INFO("    ... topic name is \"%s\"\n", node_config.topic_name_particle_weights.value.at(0).toStdString().c_str());
            // make subscriber
            particle_weights_sub = nh.subscribe(node_config.topic_name_particle_weights.value.at(0).toStdString(), 10, particle_weights_Callback);
            ROS_INFO("    ... ok");
        }
    } // <--- make particle weights subscriber

    // ---> make reset_particles service server
    if( ros::ok() )
    {
        ROS_INFO(" => %d. make service server to reset particles", ++phase);

        if( node_config.service_name_reset_particles_nd.value.at(0).isEmpty() )
        {
            ROS_INFO("    ... error: invalid service name");
            ros::shutdown();
        }
        else
        {
            ROS_INFO("    ... service name is \"%s\"", node_config.service_name_reset_particles_nd.value.at(0).toStdString().c_str() );

            // service server
            reset_particles_nd_srvserv = nh.advertiseService(node_config.service_name_reset_particles_nd.value.at(0).toStdString(), reset_particle_callback);
            fprintf(stdout, "    ... ok\n");
        }
    } // <--- make reset_particle service server

    // ---> operation
    if(ros::ok())
    {
        // initialize integrated particle weights storage
        particle_weights_integrated.weights.resize( msg_particles.poses.size(), (float) 1.0 / msg_particles.poses.size() );
        flg_resampling = false;

        ROS_INFO("  => %s main loop start", node_config.node_name.value.at(0).toStdString().c_str());
        // --->main loop
        ros::Rate loop_rate(1000);
        while(ros::ok())
        {
            loop_rate.sleep();
            ros::spinOnce();
        }

    }// <--- operation

    { // ---> finalize
        ROS_INFO("----------finalize----------");

        ROS_INFO("   ... end");
    } // <--- finalize

    return 0;
}

