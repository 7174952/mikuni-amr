/*
 * icartmini_sbtp.h
 *
 *  Created on: 2014/09/05
 *      Author: tyamada
 *       Brief: deterimine running route
 *  Updated by ryu, 2023/9/29
 *  .Use Qt v5.14 lib to update config file
 */
#include <QString>
#include <QFile>
#include <QTextStream>
#include <QGenericMatrix>
#include <QtMath>
#include <QQueue>
#include <QVector>
#include <QGenericMatrix>

#include "icartmini_sbtp_config.h"
#include "icartmini_sbtp/srv_set_navigation_path.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose2D.h"
#include "gnd-matrix-coordinate.h"
#include "gnd-coord-tree.h"
#include "gnd_rosutil.h"
#include "gnd-random.h"
#include "gnd-path-io.h"

#include "gnd_msgs/msg_path_area_and_speed_limited.h"
#include "gnd_msgs/msg_pose2d_stamped.h"
#include "gnd_msgs/msg_velocity2d_with_covariance_stamped.h"
#include "gnd_msgs/msg_path_area_and_speed_limited.h"
#include "gnd_msgs/msg_vehicle_status.h"
#include "gnd_msgs/msg_trajectory.h"

ros::Publisher						                  pub_planned_path;
gnd_msgs::msg_path_area_and_speed_limited		msg_planned_path;

ros::Publisher						                  pub_vehicle_stauts;
gnd_msgs::msg_vehicle_status				        msg_vehicle_status;

ros::Publisher						                  pub_trajectory_target;
gnd_msgs::msg_trajectory 					          msg_trajectory_target;

ros::Publisher						                  pub_trajectory_actual;
gnd_msgs::msg_trajectory					          msg_trajectory_actual;

//send to amr
ros::Publisher                      pub_amr_trajectory_pose;
geometry_msgs::Pose2D               msg_amr_trajectory_pose;

ros::Publisher                      pub_amr_set_vel;
std_msgs::Float64MultiArray         msg_amr_set_vel;

//turn round state
const uint STATE_NONE      = 0;
const uint STATE_TURN_BACK = 1;
const uint STATE_RUN_LINE  = 2;
const uint STATE_TURN_LR   = 3;

gnd_msgs::msg_waypoint_directional_named  last_start_waypoint;
gnd_msgs::msg_pose2d_stamped msg_pose;
gnd_msgs::msg_velocity2d_with_covariance_stamped msg_velocity;
uint turn_round_state = STATE_NONE;
float last_vel_dir = 0.0;
double prev_velocity = 0.0;


int flgs_errorstate = 0x00000000;
// coordinate
gnd::coord_tree coordinate_tree;
int coordinate_id_global  = -1;
int coordinate_id_robot   = -1;
int coordinate_id_pathend = -1;
int coordinate_id_trajectory_target = -1;
int coordinate_id_trajectory_actual = -1;

double time_pose_update;
double time_pointcloud_update;


typedef struct
{
    ros::Subscriber						      pointcloud_sub;
    sensor_msgs::PointCloud					msg_pointcloud;
} topic_pointcloud_t;
topic_pointcloud_t *topics_pointcloud;

void amr_stop()
{
    msg_amr_set_vel.data[0] = 0; //vehicle vel
    msg_amr_set_vel.data[1] = 0; //angle vel
    pub_amr_set_vel.publish(msg_amr_set_vel);
	
}

void amr_set_vel(double vel, double angvel)
{
    msg_amr_set_vel.data[0] = vel; //vehicle vel
    msg_amr_set_vel.data[1] = angvel; //angle vel
    pub_amr_set_vel.publish(msg_amr_set_vel);

}

void amr_set_trajectory_pose(double x, double y, double theta)
{
    msg_amr_trajectory_pose.x = x;
    msg_amr_trajectory_pose.y = y;
    msg_amr_trajectory_pose.theta = theta;
    pub_amr_trajectory_pose.publish(msg_amr_trajectory_pose);

}

bool srv_set_navi_callback(icartmini_sbtp::srv_set_navigation_path::Request& request, icartmini_sbtp::srv_set_navigation_path::Response& response)
{
    response.ret = true;

    // set path
    msg_planned_path.path.clear();
    msg_planned_path = request.path;

    // publish new path
    pub_planned_path.publish(msg_planned_path);

    if( msg_planned_path.path.size() > 0 )
    { // ---> update the current path coordinate
        QGenericMatrix<4,4,double> coord_tf_matrix;

        gnd::matrix::coordinate_converter(&coord_tf_matrix,
            msg_planned_path.path[0].end.x, msg_planned_path.path[0].end.y, 0.0,
            cos(msg_planned_path.path[0].end.theta), sin(msg_planned_path.path[0].end.theta), 0.0,
            0.0, 0.0, 1.0);
        coordinate_tree.set_coordinate( coordinate_id_pathend, &coord_tf_matrix );
    } // <--- update the current path coordinate

    if(last_start_waypoint.name.size() > 0)
    {
        if(last_start_waypoint.name ==  msg_planned_path.path[0].end.name)
        {
            turn_round_state = STATE_TURN_BACK;
        }
        else
        {
            turn_round_state = STATE_TURN_LR;
            last_vel_dir = prev_velocity;
        }
    }

    return true;
}

void pose_callback(const gnd_msgs::msg_pose2d_stamped::ConstPtr& msg)
{
    time_pose_update = ros::Time::now().toSec();
    msg_pose = *msg;
}

void pointcloud_callback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    topics_pointcloud->msg_pointcloud = *msg;
    time_pointcloud_update = ros::Time::now().toSec();

}

int main(int argc, char **argv)
{
    gnd::icartmini_sbtp::node_config node_config;

    // ---> start up, read configuration file
    if( argc > 1 )
    {
        if( gnd::icartmini_sbtp::fread_node_config( argv[1], &node_config ) < 0 )
        {
            ROS_ERROR("   ... error : fail to read configuration file %s", argv[1]);
            QString fname = QString(argv[1]) + ".tmp";
            // file out configuration file
            if( gnd::icartmini_sbtp::fwrite_node_config( fname, &node_config ) >= 0 )
            {
                ROS_INFO("  : output sample configuration file %s", fname.toStdString().c_str());
            }
            return -1;
        }
        else
        {
            ROS_INFO("   ... read configuration file %s", argv[1]);
        }
    }

    // ---> initialize ros
    if( !node_config.node_name.value[0].isEmpty() )
    {
        ros::init(argc, argv, node_config.node_name.value.at(0).toStdString());
		}
    else
    {
        ROS_ERROR("   ... Error: node name is null, you must specify the name of this node via config item %s", node_config.node_name.item.toStdString().c_str());
        return -1;
		}
    ROS_INFO(" node: %s", node_config.node_name.value.at(0).toStdString().c_str());

    // ---> ros communication object
    ros::NodeHandle                 nh;
    ros::Subscriber                 pose_sub;

    int ntopics_pointcloud = 0;

//    ros::Subscriber					  velocity_sub;
//    ros::Subscriber						mode_status_sub;
//	msg_mode_status_t					msg_mode_status;
//	msgreader_mode_status_t				msgreader_mode_status;

    ros::ServiceServer					srvserver_set_navigation_path;
//	srv_funcobj_set_navigation_path_t	srv_funcobj_set_navigation_path;
	// <--- ros communication object

    gnd::path::path_net_area_and_speed_limited  path_net;

    // initialize
		int phase = 0;
		ros::Time time_start;
    ROS_INFO("---------- initialize ----------");

		// ---> show initialization task
    if( ros::ok() )
    {
        ROS_INFO(" => show task");
        if( node_config.path_file.value.at(0).size() )
        {
            ROS_INFO("   %d. load route data file", ++phase);
        }
        if( node_config.path_file.value.at(0).size() && node_config.start_node.value.at(0).size() && node_config.dest_node.value.at(0).size() )
        {
            ROS_ERROR("    %d. find path", ++phase);
        }
        ROS_INFO("   %d. make pose subscriber for navigation", ++phase);
        if( node_config.topic_names_pointcloud.value.at(0).size() > 0 )
        {
            ROS_INFO("   %d. make point-cloud subscriber for obstacle detection", ++phase);
        }
        ROS_INFO("   %d. make vehicle control command publisher", ++phase);
		} // <--- show initialization task

		phase = 0;
		// ---> load route data file
    if( ros::ok() && node_config.path_file.value.at(0).size())
    {
        ROS_INFO(" => %d. load route data file\n", ++phase);
        ROS_INFO("        file path is \"%s\"\n", node_config.path_file.value.at(0).toStdString().c_str());

        if( gnd::path::fread(node_config.path_file.value.at(0).toStdString().c_str(), &path_net) < 0 )
        {
          ROS_ERROR("    ... error : fail to read route file");
          ros::shutdown();
        }
        else {
          ROS_INFO("    ... ok");
        }
		} // <--- load route data file

		// ---> search path
    if( ros::ok() && path_net.n_waypoints() > 0 && node_config.start_node.value.at(0).size() > 0 && node_config.dest_node.value.at(0).size() > 0 )
    {
        gnd::path::path_area_and_speed_limited ws;
        ROS_INFO(" => %d. find path from \"%s\" to \"%s\"\n", ++phase, node_config.start_node.value.at(0).toStdString().c_str(), node_config.dest_node.value.at(0).toStdString().c_str());

        // plan a path to destination
        if( path_net.find_path_dijkstra( &ws, node_config.start_node.value.at(0).toStdString().c_str(), node_config.dest_node.value.at(0).toStdString().c_str()) < 0 )
        {
            ROS_ERROR("    ... error : fail to find path");
            ros::shutdown();
        }
        else
        {
            int i;

            ROS_INFO("path: %s", ws.start.name);
            msg_planned_path.start.x = ws.start.x;
            msg_planned_path.start.y = ws.start.y;
            msg_planned_path.start.theta = 0;
            msg_planned_path.start.name = ws.start.name;
            for( i = 0; i < (signed) ws.path.size(); i++ )
            {
                gnd_msgs::msg_path_unit_area_and_speed_limited unit;

                ROS_INFO(" -> %s", ws.path[i].end.name );
                unit.end.x = ws.path[i].end.x;
                unit.end.y = ws.path[i].end.y;
                unit.end.theta = ws.path[i].end.theta;
                unit.end.name = ws.path[i].end.name;

                unit.curvature = ws.path[i].curvature;
                unit.limit_translate = ws.path[i].prop.limit_translate;
                unit.limit_rotate = ws.path[i].prop.limit_rotate;

                msg_planned_path.path.push_back(unit);
            }

            ROS_ERROR("    ... ok");
        }
		} // <--- search path

		// ---> make pose subscriber for navigation
    if( ros::ok() )
    {
        ROS_INFO(" =>  %d. make pose subscriber for navigation\n", ++phase);

        if( !node_config.topic_name_pose.value.at(0).size() )
        {
            ROS_ERROR("    ... error: invalid topic name");
            ros::shutdown();
        }
        else
        {
            ROS_INFO("    ... topic name is \"%s\"", node_config.topic_name_pose.value.at(0).toStdString().c_str() );

            // subscribe
            pose_sub = nh.subscribe(node_config.topic_name_pose.value.at(0).toStdString(), 10, pose_callback);

            ROS_INFO("    ... ok");
        }
		} // <--- make pose subscriber for navigation


		// ---> make point-cloud subscriber for obstacle detection
    if( ros::ok() && node_config.topic_names_pointcloud.value.size() > 0)
    {
        ROS_INFO(" => %d. make point-cloud subscriber for obstacle detection", ++phase);

        topics_pointcloud = new topic_pointcloud_t[node_config.topic_names_pointcloud.value.size()];
        if( !topics_pointcloud )
        {
          ROS_ERROR("    ... error: fail to allocate memories");
          ros::shutdown();
        }
        else
        {
            ntopics_pointcloud = node_config.topic_names_pointcloud.value.size();

            for( int i = 0; i < (signed)node_config.topic_names_pointcloud.value.size(); i++ )
            {
                if( !node_config.topic_names_pointcloud.value.at(i).size() )
                {
                    ROS_ERROR("    ... error: invalid topic name");
                    ros::shutdown();
                }
                else
                {
                    ROS_INFO("    ... topic name is \"%s\"", node_config.topic_names_pointcloud.value.at(i).toStdString().c_str() );

                    // subscribe. same callback function was used
                    topics_pointcloud[i].pointcloud_sub = nh.subscribe(node_config.topic_names_pointcloud.value.at(i).toStdString(), 10, pointcloud_callback);

                    ROS_INFO("    ... ok");
                }
            }
        }
		} // <--- make point-cloud subscriber for obstacle detection

		// ---> initialize paths publisher
    if( ros::ok() )
    {
        ROS_INFO(" =>  %d. make planed paths publisher", ++phase);

        if( !node_config.topic_name_planned_path.value.at(0).size() )
        {
            ros::shutdown();
            ROS_ERROR("    ... error: invalid topic name");
        }
        else
        {
            ROS_INFO("    ... topic name is \"%s\"", node_config.topic_name_planned_path.value.at(0).toStdString().c_str() );
            pub_planned_path = nh.advertise<gnd_msgs::msg_path_area_and_speed_limited>(node_config.topic_name_planned_path.value.at(0).toStdString(), 5 );
            ROS_INFO("    ... ok");
        }
		} // <--- initialize paths publisher

		//init amr publisher
    if( ros::ok() )
    {
        pub_amr_set_vel = nh.advertise<std_msgs::Float64MultiArray>("amr_set_vel",10);
        msg_amr_set_vel.data.resize(2);

        pub_amr_trajectory_pose = nh.advertise<geometry_msgs::Pose2D>("amr_trajectory_pose",10);
		} 

		// ---> initialize target trajectory publisher
    if( ros::ok() )
    {
        ROS_INFO(" =>  %d. make the target trajectory publisher", ++phase);

        if( !node_config.topic_name_trajectory_target.value.at(0).size() )
        {
            ros::shutdown();
            ROS_INFO("    ... error: invalid topic name");
        }
        else
        {
            ROS_INFO("    ... topic name is \"%s\"", node_config.topic_name_trajectory_target.value.at(0).toStdString().c_str() );
            pub_trajectory_target = nh.advertise<gnd_msgs::msg_trajectory>(node_config.topic_name_trajectory_target.value.at(0).toStdString(), 2 );
            ROS_INFO("    ... ok");
        }
		} // <--- initialize target trajectory publisher


		// ---> initialize actual trajectory publisher
    if( ros::ok() )
    {
        ROS_INFO(" =>  %d. make actual trajectory publisher", ++phase);

        if( !node_config.topic_name_trajectory_actual.value.at(0).size() )
        {
            ros::shutdown();
            ROS_ERROR("    ... error: invalid topic name");
        }
        else
        {
            ROS_INFO("    ... topic name is \"%s\"", node_config.topic_name_trajectory_actual.value.at(0).toStdString().c_str() );
            pub_trajectory_actual = nh.advertise<gnd_msgs::msg_trajectory>(node_config.topic_name_trajectory_actual.value.at(0).toStdString(), 2 );

            ROS_INFO("    ... ok");
        }
		} // <--- initialize actual trajectory publisher


		// ---> initialize vehicle status publisher
    if( ros::ok() )
    {
        ROS_INFO(" =>  %d. make vehiclle status publisher", ++phase);

        if( !node_config.topic_name_vehicle_status.value.at(0).size() )
        {
            ros::shutdown();
            ROS_ERROR("    ... error: invalid topic name");
        }
        else
        {
            ROS_INFO("    ... topic name is \"%s\"", node_config.topic_name_vehicle_status.value.at(0).toStdString().c_str() );
            pub_vehicle_stauts = nh.advertise<gnd_msgs::msg_vehicle_status>(node_config.topic_name_vehicle_status.value.at(0).toStdString(), 5 );

            ROS_INFO("    ... ok");
        }
		} // <--- initialize vehicle status publisher

    // ---> make service server
    if( ros::ok() )
    {
        ROS_INFO(" => %d. make service server to reset particles", ++phase);

        if( !node_config.service_name_set_navigation_path.value.at(0).size() )
        {
            ROS_ERROR("    ... error: invalid service name");
            ros::shutdown();
        }
        else
        {
            ROS_INFO("    ... service name is \"%s\"", node_config.service_name_set_navigation_path.value.at(0).toStdString().c_str() );
            // service server
            srvserver_set_navigation_path = nh.advertiseService(node_config.service_name_set_navigation_path.value.at(0).toStdString(), srv_set_navi_callback);
            ROS_INFO("    ... ok");
        }
		} // <--- make service server

    if( ros::ok() )
    { // ---> operate
        typedef struct point_t
        {
            double x;
            double y;
        } point_t;
        typedef struct trajectory_evaluation_elements
        {
            double clearance;
            double depth;
            double dir;
            double dist_to_end;
        } trajectory_evaluation_elements_t;
        typedef struct obstacle_for_line_and_turn_t
        {
            double depth;
            double clearance_dist;
        } obstacle_for_line_and_turn_t;

        const int Flags_ErrorState_LostPoisiton   = 0x00000001;
        const int Flags_ErrorState_LostPointcloud = 0x00000002;
        const int Flags_ErrorState_LostCompanion  = 0x00000004;

        const double Period_ErrorState_LostPosition   = 0.25;
        const double Period_ErrorState_LostPointcloud = 0.25;
        const double Period_ErrorState_LostCompanion  = 1.0;

        ros::Rate loop_rate(1000);

        // time
        double time_start;
        double time_current;

        QQueue<point_t> pointcloud_on_pathend;

        // planning
        obstacle_for_line_and_turn_t		  obstalce_for_motion;
        gnd::path::trajectory_t 			    trajectory_target;
        trajectory_evaluation_elements_t	eval_trajectory_target_for_velocity_determination;
        gnd::path::trajectory_t				    trajectory_actual;
        trajectory_evaluation_elements_t	eval_trajectory_actual_for_velocity_determination;
        double search_range_right, search_range_left;
        double schedule_planning;
        double schedule_reselect_trajecotry;
        double schedule_publish_planning;
        double velocity = 0.0;
        double prev_velocity = 0.0;


        flgs_errorstate = 0x00000000;

        QQueue<trajectory_evaluation_elements_t> eval_trajectories;
        trajectory_evaluation_elements_t eval_traj_temp;

        // send commands
        double schedule_publish_planed_path;
        double period_publish_planed_path = 3.0;

        // terminal display
        int nline_display = 0;
        double schedule_display;

        { // ---> initialize time variables
            time_start = ros::Time::now().toSec();
            time_current = time_start;
        } // <--- initialize time variables

        { // ---> initialize message header
            for( int i = 0; i < ntopics_pointcloud; i++)
            {
                topics_pointcloud[i].msg_pointcloud.header.seq = 0;
                topics_pointcloud[i].msg_pointcloud.header.stamp.fromSec(0.0);
            }
            msg_pose.header.seq = 0;
            msg_pose.header.stamp.fromSec(0.0);

            msg_velocity.header.seq = 0;
            msg_velocity.header.stamp.fromSec(0.0);
        } // <--- initialize message header

        { // ---> initialize variables for planning
            trajectory_target.lines.clear();
            trajectory_actual.lines.clear();
            schedule_reselect_trajecotry = time_start + node_config.period_reselect_trajectory.value.at(0);
            schedule_planning = time_start;
            time_pose_update = 0;
            time_pointcloud_update = 0;
            flgs_errorstate = Flags_ErrorState_LostPoisiton | Flags_ErrorState_LostPointcloud;
            schedule_publish_planning = time_start;
        } // <--- initialize variables for planning

        { // ---> initialize variables to send commands
            schedule_publish_planed_path = time_start;
        } // <--- initialize variables to send commands

        { // ---> initialize coordinate
            QGenericMatrix<4,4,double> coord_tf_matrix;

            // define global coordinate
            coord_tf_matrix.setToIdentity();
            coordinate_id_global = coordinate_tree.add_node("global", "root", &coord_tf_matrix);

            // initialize robot coordinate
            coord_tf_matrix.setToIdentity();
            coordinate_id_robot = coordinate_tree.add_node("robot", "global", &coord_tf_matrix);

            // initialize path coordinate
            if( msg_planned_path.path.size() > 0 )
            {
                gnd::matrix::coordinate_converter(&coord_tf_matrix,
                                                  msg_planned_path.path[0].end.x, msg_planned_path.path[0].end.y, 0.0,
                                                  cos(msg_planned_path.path[0].end.theta), sin(msg_planned_path.path[0].end.theta), 0.0,
                                                  0.0, 0.0, 1.0);
            }
            else
            {
                coord_tf_matrix.setToIdentity();
            }
            coordinate_tree.set_coordinate( coordinate_id_pathend, &coord_tf_matrix );
            coordinate_id_pathend = coordinate_tree.add_node("path", "global", &coord_tf_matrix);

            // initialize trajectory coordinate
            coord_tf_matrix.setToIdentity();
            coordinate_id_trajectory_target = coordinate_tree.add_node("trajectory_target", "global", &coord_tf_matrix);

            // initialize trajectory coordinate
            coord_tf_matrix.setToIdentity();
            coordinate_id_trajectory_actual = coordinate_tree.add_node("trajectory_actual", "global", &coord_tf_matrix);
        } // <--- initialize coordinate

        { // ---> initialize variables for terminal display
            schedule_display = time_start;
            nline_display = 0;
        } // <--- initialize variables for terminal display

        // ---> main loop
        ROS_INFO(" => main loop start");
        while( ros::ok() )
        {
            QVector<double> robot_on_pathend(2);
            double theta_robot_on_pathend = 0.0;

            // blocking to avoid the waste of computing resource
            loop_rate.sleep();

            // spin
            ros::spinOnce();

            // save time
            time_current = ros::Time::now().toSec();

            // ---> planning
            if( time_current > schedule_planning )
            {
                double length_current_path;
                double cos_on_current_pathend, sin_on_current_pathend;
                gnd::path::trajectory_unit_t traj_tmp;

                // obtain data
                // ---> check lost position or not
                if( time_current - time_pose_update > Period_ErrorState_LostPosition )
                {
                    // error: lost self position
                    flgs_errorstate |= Flags_ErrorState_LostPoisiton;
                }
                else
                {
                    QGenericMatrix<4,4,double> coord_tf_matrix;

                    flgs_errorstate &= ~Flags_ErrorState_LostPoisiton;

                    // define current robot coordinate
                    gnd::matrix::coordinate_converter(&coord_tf_matrix,
                        msg_pose.x, msg_pose.y, 0.0,
                        cos(msg_pose.theta), sin(msg_pose.theta), 0.0,
                        0.0, 0.0, 1.0);
                    coordinate_tree.set_coordinate( coordinate_id_robot, &coord_tf_matrix);
                } // <--- check lost position or not

                // ---> calculate the robot position on current path
                if( !(flgs_errorstate & Flags_ErrorState_LostPoisiton) && msg_planned_path.path.size() == 0)
                {
                    // path end
                    robot_on_pathend[0] = 0.0;
                    robot_on_pathend[1] = 0.0;
                    theta_robot_on_pathend = 0.0;
                }
                else if( !(flgs_errorstate & Flags_ErrorState_LostPoisiton) && (signed) msg_planned_path.path.size() > 0)
                {
                    // calculate robot position on path
                    cos_on_current_pathend = cos( msg_planned_path.path[0].end.theta );
                    sin_on_current_pathend = sin( msg_planned_path.path[0].end.theta );

                    robot_on_pathend[0] =   cos_on_current_pathend * ( msg_pose.x - msg_planned_path.path[0].end.x )
                                          + sin_on_current_pathend * ( msg_pose.y - msg_planned_path.path[0].end.y );
                    robot_on_pathend[1] = - sin_on_current_pathend * ( msg_pose.x - msg_planned_path.path[0].end.x )
                                          + cos_on_current_pathend * ( msg_pose.y - msg_planned_path.path[0].end.y );
                }
                else
                {
                    //Do nothing
                } // <--- calculate the robot position on current path

                if( time_current - time_pointcloud_update > Period_ErrorState_LostPointcloud )
                {
                    // error: lost self position
                    flgs_errorstate |= Flags_ErrorState_LostPointcloud;
                }
                else
                {
                    flgs_errorstate &= ~Flags_ErrorState_LostPointcloud;
                }


              // ---> path determination
              if( !flgs_errorstate )
              {
                  char flgs_transit_next = 0x00;
                  const char flg_transit_next = 0x01;
                  const char flg_transit_next_out_of_path = 0x02;
                  const char flg_transit_next_end_of_path = 0x04;
                  // transit the next path or not

                  // ---> when there is no path
                  if( (signed) msg_planned_path.path.size() == 0 )
                  {
                      // do nothing
                  } // <--- when there is no path
                  // ---> when the current path is last one
                  else if( (signed) msg_planned_path.path.size() == 1 )
                  {
                      // when the robot arrive at the current path-end
                      if( robot_on_pathend[0] > - node_config.pathend_margin.value.at(0) )
                      {
                          // transit to the next path
                          flgs_transit_next |= flg_transit_next;
                          flgs_transit_next |= flg_transit_next_end_of_path;
                      }
                  } // <--- when the current path is last one
                  // ---> when some paths area
                  else if( (signed) msg_planned_path.path.size() > 1 )
                  {
                      // check the distance to the path-end extend boundary
                      double length_next_path;
                      double x_robot_on_next_pathend, y_robot_on_next_pathend;
                      double cos_on_next_pathend, sin_on_next_pathend;

                      // calculate robot position on path
                      cos_on_next_pathend = cos( msg_planned_path.path[1].end.theta );
                      sin_on_next_pathend = sin( msg_planned_path.path[1].end.theta );

                      x_robot_on_next_pathend = cos_on_next_pathend * ( msg_pose.x - msg_planned_path.path[1].end.x )
                              + sin_on_next_pathend * ( msg_pose.y - msg_planned_path.path[1].end.y );
                      y_robot_on_next_pathend = - sin_on_next_pathend * ( msg_pose.x - msg_planned_path.path[1].end.x )
                              + cos_on_next_pathend * ( msg_pose.y - msg_planned_path.path[1].end.y );
                      length_next_path = cos_on_next_pathend * ( msg_planned_path.path[1].end.x - msg_planned_path.path[0].end.x )
                              + sin_on_next_pathend * ( msg_planned_path.path[1].end.y - msg_planned_path.path[0].end.y );

                      // when the robot is in the next path area
                      if( (x_robot_on_next_pathend > - length_next_path - msg_planned_path.path[1].start_extend) &&
                          (x_robot_on_next_pathend <   msg_planned_path.path[1].end_extend) &&
                          (y_robot_on_next_pathend > - msg_planned_path.path[1].right_width) &&
                          (y_robot_on_next_pathend <   msg_planned_path.path[1].left_width) )
                      {
                          // transit to the next path
                          flgs_transit_next |= flg_transit_next;
                      }
                      // when the robot arrives at the current path-end
                      else if( (robot_on_pathend[0] > - node_config.pathend_margin.value.at(0)) ||
                               (robot_on_pathend[0] > msg_planned_path.path[0].end_extend - (node_config.vehicle_length_back.value.at(0) + node_config.clearance_required.value.at(0))))
                      {
                          // transit to the next path
                          flgs_transit_next |= flg_transit_next;
                          flgs_transit_next |= flg_transit_next_out_of_path;
                      }

                  } // <--- when some paths area


                  // ---> operation that transit to the next path
                  if( flgs_transit_next )
                  {
                      // transit to the next path
                      last_start_waypoint = msg_planned_path.start;

                      msg_planned_path.start = msg_planned_path.path[0].end;
                      msg_planned_path.path.erase( msg_planned_path.path.begin() );
                      trajectory_target.lines.clear();
                      trajectory_actual.lines.clear();

                      // update
                      if ( (signed) msg_planned_path.path.size() > 0 )
                      {
                          QGenericMatrix<4,4,double> coord_tf_matrix;

                          // update the current path coordinate
                          gnd::matrix::coordinate_converter(&coord_tf_matrix,
                              msg_planned_path.path[0].end.x, msg_planned_path.path[0].end.y, 0.0,
                              cos(msg_planned_path.path[0].end.theta), sin(msg_planned_path.path[0].end.theta), 0.0,
                              0.0, 0.0, 1.0);
                          coordinate_tree.set_coordinate( coordinate_id_pathend, &coord_tf_matrix );

                          { // ---> update target trajectory
                              double diff_theta;
                              double y_robot_on_next_pathend;

                              y_robot_on_next_pathend = - sin( msg_planned_path.path[0].end.theta ) * ( msg_pose.x - msg_planned_path.path[0].end.x )
                                      + cos( msg_planned_path.path[0].end.theta ) * ( msg_pose.y - msg_planned_path.path[0].end.y );
                              diff_theta = gnd::rad_normalize(msg_pose.theta - msg_planned_path.path[0].end.theta);


                              // trajectory
                              if( fabs(diff_theta) < gnd_deg2rad(10.0) )
                              {
                                  double shift_path = y_robot_on_next_pathend;
                                  // when robot is out of next path
                                  if( shift_path > msg_planned_path.path[0].left_width - ( node_config.vehicle_width_left.value.at(0) + node_config.clearance_required.value.at(0))  )
                                  {
                                      shift_path = msg_planned_path.path[0].left_width - ( node_config.vehicle_width_left.value.at(0) + node_config.clearance_required.value.at(0));
                                  }
                                  else if( shift_path < - msg_planned_path.path[0].right_width + ( node_config.vehicle_width_left.value.at(0) + node_config.clearance_required.value.at(0)) )
                                  {
                                      shift_path = - msg_planned_path.path[0].right_width + ( node_config.vehicle_width_left.value.at(0) + node_config.clearance_required.value.at(0));
                                  }

                                  traj_tmp.end.x = msg_planned_path.path[0].end.x + sin(msg_planned_path.path[0].end.theta) * shift_path;
                                  traj_tmp.end.y = msg_planned_path.path[0].end.y + cos(msg_planned_path.path[0].end.theta) * shift_path;
                                  traj_tmp.end.theta = msg_planned_path.path[0].end.theta;
                                  trajectory_target.lines.push_back(traj_tmp);
                              }
                              else if( ((y_robot_on_next_pathend > 0) && (diff_theta < 0)) ||
                                       ((y_robot_on_next_pathend < 0) && (diff_theta > 0)) )
                              {
                                  traj_tmp.end.x = msg_planned_path.path[0].end.x;
                                  traj_tmp.end.y = msg_planned_path.path[0].end.y;
                                  traj_tmp.end.theta = msg_planned_path.path[0].end.theta;
                                  trajectory_target.lines.push_back(traj_tmp);
                              }
                              else
                              {
                                  double shift_path = y_robot_on_next_pathend;
                                  // when robot is out of next path
                                  if( shift_path > msg_planned_path.path[0].left_width - ( node_config.vehicle_width_left.value.at(0) + node_config.clearance_required.value.at(0))  )
                                  {
                                      shift_path = msg_planned_path.path[0].left_width - ( node_config.vehicle_width_left.value.at(0) + node_config.clearance_required.value.at(0));
                                  }
                                  else if( shift_path < - msg_planned_path.path[0].right_width + ( node_config.vehicle_width_left.value.at(0) + node_config.clearance_required.value.at(0)) )
                                  {
                                      shift_path = - msg_planned_path.path[0].right_width + ( node_config.vehicle_width_left.value.at(0) + node_config.clearance_required.value.at(0));
                                  }

                                  traj_tmp.end.x = msg_planned_path.path[0].end.x + sin(msg_planned_path.path[0].end.theta) * shift_path;
                                  traj_tmp.end.y = msg_planned_path.path[0].end.y + cos(msg_planned_path.path[0].end.theta) * shift_path;
                                  traj_tmp.end.theta = msg_planned_path.path[0].end.theta;
                                  trajectory_target.lines.push_back(traj_tmp);
                              }

                              // update the current path coordinate
                              gnd::matrix::coordinate_converter(&coord_tf_matrix,
                                    trajectory_target.lines[0].end.x, trajectory_target.lines[0].end.y, 0.0,
                                    cos(trajectory_target.lines[0].end.theta), sin(trajectory_target.lines[0].end.theta), 0.0,
                                    0.0, 0.0, 1.0);
                              coordinate_tree.set_coordinate( coordinate_id_trajectory_target, &coord_tf_matrix );

                          } // <--- update target trajectory

                          //A->B->A, turn round 180 Deg
                          if(last_start_waypoint.name.size() > 0)
                          {
                              if(last_start_waypoint.name == msg_planned_path.path[0].end.name)
                              {
                                turn_round_state = STATE_TURN_BACK;
                              }
                              else
                              {
                                turn_round_state = STATE_TURN_LR;
                                last_vel_dir = prev_velocity;
                              }
                          }

                      }
                      else
                      {
                          // do nothing
                      }
                  } // <--- operation that transit to the next path
              } // <--- path determination

              // ---> trajectory determination
              if( msg_planned_path.path.size() <= 0 )
              {
                  trajectory_actual.lines.clear();
                  trajectory_target.lines.clear();
              }
              else if( !flgs_errorstate )
              {
                  bool flg_reselect_trajectory = false;
                  double shift_from_pathline_to_trajectory;

                  // if robot have no target trajectories
                  // ---> set trajectory
                  if( trajectory_target.lines.size() == 0 )
                  {
                      QGenericMatrix<4,4,double> coord_tf_matrix;
                      // trajectory
                      traj_tmp.end.x = msg_planned_path.path[0].end.x;
                      traj_tmp.end.y = msg_planned_path.path[0].end.y;
                      traj_tmp.end.theta = msg_planned_path.path[0].end.theta;
                      trajectory_target.lines.push_back(traj_tmp);

                      gnd::matrix::coordinate_converter(&coord_tf_matrix,
                          trajectory_target.lines[0].end.x, trajectory_target.lines[0].end.y, 0.0,
                          cos(trajectory_target.lines[0].end.theta), sin(trajectory_target.lines[0].end.theta), 0.0,
                          0.0, 0.0, 1.0);
                      coordinate_tree.set_coordinate( coordinate_id_trajectory_target, &coord_tf_matrix );
                  } // <--- set trajectory

                  { // ---> coordinate transformation of point-cloud (from robot to path-end) and check obstacles on the current target trajectory
                      QGenericMatrix<4,4,double> coord_tf_matrix;
                      QGenericMatrix<4,1,double> point_on_robot;
                      QGenericMatrix<4,1,double> point_on_pathend;
                      point_t point;
                      double search_range_right_from_robot, search_range_left_from_robot;

                      length_current_path = - ( cos(msg_planned_path.path[0].end.theta) * ( msg_planned_path.start.x - msg_planned_path.path[0].end.x )
                          + sin(msg_planned_path.path[0].end.theta) * ( msg_planned_path.start.y - msg_planned_path.path[0].end.y ) );

                      // coordinate convert matrix
                      coordinate_tree.get_convert_matrix( coordinate_id_robot, coordinate_id_pathend, &coord_tf_matrix );
                      shift_from_pathline_to_trajectory = -sin_on_current_pathend * ( trajectory_target.lines[0].end.x - msg_planned_path.path[0].end.x )
                                                         + cos_on_current_pathend * ( trajectory_target.lines[0].end.y - msg_planned_path.path[0].end.y );

                      // ---> 1. extract the point-cloud data within the current path area
                      //      2. coordinate transform from robot to path-end
                      //      3. check obstacles on the current trajectory
                      //      4. check obstacles in front of robot
                      //      5. check trajectories to be able to connect
                      // (for reduction of computing cost, it operates above 5 tasks simultaneously in a scanning loop)
                      eval_trajectory_target_for_velocity_determination.depth = node_config.depth_for_trajectory_selection.value.at(0);
                      eval_trajectory_target_for_velocity_determination.clearance = node_config.clearance_margin.value.at(0);
                      eval_trajectory_target_for_velocity_determination.dir = 0.0;
                      eval_trajectory_target_for_velocity_determination.dist_to_end = (- robot_on_pathend[0]);
                      obstalce_for_motion.depth = node_config.depth_for_trajectory_selection.value.at(0);
                      obstalce_for_motion.clearance_dist = node_config.clearance_margin.value.at(0);
                      search_range_right_from_robot = -DBL_MAX / 2.0;
                      search_range_left_from_robot = DBL_MAX / 2.0;
                      pointcloud_on_pathend.clear();
                      // ---> scanning loop (point cloud topics)
                      for( int j = 0; j < ntopics_pointcloud; j++ )
                      {
                          // ---> scanning loop (points)
                          for( int i = 0; i < (signed)topics_pointcloud[j].msg_pointcloud.points.size(); i++ )
                          {
                              double depth, clearance;
                              // ignore the points showing the robot itself
                              if( (topics_pointcloud[j].msg_pointcloud.points[i].x <   node_config.vehicle_length_front.value.at(0)) &&
                                  (topics_pointcloud[j].msg_pointcloud.points[i].x > - node_config.vehicle_length_back.value.at(0)) &&
                                  (topics_pointcloud[j].msg_pointcloud.points[i].y <   node_config.vehicle_width_left.value.at(0))  &&
                                  (topics_pointcloud[j].msg_pointcloud.points[i].y > - node_config.vehicle_width_right.value.at(0)) )
                              {
                                  continue;
                              }

                              { // ---> check obstacles in front of robot
                                  // obstacles in front of the robot
                                  if( (topics_pointcloud[j].msg_pointcloud.points[i].x >   node_config.vehicle_length_front.value.at(0)) &&
                                      (topics_pointcloud[j].msg_pointcloud.points[i].y <   node_config.vehicle_width_left.value.at(0))  &&
                                      (topics_pointcloud[j].msg_pointcloud.points[i].y > - node_config.vehicle_width_right.value.at(0)) )
                                  {
                                      obstalce_for_motion.depth = obstalce_for_motion.depth < (topics_pointcloud[j].msg_pointcloud.points[i].x - node_config.vehicle_length_front.value.at(0)) ?
                                          obstalce_for_motion.depth : (topics_pointcloud[j].msg_pointcloud.points[i].x - node_config.vehicle_length_front.value.at(0));

                                  }

                                  // clearance of the straight trajectory in front of the robot
                                  if( (topics_pointcloud[j].msg_pointcloud.points[i].x <   node_config.vehicle_length_front.value.at(0)) &&
                                      (topics_pointcloud[j].msg_pointcloud.points[i].x > - node_config.vehicle_length_back.value.at(0)) )
                                  {

                                      double clearance = topics_pointcloud[j].msg_pointcloud.points[i].y < 0 ? ( -topics_pointcloud[j].msg_pointcloud.points[i].y) - node_config.vehicle_width_right.value.at(0) : topics_pointcloud[j].msg_pointcloud.points[i].y - node_config.vehicle_width_left.value.at(0);
                                      obstalce_for_motion.clearance_dist = clearance < obstalce_for_motion.clearance_dist ?
                                          clearance : obstalce_for_motion.clearance_dist;
                                  }
                              } // <--- check obstacles in front of robot

                              { // ---> coordinate transform
                                  point_on_robot(0,0) = topics_pointcloud[j].msg_pointcloud.points[i].x;
                                  point_on_robot(0,1) = topics_pointcloud[j].msg_pointcloud.points[i].y;
                                  point_on_robot(0,2) = topics_pointcloud[j].msg_pointcloud.points[i].z;
                                  point_on_robot(0,3) = 1;
                                  point_on_pathend =  point_on_robot * coord_tf_matrix;
                              } // <--- coordinate transform

                              { // ---> extract the point-cloud data within the current path area
                                  if( (topics_pointcloud[j].msg_pointcloud.points[i].x < node_config.vehicle_length_front.value.at(0) + 5.0 * node_config.max_velocity.value.at(0)) &&
                                      (topics_pointcloud[j].msg_pointcloud.points[i].x > -(node_config.vehicle_length_back.value.at(0) + 5.0 * node_config.max_velocity.value.at(0))) &&
                                      (topics_pointcloud[j].msg_pointcloud.points[i].y < node_config.vehicle_width_left.value.at(0) + 5.0 * node_config.max_velocity.value.at(0)) &&
                                      (topics_pointcloud[j].msg_pointcloud.points[i].y > -(node_config.vehicle_width_left.value.at(0) + 5.0 * node_config.max_velocity.value.at(0))) )
                                  {

                                    // with the exception, remain the points within the range that the robot is able to reach in 5 second
                                    // to do nothing
                                  }
                                  else
                                  {
                                      // remove data out of the current path range
                                      if( point_on_pathend(0,0) >  msg_planned_path.path[0].end_extend )							continue;
                                      if( point_on_pathend(0,0) < -msg_planned_path.path[0].start_extend - length_current_path )	continue;
                                      if( point_on_pathend(0,1) >  msg_planned_path.path[0].left_width )							continue;
                                      if( point_on_pathend(0,1) < -msg_planned_path.path[0].right_width )							continue;
                                  }

                                  point.x = point_on_pathend(0,0);
                                  point.y = point_on_pathend(0,1);

                                  // push back
                                  pointcloud_on_pathend.push_back( point );
                              } // <--- extract the point-cloud data within the current path area


                              { // ---> check obstacles on the current trajectory
                                  // calculate depth and clearance
                                  depth = point_on_pathend(0,0) - (robot_on_pathend[0] + node_config.vehicle_length_front.value.at(0));
                                  if( point_on_pathend(0,1) > shift_from_pathline_to_trajectory + (node_config.vehicle_width_left.value.at(0) + node_config.clearance_required.value.at(0)) )
                                  {
                                      clearance = point_on_pathend(0,1) - (shift_from_pathline_to_trajectory + (node_config.vehicle_width_left.value.at(0) + node_config.clearance_required.value.at(0)));
                                  }
                                  else if(point_on_pathend(0,1) < shift_from_pathline_to_trajectory - (node_config.vehicle_width_right.value.at(0) + node_config.clearance_required.value.at(0)) )
                                  {
                                      clearance = -point_on_pathend(0,1) + (shift_from_pathline_to_trajectory - (node_config.vehicle_width_right.value.at(0) + node_config.clearance_required.value.at(0)));
                                  }
                                  else
                                  {
                                      clearance = 0;
                                  }

                                  // calculate nearest depth and clearance
                                  if( (point_on_pathend(0,0) < msg_planned_path.path[0].end_extend) &&
                                      (point_on_pathend(0,0) > robot_on_pathend[0] - node_config.vehicle_length_back.value.at(0)))
                                  {
                                      // update nearest depth
                                      if( (depth > 0) && (depth < eval_trajectory_target_for_velocity_determination.dist_to_end - node_config.vehicle_length_front.value.at(0)) &&
                                          (point_on_pathend(0,1) < shift_from_pathline_to_trajectory + (node_config.vehicle_width_left.value.at(0) + node_config.clearance_required.value.at(0))) &&
                                          (point_on_pathend(0,1) > shift_from_pathline_to_trajectory - (node_config.vehicle_width_right.value.at(0) + node_config.clearance_required.value.at(0))) )
                                      {
                                          eval_trajectory_target_for_velocity_determination.depth = depth < eval_trajectory_target_for_velocity_determination.depth ? depth : eval_trajectory_target_for_velocity_determination.depth;
                                      }
                                      // update clearance
                                      else if( (depth > 0) &&  (depth < eval_trajectory_target_for_velocity_determination.dist_to_end - node_config.vehicle_length_front.value.at(0)) &&
                                          (depth < node_config.depth_for_slow_down.value.at(0)) )
                                      {
                                          eval_trajectory_target_for_velocity_determination.clearance = clearance < eval_trajectory_target_for_velocity_determination.clearance ? clearance : eval_trajectory_target_for_velocity_determination.clearance;
                                      }
                                  }
                              } // ---> check obstacles on the current trajectory

                              // ---> check trajectories to be able to connect
                              if( (point_on_pathend(0,0) < robot_on_pathend[0] + node_config.vehicle_length_front.value.at(0)) &&
                                  (point_on_pathend(0,0) > robot_on_pathend[0] - node_config.vehicle_length_back.value.at(0))  )
                              {
                                  double r = point_on_pathend(0,1) - robot_on_pathend[1];

                                  if( (r > 0) && (search_range_left_from_robot < r) )
                                  {
                                      search_range_left_from_robot = r;
                                  }
                                  else if( (r < 0) && (search_range_right_from_robot > r) )
                                  {
                                      search_range_right_from_robot = r;
                                  }
                              } // <--- check trajectories to be able to connect
                          } // <--- scanning loop (points)
                      } // <--- scanning loop (point cloud topics)

                      // search range
                      search_range_left = search_range_left_from_robot + robot_on_pathend[1];
                      search_range_left = search_range_left < msg_planned_path.path[0].left_width ? search_range_left : msg_planned_path.path[0].left_width;
                      search_range_right = search_range_right_from_robot + robot_on_pathend[1];
                      search_range_right = search_range_right > -msg_planned_path.path[0].right_width ? search_range_right : -msg_planned_path.path[0].right_width;
                      // add vehicle width and margin
                      search_range_left -= (node_config.vehicle_width_left.value.at(0) + node_config.clearance_required.value.at(0));
                      search_range_right += (node_config.vehicle_width_right.value.at(0) + node_config.clearance_required.value.at(0));

                      // depth
                      eval_trajectory_target_for_velocity_determination.depth = eval_trajectory_target_for_velocity_determination.depth < node_config.depth_for_stop.value.at(0) ? 0 : eval_trajectory_target_for_velocity_determination.depth;
                  }
                  // <--- 1. extract the laser-scanner data within the current path area
                  //      2. coordinate transform from robot to path-end
                  //      3. check obstacles on the current trajectory
                  //      4. check obstacles in front of robot
                  //      5. check trajectories to be able to connect


                  { // ---> determinate to change the target trajectory or not
                      // there is no data representing obstacle
                      if( eval_trajectory_target_for_velocity_determination.depth > node_config.depth_for_slow_down.value.at(0) )
                      {
                          // reset trajectory reselect schedule
                          schedule_reselect_trajecotry = time_current + node_config.period_reselect_trajectory.value.at(0);
                      }
                      // for some time, there are some data representing obstacle
                      if( time_current > schedule_reselect_trajecotry )
                      {
                          // trajectory reselect
                          flg_reselect_trajectory = true;
                      }
                      // exception: no target trajecotry
                      else if( msg_planned_path.path.size() > 0 && trajectory_target.lines.size() <= 0 )
                      {
                          // trajectory reselect
                          flg_reselect_trajectory = true;
                      }

                  } // <--- determinate to change the target trajectory or not

                  // if change the target trajectory
                  // ---> reselect the target trajectory
                  if( flg_reselect_trajectory )
                  {
                      int i_max_eval = -1;
                      double max_evaluation = 0;
                      double n = ( node_config.vehicle_width_right.value.at(0) + node_config.vehicle_width_left.value.at(0) + 2 * node_config.clearance_required.value.at(0) ) / node_config.discretized_interval_of_target_trajectory.value.at(0);

                      { // ---> determine the candidate trajectories
                          int n_trajectories;

                          eval_trajectories.clear();

                          // quantization : transform width into some line trajectories
                          search_range_left = search_range_left < 0 ? - ceil( -search_range_left / node_config.discretized_interval_of_target_trajectory.value.at(0) ) * node_config.discretized_interval_of_target_trajectory.value.at(0)
                              : ceil( search_range_left / node_config.discretized_interval_of_target_trajectory.value.at(0) ) * node_config.discretized_interval_of_target_trajectory.value.at(0) ;
                          search_range_right = search_range_right > 0 ? ceil( search_range_right / node_config.discretized_interval_of_target_trajectory.value.at(0) ) * node_config.discretized_interval_of_target_trajectory.value.at(0)
                              : - ceil( -search_range_right / node_config.discretized_interval_of_target_trajectory.value.at(0) ) * node_config.discretized_interval_of_target_trajectory.value.at(0) ;
                          n_trajectories = ceil( ( (search_range_left - search_range_right) / node_config.discretized_interval_of_target_trajectory.value.at(0) ) + 0.4999 );

                          if( n_trajectories > 0 )
                          {
                              //eval_trajectories.resize(n_trajectories);
                              for( int i = 0; i < (signed)eval_trajectories.size(); i++ )
                              {
                                  // initialize evaluation elements
                                  eval_traj_temp.depth = node_config.depth_for_trajectory_selection.value.at(0);
                                  eval_traj_temp.clearance = node_config.clearance_margin.value.at(0);
                                  eval_traj_temp.dir = 0.0;
                                  eval_traj_temp.dist_to_end =  (- robot_on_pathend[0]);
                                  eval_trajectories.push_back(eval_traj_temp);
                              }
                          }
                      } // <--- determine the candidate trajectories


                      // ---> calculation of the trajectory evaluation elements
                      if( eval_trajectories.size() > 0 )
                      {
                          // ---> depth
                          for( int i = 0; i < (signed)pointcloud_on_pathend.size(); i++ )
                          {
                              int j_begin, j_end;
                              double depth;

                              if( pointcloud_on_pathend[i].x < robot_on_pathend[0] + node_config.vehicle_length_front.value.at(0) ) continue;
                              if( pointcloud_on_pathend[i].y < search_range_right - node_config.vehicle_width_right.value.at(0) - node_config.clearance_required.value.at(0) )	continue;
                              if( pointcloud_on_pathend[i].y > search_range_left + node_config.vehicle_width_left.value.at(0) + node_config.clearance_required.value.at(0) )		continue;

                              j_begin = ((pointcloud_on_pathend[i].y - node_config.vehicle_width_right.value.at(0) - node_config.clearance_required.value.at(0)) - (search_range_right)) / node_config.discretized_interval_of_target_trajectory.value.at(0);
                              j_begin = j_begin >= 0 ? j_begin : 0;
                              j_end = ((pointcloud_on_pathend[i].y + node_config.vehicle_width_left.value.at(0) + node_config.clearance_required.value.at(0)) - (search_range_right)) / node_config.discretized_interval_of_target_trajectory.value.at(0);
                              depth = pointcloud_on_pathend[i].x - (robot_on_pathend[0] + node_config.vehicle_length_front.value.at(0) + node_config.clearance_required.value.at(0)) - node_config.depth_for_slow_down.value.at(0);

                              // evaluate depth
                              for( int j = j_begin; j < j_end && j < (signed)eval_trajectories.size(); j++ )
                              {
                                  if( (depth < eval_trajectories[j].dist_to_end - (node_config.vehicle_length_front.value.at(0) + node_config.clearance_required.value.at(0))) &&
                                      (eval_trajectories[j].depth > depth) )
                                  {
                                      if( depth < node_config.depth_for_stop.value.at(0) )
                                      {
                                          eval_trajectories[j].depth = 0;
                                          eval_trajectories[j].clearance = 0;
                                      }
                                      else
                                      {
                                          eval_trajectories[j].depth = depth;
                                      }
                                  }
                              }
                          } // <--- depth

                          // ---> clearance
                          for( int i = 0; i < (signed)pointcloud_on_pathend.size(); i++ )
                          {
                              int j_begin, j_end;
                              double depth;

                              if( pointcloud_on_pathend[i].x < robot_on_pathend[0] + node_config.vehicle_length_front.value.at(0) + node_config.clearance_required.value.at(0) ) continue;
                              if( pointcloud_on_pathend[i].y < search_range_right - node_config.vehicle_width_right.value.at(0) - node_config.clearance_required.value.at(0) - node_config.clearance_margin.value.at(0))	continue;
                              if( pointcloud_on_pathend[i].y > search_range_left + node_config.vehicle_width_left.value.at(0) + node_config.clearance_required.value.at(0) - node_config.clearance_margin.value.at(0) )		continue;

                              j_begin = ((pointcloud_on_pathend[i].y - node_config.vehicle_width_right.value.at(0) - node_config.clearance_required.value.at(0) - node_config.clearance_margin.value.at(0)) - (search_range_right)) / node_config.discretized_interval_of_target_trajectory.value.at(0);
                              j_begin = j_begin >= 0 ? j_begin : 0;
                              j_end = ((pointcloud_on_pathend[i].y + node_config.vehicle_width_left.value.at(0) + node_config.clearance_required.value.at(0) + node_config.clearance_margin.value.at(0)) - (search_range_right)) / node_config.discretized_interval_of_target_trajectory.value.at(0);
                              depth = pointcloud_on_pathend[i].x - (robot_on_pathend[0] + node_config.vehicle_length_front.value.at(0) + node_config.clearance_required.value.at(0));

                              // evaluate clearance
                              for( int j = j_begin; j < j_end && j < (signed)eval_trajectories.size(); j++ )
                              {
                                  if( (eval_trajectories[j].depth > 0) && (depth < eval_trajectories[j].depth) &&
                                      (depth < (eval_trajectories[j].dist_to_end + node_config.vehicle_length_front.value.at(0))) )
                                  {
                                      double y_trajectory = search_range_right + j * node_config.discretized_interval_of_target_trajectory.value.at(0);
                                      double clearance_left = pointcloud_on_pathend[i].y - (y_trajectory  - node_config.vehicle_width_right.value.at(0) - node_config.clearance_required.value.at(0));
                                      double clearance_right = -(pointcloud_on_pathend[i].y - (y_trajectory  - node_config.vehicle_width_right.value.at(0) - node_config.clearance_required.value.at(0)));
                                      double clearance = clearance_left > clearance_right ? clearance_left : clearance_right;

                                      clearance = clearance < node_config.clearance_margin.value.at(0) ? clearance : node_config.clearance_margin.value.at(0);
                                      eval_trajectories[j].clearance = eval_trajectories[j].clearance < clearance ? eval_trajectories[j].clearance : clearance;
                                  }
                              }
                          } // <--- clearance
                      } // <--- calculation of trajectory evaluation elements

                      // ---> select trajectory
                      if( eval_trajectories.size() > 0 )
                      {
                          double eval_depth, eval_clearance, eval_dist, eval;

                          i_max_eval = -1;
                          max_evaluation = 0;

                          for( int i = 0; i < (signed)eval_trajectories.size(); i++ )
                          {
                              if( eval_trajectories[i].depth <= 0 )
                              {
                                  eval = 0;
                              }
                              else
                              {
                                  eval_depth = eval_trajectories[i].depth / (node_config.depth_for_trajectory_selection.value.at(0) - node_config.depth_for_slow_down.value.at(0));
                                  eval_clearance = eval_trajectories[i].clearance / node_config.clearance_margin.value.at(0);
                                  eval_dist = exp( - gnd_square( 1.0 / 1.5 *  fabs(search_range_right + node_config.discretized_interval_of_target_trajectory.value.at(0) * i - robot_on_pathend[1]) ) );
                                  eval = node_config.target_trajectory_weight_depth.value.at(0) * eval_depth
                                    + node_config.target_trajecotry_weight_clearance.value.at(0) * eval_clearance
                                    + node_config.target_trajecotry_weight_distance.value.at(0) * eval_dist;
                              }
                              if( (max_evaluation < eval) && (eval > 0)  )
                              {
                                  max_evaluation = eval;
                                  i_max_eval = i;
                              }
                          }
                      } // <--- select trajectory

                      // ---> set new target trajectory
                      if( i_max_eval >= 0 )
                      {
                          double d = search_range_right + node_config.discretized_interval_of_target_trajectory.value.at(0) * i_max_eval;
                          QGenericMatrix<4,4,double> coord_tf_matrix;

                          trajectory_target.lines.clear();
                          traj_tmp.end.theta = msg_planned_path.path[0].end.theta;
                          traj_tmp.end.x = msg_planned_path.path[0].end.x - sin(trajectory_target.lines[0].end.theta) * d;
                          traj_tmp.end.y = msg_planned_path.path[0].end.y + cos(trajectory_target.lines[0].end.theta) * d;
                          trajectory_target.lines.push_back(traj_tmp);
                          schedule_reselect_trajecotry = time_current + node_config.period_reselect_trajectory.value.at(0);

                          eval_trajectory_target_for_velocity_determination.depth = eval_trajectories[i_max_eval].depth;
                          eval_trajectory_target_for_velocity_determination.clearance = eval_trajectories[i_max_eval].clearance;
                          eval_trajectory_target_for_velocity_determination.dir = eval_trajectories[i_max_eval].dir;

                          // coordinate
                          gnd::matrix::coordinate_converter(&coord_tf_matrix,
                              trajectory_target.lines[0].end.x, trajectory_target.lines[0].end.y, 0.0,
                              cos(trajectory_target.lines[0].end.theta), sin(trajectory_target.lines[0].end.theta), 0.0,
                              0.0, 0.0, 1.0);
                          coordinate_tree.set_coordinate( coordinate_id_trajectory_target, &coord_tf_matrix );

                          // reset actual trajecotry
                          trajectory_actual.lines.clear();
                      } // ---> set new target trajectory

                  } // <--- reselect the target trajectory

                  // ---> determine the actual trajectory
                  if( trajectory_target.lines.size() <= 0 )
                  {
                      // no target
                      trajectory_actual.lines.clear();
                  }
                  else
                  {
                      QGenericMatrix<4,4,double> coord_tf_matrix_global2target;
                      QGenericMatrix<4,4,double> coord_tf_matrix_target2pathend;
                      QGenericMatrix<4,1,double> pose_on_global;
                      QGenericMatrix<4,1,double> pose_on_trajectory_target;
                      QGenericMatrix<4,1,double> pose_on_pathend;

                      coordinate_tree.get_convert_matrix(coordinate_id_global, coordinate_id_trajectory_target, &coord_tf_matrix_global2target);
                      coordinate_tree.get_convert_matrix(coordinate_id_trajectory_target, coordinate_id_pathend, &coord_tf_matrix_target2pathend);
                      { // ---> calculate the distance to target trajectory
                          pose_on_global(0,0) = msg_pose.x;
                          pose_on_global(0,1) = msg_pose.y;
                          pose_on_global(0,2) = msg_pose.theta;
                          pose_on_global(0,3) = 1;

                          pose_on_trajectory_target = pose_on_global * coord_tf_matrix_global2target;
                          pose_on_pathend = pose_on_trajectory_target * coord_tf_matrix_target2pathend;
                      } // <--- calculate the distance to target trajectory

                      // ---> calculate the actual trajectory
                      trajectory_actual.lines.clear();
                      // ---> if robot is not near the target trajectory, actual trajectory is planned an intermediate trajectory
                      if( fabs(pose_on_trajectory_target(0,1)) > node_config.distance_threshold_intermediate.value.at(0) )
                      {
                          QQueue<gnd::path::trajectory_unit_t> trajectories_searched_on_pathend;
                          gnd::path::trajectory_unit_t traj_ws;
                          int size;
                          double search_dir_size;
                          double search_range_start;
                          double search_range_end;
                          int i_max_eval;
                          double max_eval;
                          double max_dist_to_end = 0;

                          // calculate the search range
                          search_dir_size = pose_on_trajectory_target(0,1) > 0 ? node_config.mediate_trajectory_search_angular_size.value.at(0) : -node_config.mediate_trajectory_search_angular_size.value.at(0);

                          search_range_start = pose_on_trajectory_target(0,1) > 0 ? -M_PI / 2.0 : M_PI / 2.0;
                          search_range_end = atan2( -pose_on_trajectory_target(0,1), node_config.mediate_trajectory_search_range.value.at(0) );
                          if( search_range_end >  node_config.mediate_trajectory_search_angular_range.value.at(0) ) search_range_end =   node_config.mediate_trajectory_search_angular_range.value.at(0);
                          if( search_range_end < -node_config.mediate_trajectory_search_angular_range.value.at(0) ) search_range_end = - node_config.mediate_trajectory_search_angular_range.value.at(0);

                          size = (search_range_end - search_range_start) / search_dir_size + 1;
                          search_range_end = search_range_start + (size - 1) * search_dir_size;
                          if(search_dir_size < 0)
                          {
                              double ws;
                              search_dir_size = -search_dir_size;
                              ws = search_range_end;
                              search_range_end = search_range_start;
                              search_range_start = ws;
                          }

                          // ---> create the candidate trajectories
                          eval_trajectories.clear();
                          for( int i = 0; i < size; i++ )
                          {
                              double dir_on_target;
                              QGenericMatrix<4,1,double> trajectory_searched_on_target;
                              QGenericMatrix<4,1,double> trajectory_searched_on_pathend;

                              dir_on_target = gnd::rad_normalize( search_range_start + i * search_dir_size );
                              if( fabs(dir_on_target - M_PI / 2.0) > M_PI * 1.0 / 180.0 )
                              {
                                  trajectory_searched_on_target(0,0) = pose_on_trajectory_target(0,0) - pose_on_trajectory_target(0,1) / tan(dir_on_target);
                              }
                              else
                              {
                                  trajectory_searched_on_target(0,0) = pose_on_trajectory_target(0,0);
                              }
                              trajectory_searched_on_target(0,1) = 0;
                              trajectory_searched_on_target(0,2) = 0;
                              trajectory_searched_on_target(0,3) = 1;

                              trajectory_searched_on_pathend = trajectory_searched_on_target * coord_tf_matrix_target2pathend;

                              // calculate on global coordinate
                              traj_ws.curvature = 0;
                              traj_ws.end.theta = gnd::rad_normalize( dir_on_target + (msg_planned_path.path[0].end.theta - trajectory_target.lines[0].end.theta) );
                              traj_ws.end.x = trajectory_searched_on_pathend(0,0);
                              traj_ws.end.y = trajectory_searched_on_pathend(0,1);
                              trajectories_searched_on_pathend.push_back(traj_ws);

                              // initialize
                              eval_traj_temp.depth = node_config.depth_for_trajectory_selection.value.at(0);
                              eval_traj_temp.dist_to_end = sqrt( gnd_square(trajectory_searched_on_pathend(0,0) - pose_on_pathend(0,0)) + gnd_square(trajectory_searched_on_pathend(0,1) - pose_on_pathend(0,1)) );
                              eval_traj_temp.dir = trajectories_searched_on_pathend[i].end.theta;
                              eval_traj_temp.clearance = node_config.clearance_margin.value.at(0);
                              eval_trajectories.push_back(eval_traj_temp);

                              max_dist_to_end = max_dist_to_end > eval_trajectories[i].dist_to_end ? max_dist_to_end : eval_trajectories[i].dist_to_end;
                          } // <--- create the candidate trajectories

                          // ---> evaluate the clearance
                          for( int i = 0; i < (signed)pointcloud_on_pathend.size(); i++ )
                          {
                              double r_point, theta_point;
                              double search_range_right;
                              double search_range_left;
                              int j_begin, j_end;

                              r_point = sqrt( gnd_square(pointcloud_on_pathend[i].x - pose_on_pathend(0,0)) + gnd_square(pointcloud_on_pathend[i].y - pose_on_pathend(0,1)) );
                              if( r_point > max_dist_to_end + node_config.vehicle_length_front.value.at(0) + node_config.clearance_required.value.at(0) ) continue;

                              theta_point = atan2(pointcloud_on_pathend[i].y - pose_on_pathend(0,1), pointcloud_on_pathend[i].x - pose_on_pathend(0,1));
                              // search range right
                              if( r_point == 0 || r_point <= node_config.vehicle_width_right.value.at(0) + node_config.clearance_required.value.at(0) + node_config.clearance_margin.value.at(0) )
                              {
                                  search_range_right = theta_point - M_PI / 2.0;
                              }
                              else
                              {
                                  search_range_right = theta_point - asin( (node_config.vehicle_width_right.value.at(0) + node_config.clearance_required.value.at(0) + node_config.clearance_margin.value.at(0)) / r_point );
                              }
                              if( search_range_start < search_range_right )
                              {
                                  j_begin = (search_range_right - search_range_start) / search_dir_size;
                              }
                              else
                              {
                                  j_begin = 0;
                              }

                              // search range left
                              if( (r_point == 0) || (r_point <= node_config.vehicle_width_left.value.at(0) + node_config.clearance_required.value.at(0) + node_config.clearance_margin.value.at(0)) )
                              {
                                  search_range_left = theta_point + M_PI / 2.0;
                              }
                              else
                              {
                                  search_range_left = theta_point + asin( (node_config.vehicle_width_left.value.at(0) + node_config.clearance_required.value.at(0) + node_config.clearance_margin.value.at(0)) / r_point );
                              }
                              if( search_range_end > search_range_left )
                              {
                                  j_end = (search_range_left - search_range_start) / search_dir_size + 1;
                              }
                              else
                              {
                                  j_end = trajectories_searched_on_pathend.size();
                              }

                              for( int j = j_begin; j < j_end; j++ )
                              {
                                  double dir;
                                  double clearance;

                                  if( r_point > eval_trajectories[j].dist_to_end ) continue;

                                  dir = search_range_start + j * search_dir_size;
                                  clearance = r_point * sin( theta_point - dir );

                                  if( clearance > 0 )
                                  {
                                      clearance = clearance - node_config.vehicle_width_left.value.at(0) - node_config.clearance_required.value.at(0);
                                  }
                                  else
                                  {
                                      clearance = -(clearance + node_config.vehicle_width_right.value.at(0) + node_config.clearance_required.value.at(0));
                                  }
                                  clearance = clearance > 0 ? clearance : 0;

                                  // update
                                  if( eval_trajectories[j].clearance > clearance )
                                  {
                                      eval_trajectories[j].clearance = clearance;
                                  }
                                  if( clearance <= 0 )
                                  {
                                      eval_trajectories[j].clearance = 0;
                                      eval_trajectories[j].depth = 0;
                                  }
                              }

                          } // <--- evaluate the clearance

                          i_max_eval = -1;
                          max_eval = 0;
                          for( int i = 0; i < (signed)eval_trajectories.size(); i++ )
                          {
                              double eval;
                              double eval_clearance;
                              double eval_dir;
                              double dir;
                              dir = search_range_start + i * search_dir_size;

                              // not have required clearance
                              if( eval_trajectories[i].clearance <= 0 ) continue;
                              if( eval_trajectories[i].depth <= 0 ) continue;

                              eval_clearance = eval_trajectories[i].clearance / node_config.clearance_margin.value.at(0);
                              eval_dir = cos(eval_trajectories[i].dir);
                              eval_dir = eval_dir > 0 ? eval_dir : 0;
                              eval = node_config.mediate_trajectory_weight_clearance.value.at(0) * eval_clearance + node_config.mediate_trajectory_weight_direction.value.at(0) * eval_dir;

                              if( max_eval < eval )
                              {
                                  max_eval = eval;
                                  i_max_eval = i;
                              }
                          }

                          // ---> set actual trajectory
                          if( i_max_eval >= 0 )
                          {
                              QGenericMatrix<4,4,double> coord_tf_matrix_pathend2global;
                              QGenericMatrix<4,1,double> trajectory_on_pathend;
                              QGenericMatrix<4,1,double> trajectory_on_global;

                              trajectory_on_pathend(0,0) = trajectories_searched_on_pathend[i_max_eval].end.x;
                              trajectory_on_pathend(0,1) = trajectories_searched_on_pathend[i_max_eval].end.y;
                              trajectory_on_pathend(0,2) = 0;
                              trajectory_on_pathend(0,3) = 1;

                              coordinate_tree.get_convert_matrix(coordinate_id_pathend, coordinate_id_global, &coord_tf_matrix_pathend2global);
                              trajectory_on_global = trajectory_on_pathend * coord_tf_matrix_pathend2global;

                              traj_tmp.end.x = trajectory_on_global(0,0);
                              traj_tmp.end.y = trajectory_on_global(0,1);
                              traj_tmp.end.theta = trajectories_searched_on_pathend[i_max_eval].end.theta + msg_planned_path.path[0].end.theta;
                              traj_tmp.curvature = trajectories_searched_on_pathend[i_max_eval].curvature;
                              trajectory_actual.lines.push_back(traj_tmp);

                              eval_trajectory_actual_for_velocity_determination = eval_trajectories[i_max_eval];
                          } // <--- set actual trajectory
                      } // <--- if robot is not near the target trajectory, the actual trajectory is planned an intermediate trajectory
                      // ---> if robot is near the target trajectory, the actual trajectory is the target trajectory
                      else
                      {
                          // the actual trajectory is the target trajectory
                          trajectory_actual.lines.clear();
                          trajectory_actual.lines.push_back(trajectory_target.lines.at(0));

                          eval_trajectory_actual_for_velocity_determination = eval_trajectory_target_for_velocity_determination;
                      } // <--- if robot is near the target trajectory, the actual trajectory is the target trajectory


                      // ---> update coordinate
                      if( trajectory_actual.lines.size() > 0 )
                      {
                          QGenericMatrix<4,4,double> coord_tf_matrix_actual2global;

                          gnd::matrix::coordinate_converter(&coord_tf_matrix_actual2global,
                              trajectory_actual.lines[0].end.x, trajectory_actual.lines[0].end.y, 0,
                              cos(trajectory_actual.lines[0].end.theta), sin(trajectory_actual.lines[0].end.theta), 0,
                              0, 0, 1);
                          coordinate_tree.set_coordinate(coordinate_id_trajectory_actual, &coord_tf_matrix_actual2global);
                      } // <--- update coordinate

                  } // <---  determine the actual trajectory
              } // <--- trajectory determination

              // ---> velocity determination
              if( trajectory_actual.lines.size() <= 0 )
              {
                  velocity = 0;
              }
              else
              {
                  double vel_dir;
                  double vel_trajectory_depth;
                  double vel_motion_depth;

                  { // ---> velocity designed based on direction
                      double dir_diff;

                      dir_diff = gnd::rad_normalize(trajectory_actual.lines[0].end.theta - msg_pose.theta);

                      switch (turn_round_state)
                      {
                          case STATE_TURN_BACK:
                              vel_dir = 0;
                              if((last_start_waypoint.name.size() > 0) && (fabs(dir_diff) < 0.15))
                              {
                                  turn_round_state = STATE_RUN_LINE;
                              }
                              break;
                          case STATE_RUN_LINE:
                              vel_dir = node_config.max_velocity.value.at(0) * cos(dir_diff);
                              break;
                          case STATE_TURN_LR:
                              vel_dir = node_config.max_velocity.value.at(0) * cos(dir_diff) + last_vel_dir;
                              if((last_start_waypoint.name.size() > 0) && (fabs(dir_diff) < 0.15))
                              {
                                  turn_round_state = STATE_RUN_LINE;
                              }
                              break;
                          case STATE_NONE:
                          default:
                              vel_dir = node_config.max_velocity.value.at(0) * cos(dir_diff);
                              break;
                      }

                      vel_dir = vel_dir < 0 ? 0 : vel_dir;

                      vel_dir = vel_dir < prev_velocity + node_config.acceleration.value.at(0) ? vel_dir : prev_velocity + node_config.acceleration.value.at(0);
                      vel_dir = vel_dir > prev_velocity - node_config.deceleration.value.at(0) ? vel_dir : prev_velocity - node_config.deceleration.value.at(0);
                  } // <--- velocity designed based on direction

                  { // ---> velocity designed based on depth
                      if( eval_trajectory_actual_for_velocity_determination.depth >= node_config.depth_for_slow_down.value.at(0) )
                      {
                          vel_trajectory_depth = node_config.max_velocity.value.at(0);
                      }
                      else if( eval_trajectory_actual_for_velocity_determination.depth > node_config.depth_for_stop.value.at(0) )
                      {
                          vel_trajectory_depth = node_config.max_velocity.value.at(0) *
                              ( (eval_trajectory_actual_for_velocity_determination.depth - node_config.depth_for_stop.value.at(0)) / (node_config.depth_for_slow_down.value.at(0) - node_config.depth_for_stop.value.at(0)));
                      }
                      else
                      {
                          vel_trajectory_depth = 0;
                      }
                  } // <--- velocity designed based on depth


                  { // ---> velocity designed based on depth
                      if( obstalce_for_motion.depth - node_config.clearance_required.value.at(0) >= node_config.depth_for_slow_down.value.at(0) )
                      {
                          vel_motion_depth = node_config.max_velocity.value.at(0);
                      }
                      else
                      {
                          vel_motion_depth = node_config.max_velocity.value.at(0) *
                              ( (obstalce_for_motion.depth - node_config.clearance_required.value.at(0)) / (node_config.depth_for_slow_down.value.at(0)));
                      }
                  } // <--- velocity designed based on depth

                  velocity = vel_trajectory_depth < vel_motion_depth ? vel_trajectory_depth : vel_motion_depth;
                          velocity = velocity < vel_dir ? velocity : vel_dir;
              } // <--- velocity determination



              // send control command to follow the trajectory
              if( flgs_errorstate )
              {
                  // send stop command
                  amr_stop();
                  prev_velocity = 0.0;
              }
              // no trajectory case
              else if( trajectory_actual.lines.size() <= 0 )
              {
                  // send stop command
                  amr_stop();
                  prev_velocity = 0.0;
              }
              else if( trajectory_actual.lines.size() > 0 )
              {
                  // send trajectory and velocity limitation
                  // stop on line command
                  amr_set_vel(std::min(velocity, node_config.max_velocity.value.at(0)),node_config.max_angular_velocity.value.at(0));
                  amr_set_trajectory_pose(trajectory_actual.lines[0].end.x,trajectory_actual.lines[0].end.y,trajectory_actual.lines[0].end.theta);
                  prev_velocity = velocity;
              }
              else
              {
                  // impossible case
                  // set default command (stop command)
                  prev_velocity = 0.0;
              }

              // publish status
              if ( msg_planned_path.path.size() <= 0 )
              {
                  msg_vehicle_status.status = msg_vehicle_status.VEHICLE_STATE_IDLE;
              }
              else
              {
                  if ( (eval_trajectory_actual_for_velocity_determination.depth < node_config.depth_for_stop.value.at(0)) && (std::fabs(velocity) < DBL_EPSILON) )
                  {
                      msg_vehicle_status.status = msg_vehicle_status.VEHICLE_STATE_STOP_OBSTACLE;
                  }
                  else
                  {
                      msg_vehicle_status.status = msg_vehicle_status.VEHICLE_STATE_RUN;
                  }
              }
              pub_vehicle_stauts.publish(msg_vehicle_status);

              // schedule next planning
              schedule_planning = gnd_loop_next( time_current, time_start, node_config.period_planning.value.at(0) );
            } // <---  planning


            // ---> publish planed trajectory
            if( time_current > schedule_publish_planning )
            {
                // planned path
                if( msg_planned_path.path.size() > 0 )
                {
                    pub_planned_path.publish(msg_planned_path);
                }
                // target trajectory
                if( trajectory_target.lines.size() > 0 )
                {
                    msg_trajectory_target.end.x = trajectory_target.lines[0].end.x;
                    msg_trajectory_target.end.y = trajectory_target.lines[0].end.y;
                    msg_trajectory_target.end.theta = trajectory_target.lines[0].end.theta;
                    msg_trajectory_target.curvature = trajectory_target.lines[0].curvature;
                    // publish
                    pub_trajectory_target.publish(msg_trajectory_target);
                }
                // actual trajectory
                if( trajectory_actual.lines.size() > 0 )
                {
                    msg_trajectory_actual.end.x = trajectory_actual.lines[0].end.x;
                    msg_trajectory_actual.end.y = trajectory_actual.lines[0].end.y;
                    msg_trajectory_actual.end.theta = trajectory_actual.lines[0].end.theta;
                    msg_trajectory_actual.curvature = trajectory_actual.lines[0].curvature;
                    // publish
                    pub_trajectory_actual.publish(msg_trajectory_actual);
                }

                // next schedule
                schedule_publish_planning = gnd_loop_next(time_current, time_start, node_config.period_publish_planning.value.at(0));
            } // ---> publish planed trajectory
        } // <--- main loop
    } // <--- operate


    { // ---> finalize
        ROS_INFO("---------- finalize ----------");

        { // ---> order to stop
            amr_stop();
        } // <--- order to stop
        ROS_INFO("  ... %s finished", node_config.node_name.value.at(0).toStdString().c_str());
    } // <--- finalize

	return 0;
}
