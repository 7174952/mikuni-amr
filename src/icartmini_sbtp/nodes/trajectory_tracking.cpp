/*
 * trajectory_tracking.h
 *
 *  Created on: 2023/12/01
 *      Author: ryu
 *       Brief: change trajectory to velocity and sent to amr
 */
#include <cstdio>
#include <cmath>
#include <vector>

/* ros */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Pose2D.h"

#include "trajectory_tracking.h"
#include "om_cart/om_cart_state.h"
#include "icartmini_sbtp/target_move_angle.h"


Trajectory_tracking::Trajectory_tracking()
{
    amr_param.PARAM_CONTROL_CYCLE = 0.020;
    amr_param.PARAM_MAX_CENTRIFUGAL_ACC = 2.45;
    amr_param.PARAM_L_C1 = 0.01; //0.01;
    amr_param.PARAM_L_K1 = 800;  //800;
    amr_param.PARAM_L_K2 = 10;   //300;
    amr_param.PARAM_L_K3 = 25;   //200
    amr_param.PARAM_L_DIST = 0.6;

    //init circle trajectory
    amr_info.path_circle.xc = 0.0;
    amr_info.path_circle.yc = 0.0;
    amr_info.path_circle.radius = 0.0;
    run_state = STATE_RUN_LINE; //STATE_RUN_DEFAULT;

    amr_info.odom_pose.x = 0;
    amr_info.odom_pose.y = 0;
    amr_info.odom_pose.theta = 0;
    amr_info.odom_pose.time = 0;

    amr_info.actual_pose.x = 0;
    amr_info.actual_pose.y = 0;
    amr_info.actual_pose.theta = 0;
    amr_info.actual_pose.time = 0;

    //accel default
    amr_info.dv = 1.5;
    amr_info.dw = 0.5;
    amr_info.vref_smooth = 0;
    amr_info.wref_smooth = 0;
    amr_info.vref = 0;
    amr_info.wref = 0;
    amr_info.stopAngle = 0.0;
    amr_info.isTracking_on = false;

    msg_cmd_vel.linear.x = 0;
    msg_cmd_vel.linear.y = 0;
    msg_cmd_vel.linear.z = 0;
    msg_cmd_vel.angular.x = 0;
    msg_cmd_vel.angular.y = 0;
    msg_cmd_vel.angular.z = 0;

    before_time = 0;
    now_time = 0;
    amr_running = false;
    cir_end_1_theta = 0;
    cir_end_2_theta = 0;

    sub_max_vel = nh.subscribe<std_msgs::Float64MultiArray>("amr_set_vel",100,&Trajectory_tracking::maxVelCallback,this);
    sub_trajectory_pose = nh.subscribe<geometry_msgs::Pose2D>("amr_trajectory_pose",100,&Trajectory_tracking::trajectoryPoseCallback,this);
    sub_odom_pose = nh.subscribe<gnd_msgs::msg_pose2d_stamped>("pose_particle_localizer",100,&Trajectory_tracking::odomPoseCallback, this);
    subsc_vehicle_status = nh.subscribe<gnd_msgs::msg_vehicle_status>("icartmini_sbtp/vehicle_status", 10, &Trajectory_tracking::vehicleStatusCallback,this);
    subsc_odom_vel = nh.subscribe<gnd_msgs::msg_velocity2d_with_covariance_stamped>("/vehicle_vel",10,&Trajectory_tracking::odomVelCallback,this);
    sub_planned_path = nh.subscribe<gnd_msgs::msg_path_area_and_speed_limited>("/planned_path",10,&Trajectory_tracking::plannedPathCallback,this);

    sub_cart_status = nh.subscribe<om_cart::om_cart_state>("cart_status",100, &Trajectory_tracking::cartStatusCallback,this);
    for(int i = 0; i < CART_STATUS_LEN; i++)
    {
        cart_status.data[i] = 0;
    }

    pub_vehicle_status = nh.advertise<gnd_msgs::msg_vehicle_status>("vehicle_status",100);
    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    srv_moveAngle = nh.advertiseService("trajectory_tracking/set_move_angle", &Trajectory_tracking::setMoveAngle, this);
}

void Trajectory_tracking::cartStatusCallback(const om_cart::om_cart_state::ConstPtr& status)
{
    for(int i = 0; i < CART_STATUS_LEN; i++)
    {
        cart_status.data[i] = status->data[i];
    }

}

bool Trajectory_tracking::setMoveAngle(icartmini_sbtp::target_move_angle::Request &req, icartmini_sbtp::target_move_angle::Response &res)
{
    amr_info.stopAngle = req.moveTheta;

    res.result = true;

    return true;

}

void Trajectory_tracking::maxVelCallback(const std_msgs::Float64MultiArray::ConstPtr& max_vel)
{
    amr_info.v_max = max_vel->data[0];
    amr_info.w_max = max_vel->data[1];

    if((fabs(amr_info.v_max) > 0.001) || (fabs(amr_info.w_max) > 0.001))
    {
        amr_running = true;
    }
}

void Trajectory_tracking::trajectoryPoseCallback(const geometry_msgs::Pose2D::ConstPtr& trajectory_pose)
{
    amr_info.actual_pose.x = trajectory_pose->x;
    amr_info.actual_pose.y = trajectory_pose->y;
    amr_info.actual_pose.theta = trajectory_pose->theta;
    amr_info.actual_pose.time = ros::Time::now().toSec();

    amr_info.theta_last = trajectory_pose->theta;

}

void Trajectory_tracking::odomPoseCallback(const gnd_msgs::msg_pose2d_stamped::ConstPtr& odom)
{
    amr_info.odom_pose.x = odom->x;
    amr_info.odom_pose.y = odom->y;
    amr_info.odom_pose.theta = odom->theta;
    amr_info.odom_pose.time = ros::Time::now().toSec();
}

void Trajectory_tracking::odomVelCallback(const gnd_msgs::msg_velocity2d_with_covariance_stamped::ConstPtr& vel)
{
    amr_info.vref_smooth = vel->vel_x;
    amr_info.wref_smooth = vel->vel_ang;

    if((fabs(amr_info.v_max) < 0.001) && (fabs(amr_info.w_max) < 0.001) && (fabs(amr_info.vref_smooth) < 0.001) && (fabs(amr_info.wref_smooth) < 0.001))
    {
        amr_running = false;
    }
}

void Trajectory_tracking::vehicleStatusCallback(const gnd_msgs::msg_vehicle_status::ConstPtr& vehicle_status)
{
    gnd_msgs::msg_vehicle_status status_tmp;

    if(vehicle_status->status == gnd_msgs::msg_vehicle_status::VEHICLE_STATE_IDLE)
    {
        status_tmp.status = (amr_info.isTracking_on == true) ? gnd_msgs::msg_vehicle_status::VEHICLE_STATE_RUN : gnd_msgs::msg_vehicle_status::VEHICLE_STATE_IDLE;
    }
    else
    {
        status_tmp.status = vehicle_status->status;
    }

    pub_vehicle_status.publish(status_tmp);
}

void Trajectory_tracking::plannedPathCallback(const gnd_msgs::msg_path_area_and_speed_limited::ConstPtr& path)
{
    planned_path.start = path->start;
    planned_path.path.resize(path->path.size());
    planned_path.path = path->path;
}

void Trajectory_tracking::run()
{
    now_time            = get_time();
    amr_info.control_dt = now_time - before_time;
    before_time         = now_time;
    double delt_dist;

    if(amr_running)
    {
        switch (run_state)
        {
        case STATE_RUN_LINE:
            if(planned_path.path.size() > 1)
            {
                delt_dist = (amr_info.odom_pose.x - planned_path.path[0].end.x) * cos(planned_path.path[0].end.theta) + (amr_info.odom_pose.y - planned_path.path[0].end.y) * sin(planned_path.path[0].end.theta);
                cir_end_1_theta = planned_path.path[0].end.theta;
                cir_end_2_theta = planned_path.path[1].end.theta;

                if(delt_dist < -amr_info.l0)
                {                        
                    stop_line(&amr_info.odom_pose, &amr_info);

                }
                else
                {
                    /*------------------------------------------
                     * determine tracking round center
                     * r0=l0*cot((theta_end - theta_start)/2)
                     * xc=xR - r0*sin(theta_R)
                     * yc=yR + ro*cos(theta_R)
                    -------------------------------------------*/
                    amr_info.path_circle.radius = amr_info.l0 * (1/tan((planned_path.path[1].end.theta -  planned_path.path[0].end.theta)/2));
                    amr_info.path_circle.xc = amr_info.odom_pose.x - amr_info.path_circle.radius * sin(trans_q(amr_info.odom_pose.theta));
                    amr_info.path_circle.yc = amr_info.odom_pose.y + amr_info.path_circle.radius * cos(trans_q(amr_info.odom_pose.theta));
                    run_state = STATE_RUN_CIRCLE;
                }

            }
            else
            {
                stop_line(&amr_info.odom_pose, &amr_info);
            }

            msg_cmd_vel.linear.x = amr_info.vref;
            msg_cmd_vel.angular.z = amr_info.wref;

            pub_cmd_vel.publish(msg_cmd_vel);
            amr_info.isTracking_on = true;
            break;
        case STATE_RUN_CIRCLE:
            circle_follow(&amr_info.odom_pose, &amr_info);
            if(  (  (cir_end_2_theta > cir_end_1_theta) && (trans_q(amr_info.odom_pose.theta) > cir_end_2_theta - 0.08))  //turn left
              || (  (cir_end_2_theta < cir_end_1_theta) && (trans_q(amr_info.odom_pose.theta) < cir_end_2_theta + 0.08))) //turn right

            { //tracking round complete
                run_state = STATE_RUN_LINE;
            }

            msg_cmd_vel.linear.x = amr_info.vref;
            msg_cmd_vel.angular.z = amr_info.wref;

            pub_cmd_vel.publish(msg_cmd_vel);
            amr_info.isTracking_on = true;

            break;
        case STATE_RUN_DEFAULT:
        default:
            run_state = STATE_RUN_LINE;
        }
    }
    else
    {
        double angle_tmp;
        double data[] = {0.0, (M_PI/180)*5};
        double resData[2];
        angle_tmp = amr_info.theta_last +  amr_info.stopAngle;
        data[0] = angle_tmp;

        //set angle after stopped
        if(near_ang_com(data,resData,&amr_info) == 0 && (amr_info.isTracking_on == true))
        {
            amr_info.actual_pose.theta = angle_tmp;
            amr_info.w_max = M_PI/9;

            spin(&amr_info.odom_pose,&amr_info);
            msg_cmd_vel.linear.x = amr_info.vref;
            msg_cmd_vel.angular.z = amr_info.wref;
            pub_cmd_vel.publish(msg_cmd_vel);

        }
        else
        {
            amr_info.isTracking_on = false;
            msg_cmd_vel.linear.x = 0;
            msg_cmd_vel.angular.z = 0;
            pub_cmd_vel.publish(msg_cmd_vel);

        }
    }

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"trajectory_tracking");
    Trajectory_tracking trajectory_tracking;

    ROS_INFO("Trajectory tracking node started!");

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        trajectory_tracking.run();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
