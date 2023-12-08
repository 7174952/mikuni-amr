#ifndef __TRAJECTORY_TRACKING_H__
#define __TRAJECTORY_TRACKING_H__

/* ros */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Joy.h"

#include "icartmini_sbtp/target_move_angle.h"
#include "gnd_msgs/msg_pose2d_stamped.h"
#include "gnd_msgs/msg_vehicle_status.h"
#include "gnd_msgs/msg_velocity2d_with_covariance_stamped.h"
#include "gnd_msgs/msg_path_area_and_speed_limited.h"


#include "om_cart/om_cart_state.h"
#include "om_cart.h"

#define SIGN(x) ((x < 0) ? -1 : 1)

class Trajectory_tracking
{
public:
    typedef struct
    {
        double x;
        double y;
        double theta;
        double time;
    } Space2D;

    typedef struct
    {
        double xc;
        double yc;
        double radius;
    }Circle2D;

    enum {
        STATE_RUN_DEFAULT = 0,
        STATE_RUN_LINE,
        STATE_RUN_CIRCLE,
    };

    typedef struct
    {
        Space2D actual_pose;
        Space2D odom_pose;
        Circle2D path_circle;
        const double l0 = 1.8;//2.15;//2.3 //meter
        double v_max;
        double w_max;
        double vref;
        double wref;
        double vref_smooth;
        double wref_smooth;
        double control_dt;
        double dv;
        double dw;
        double theta_last;
        double stopAngle;
        std::string targetName;
        bool isTracking_on;

    } Amr_info;
    
    typedef struct
    {
        // 車体コントロールパラメータ
        double PARAM_CONTROL_CYCLE;
        double PARAM_MAX_CENTRIFUGAL_ACC;

        // 軌跡追従パラメータ
        double PARAM_L_C1;
        uint16_t PARAM_L_K1;
        uint16_t PARAM_L_K2;
        uint16_t PARAM_L_K3;
        double PARAM_L_DIST;

    } Parameter;

    Trajectory_tracking();
    void run();

    //control_motion.cpp
    double trans_q(double theta);
    double circle_follow(Space2D* p_odm, Amr_info* p_amr_info);
    double line_follow(Space2D* p_odm, Amr_info* p_amr_info);
    double spin(Space2D* p_odm, Amr_info* p_amr_info);
    double orient(Space2D* p_odm, Amr_info* p_amr_info);
    double dist_pos(Space2D* p_odm, Amr_info* p_amr_info);
    int stop_line(Space2D* p_odm, Amr_info* p_amr_info);
    double get_time(void);

    //command_get.cpp
    void get_pos_com(double *data, double *resdata, Amr_info* p_amr_info);
    void get_vref_com(double *data, double *resdata, Amr_info* p_amr_info);
    void get_vel_com(double *data, double *resdata, Amr_info* p_amr_info);
    int near_pos_com(double *data, double *resdata, Amr_info* p_amr_info);
    int near_ang_com(double *data, double *resdata, Amr_info* p_amr_info);
    int over_line_com(double *data, double *resdata, Amr_info* p_amr_info);

    //waypoint info
    bool get_waypoint_status();
    void set_target_point();

private:
    ros:: NodeHandle nh;
    ros::Subscriber sub_trajectory_pose;
    ros::Subscriber sub_max_vel;
    ros::Subscriber sub_odom_pose;
    ros::Publisher  pub_cmd_vel;   

    Amr_info amr_info;
    Parameter amr_param;

    double before_time;
    double now_time;

    bool amr_running;
    geometry_msgs::Twist msg_cmd_vel;
    gnd_msgs::msg_path_area_and_speed_limited planned_path;
    uint8_t run_state;
    std::string start_waypoint_name;
    double cir_end_1_theta;
    double cir_end_2_theta;

    ros::Subscriber	subsc_controller;
    ros::Subscriber subsc_vehicle_status;
    ros::Publisher  pub_vehicle_status;
    ros::ServiceServer srv_moveAngle;
    ros::ServiceClient client_setTarget;
    icartmini_sbtp::target_move_angle srv_dest;


    ros::Subscriber subsc_odom_vel;
    ros::Subscriber sub_planned_path;

    ros::Subscriber sub_cart_status;
    Cart_Status_Info cart_status;
    const int CART_STATUS_LEN = 30;

    void maxVelCallback(const std_msgs::Float64MultiArray::ConstPtr& max_vel);
    void trajectoryPoseCallback(const geometry_msgs::Pose2D::ConstPtr& trajectory_pose);
    void odomPoseCallback(const gnd_msgs::msg_pose2d_stamped::ConstPtr& odom);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    bool setMoveAngle(icartmini_sbtp::target_move_angle::Request &req, icartmini_sbtp::target_move_angle::Response &res);

    void vehicleStatusCallback(const gnd_msgs::msg_vehicle_status::ConstPtr& vehicle_status);
    void odomVelCallback(const gnd_msgs::msg_velocity2d_with_covariance_stamped::ConstPtr& vel);
    void plannedPathCallback(const gnd_msgs::msg_path_area_and_speed_limited::ConstPtr& path);
    void cartStatusCallback(const om_cart::om_cart_state::ConstPtr& status);

    //control_motion.cpp 
    double regurator(double d, double q, double r, double v_max, double w_max, Amr_info* p_amr_info);
    double timeoptimal_servo(double err, double vel_max, double vel, double acc);
    double timeoptimal_servo2(double err, double vel_max, double vel, double acc, double vel_end);

};

#endif
