#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <queue>

#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_state.h"
#include "gnd_msgs/msg_pose2d_stamped.h"
#include "gnd_msgs/msg_velocity2d_with_covariance_stamped.h"
#include "om_cart/om_cart_state.h"
#include "om_cart/om_cart_cmd.h"
#include "om_data.h"
#include "om_cart.h"
#include "std_msgs/Bool.h"

gnd_msgs::msg_velocity2d_with_covariance_stamped    msg_velocity; //publisher
const double velocity_standard_error_ratio[] = {0.050000, 0.050000, 0.050000};
const double velocity_standard_error_random_element[] = {0.010000, 0.010000, 0.010000};
/* グローバル変数 */
int gState_driver = 0;  /* 通信可能フラグ変数(0:通信可能,1:通信中) */
double last_publish_time;

//Odometry odom;
Om_data om_data;
Pose2D pose;
gnd_msgs::msg_pose2d_stamped msg_pose;
// double timeCorrected;

ros::Publisher cart_velocity_pub;
ros::Publisher om_query_pub;
ros::Publisher cart_status_pub;
// ros::Publisher pose_pub;

std::queue<om_cart::om_cart_cmd> cmd_msg_buf;

// long double confineRadian(long double rad);
void om_state_callback(const om_modbus_master::om_state::ConstPtr& msg);
void om_resp_callback(const om_modbus_master::om_response::ConstPtr& msg);
void drive_cmd_vel_callback(const sensor_msgs::JointState::ConstPtr& msg);
void drive_cmd_joy_callback(const sensor_msgs::Joy::ConstPtr& msg);
void wait();
void update();

void cart_auto_drive_cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd)
{
    om_cart::om_cart_cmd cmd_tmp;
    cmd_tmp.type = SET_SPEED;
    cmd_tmp.size = 2;
    cmd_tmp.data[0] = cmd->linear.x;
    cmd_tmp.data[1] = cmd->angular.z;

    cmd_msg_buf.push(cmd_tmp);

}

void cart_s_on_callback(const std_msgs::Bool::ConstPtr& state)
{
    om_cart::om_cart_cmd cmd_tmp;
    cmd_tmp.type = OPERATE;
    cmd_tmp.size = 1;
    cmd_tmp.data[0] = (state->data == false) ? S_OFF : S_ON;

    cmd_msg_buf.push(cmd_tmp);
    
}

void om_state_callback(const om_modbus_master::om_state::ConstPtr& msg)
{
    gState_driver = msg->state_driver;

}

void om_resp_callback(const om_modbus_master::om_response::ConstPtr& msg)
{
    om_cart::om_cart_state cart_status_msg;
    Cart_Status_Info cart_status_info;
    for(int i=0; i < CART_STATUS_INFO_SIZE; i++)
    {
        cart_status_info.data[i] = 0;
    }

    double vel_left, vel_right, vel_line, vel_theta;

    switch( msg->func_code)
    {
        case FC_READ: //Cart status
            {
                Cart_Resp_Info resp_info;

                for(int8_t i = 0; i < CART_RESP_INFO_ITEM_SIZE; i++)
                {
                    resp_info.data[i] = msg->data[i];
                }

                vel_left =  (double)resp_info.Motor_Status.left.vel / UNIT_RATIO;
                vel_right = -(double)resp_info.Motor_Status.right.vel / UNIT_RATIO;
                vel_line = ((vel_left + vel_right) / 2)/1000;
                vel_theta = (vel_right - vel_left) / CART_TREAD;

                cart_status_info.cart_status.vel_line = vel_line;
                cart_status_info.cart_status.vel_theta = vel_theta;
                cart_status_info.cart_status.vel_left = vel_left;
                cart_status_info.cart_status.vel_right = vel_right;
                cart_status_info.cart_status.alm_code_L = resp_info.Motor_Status.left.alm_code;
                cart_status_info.cart_status.alm_code_R = resp_info.Motor_Status.right.alm_code;
                cart_status_info.cart_status.main_power_volt_L = resp_info.Motor_Status.left.main_volt;
                cart_status_info.cart_status.main_power_volt_R = resp_info.Motor_Status.right.main_volt;
                cart_status_info.cart_status.main_power_curr_L = resp_info.Motor_Status.left.main_curr;
                cart_status_info.cart_status.main_power_curr_R = resp_info.Motor_Status.right.main_curr;
                cart_status_info.cart_status.motor_temp_L = resp_info.Motor_Status.left.motor_temp;
                cart_status_info.cart_status.motor_temp_R = resp_info.Motor_Status.right.motor_temp;
                cart_status_info.cart_status.driver_temp_L = resp_info.Motor_Status.left.driver_temp;
                cart_status_info.cart_status.driver_temp_R = resp_info.Motor_Status.right.driver_temp;
                cart_status_info.cart_status.emergen_stop = ((resp_info.Motor_Status.left.driver_IO_OUTPUT >> 5) & 0x00000001)
                                                          + ((resp_info.Motor_Status.right.driver_IO_OUTPUT >> 5) & 0x00000001);               
                cart_status_msg.type = CART_STATUS;
                cart_status_msg.size = CART_STATUS_INFO_SIZE;

                for(int8_t i = 0; i < CART_STATUS_INFO_SIZE; i++)
                {
                    cart_status_msg.data[i] = cart_status_info.data[i];
                }
                cart_status_pub.publish(cart_status_msg);

                //make and send velocity
                ros::Time currentTime = ros::Time::now();
                
                msg_velocity.header.stamp = currentTime;
                msg_velocity.header.seq++;
                msg_velocity.header.frame_id = "";
                msg_velocity.vel_x = vel_line;
                msg_velocity.vel_y = 0;
                msg_velocity.vel_ang = vel_theta;
                msg_velocity.measuring_period = currentTime.toSec() - last_publish_time;
                msg_velocity.covariance[0] = std::pow( fabs(msg_velocity.vel_x) * velocity_standard_error_ratio[0] + velocity_standard_error_random_element[0] ,2);
                msg_velocity.covariance[1] = msg_velocity.covariance[3] = 0.0;
                msg_velocity.covariance[2] = msg_velocity.covariance[6] = 0.0;
                msg_velocity.covariance[4] = std::pow( fabs(msg_velocity.vel_y) * velocity_standard_error_ratio[1] + velocity_standard_error_random_element[1] ,2);
                msg_velocity.covariance[5] = msg_velocity.covariance[7] = 0.0;
                msg_velocity.covariance[8] = std::pow( fabs(msg_velocity.vel_ang) * velocity_standard_error_ratio[2] + velocity_standard_error_random_element[2] ,2);
                cart_velocity_pub.publish(msg_velocity);
                //update timeStep
                last_publish_time = currentTime.toSec();

            }
            break;
        case FC_WRITE:
            cart_status_msg.type = COMM_RESULT;
            cart_status_msg.size = 2;
            cart_status_msg.data[0] = 1;
            cart_status_msg.data[1] = msg->func_code;
            cart_status_pub.publish(cart_status_msg);
            break;
        case FC_READ_WRITE:

            break;
        default:
            break;
    }

}

void cart_set_cmd_callback(const om_cart::om_cart_cmd::ConstPtr& msg)
{
    cmd_msg_buf.push(*msg);
}

//---------------------------------------------------------------------------
//   処理待ちサービス関数
//
//@details	規定時間後(30ms)、通信可能になるまでウェイトがかかるサービス
//----------------------------------------------------------------------------
void wait()
{
    ros::Duration(0.03).sleep();
    ros::spinOnce();

    /* ドライバの通信が終了するまでループ */
    while(gState_driver == 1)
    {
        ros::spinOnce();
    }
}

void update()
{
    if(cmd_msg_buf.size() > 0)
    {
        om_cart::om_cart_cmd msg = cmd_msg_buf.front();

        //check
        switch (msg.type)
        {
            case OPERATE:
                switch((uint)msg.data[0])
                {
                    case S_OFF:
                        om_data.drive_wheel_end(CART_ID, &om_query_pub);
                        break;
                    case S_ON:
                        om_data.drive_wheel_begin(CART_ID, &om_query_pub);
                        break;
                    case ALM_RST:
                    default:
                        om_data.reset_alarm(WHEEL_LEFT_ID, 0,&om_query_pub);
                        wait();
                        om_data.reset_alarm(WHEEL_LEFT_ID, 1,&om_query_pub);
                        wait();
                        om_data.reset_alarm(WHEEL_LEFT_ID, 0,&om_query_pub);
                        wait();
                        om_data.reset_alarm(WHEEL_RIGHT_ID, 0,&om_query_pub);
                        wait();
                        om_data.reset_alarm(WHEEL_RIGHT_ID, 1,&om_query_pub);
                        wait();
                        om_data.reset_alarm(WHEEL_RIGHT_ID, 0,&om_query_pub);
                        break;
                }

                break;
            case SET_SPEED: //update motor speed
                Cart_Cmd_Info cmd_info;

                for(uint8_t i = 0; i < msg.size; i++)
                {
                    cmd_info.data[i] = msg.data[i];
                }

                om_data.drive_cart_cmd(CART_ID, cmd_info.set_speed.vel_line*100000, cmd_info.set_speed.vel_theta*100, &om_query_pub);
                wait();
                om_data.update_share_state(CART_ID, &om_query_pub);
                
                break;
            case SET_MOTOR_PARAM: //TBD

                break;
            case SET_CART_PARAM: //TBD

                break;
            case UN_USED:
            default: //Do nothing

                break;
        }
        cmd_msg_buf.pop();
    }
    else
    {
        om_data.update_share_state(CART_ID, &om_query_pub);
    }

    wait();

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "om_cart");
    ros::NodeHandle nh;

    //Subscriber
    ros::Subscriber om_state_sub = nh.subscribe("om_state1", 100, &om_state_callback);
    ros::Subscriber om_resp_sub = nh.subscribe("om_response1", 100, &om_resp_callback);
    ros::Subscriber cart_set_cmd_sub = nh.subscribe("cart_cmd", 1, &cart_set_cmd_callback);
    //note: can be changed in new 
    ros::Subscriber cart_auto_drive_cmd_sub = nh.subscribe("cmd_vel",1,&cart_auto_drive_cmd_callback);
    ros::Subscriber cart_s_on_sub = nh.subscribe("cart_s_on",1,&cart_s_on_callback);

    //Publisher
    cart_status_pub = nh.advertise<om_cart::om_cart_state>("cart_status",100);

    om_query_pub = nh.advertise<om_modbus_master::om_query>("om_query1",1);
    cart_velocity_pub = nh.advertise<gnd_msgs::msg_velocity2d_with_covariance_stamped>("vehicle_vel",100);

    pose.x = pose.y = pose.theta = 0.0;

    //init velocity publisher
    double initTime = ros::Time::now().toSec();
    msg_velocity.header.stamp.fromSec(initTime);
    msg_velocity.header.seq = 0;
    msg_velocity.header.frame_id = "";
    msg_velocity.vel_x = 0;
    msg_velocity.vel_y = 0;
    msg_velocity.vel_ang = 0;
    msg_velocity.covariance[0] = std::pow(0.010000,2);
    msg_velocity.covariance[1] = msg_velocity.covariance[3] = 0.0;
    msg_velocity.covariance[2] = msg_velocity.covariance[6] = 0.0;
    msg_velocity.covariance[4] = std::pow( 0.010000,2);
    msg_velocity.covariance[5] = msg_velocity.covariance[7] = 0.0;
    msg_velocity.covariance[8] = std::pow( 1.000000 * (M_PI / 180.0) ,2);
    last_publish_time = initTime;

    ros::Duration(1.0).sleep();

    //config all share register
    ROS_INFO("Init to config Share register!");

    om_data.set_share_ID(om_data.slave_1_id, 1, &om_query_pub);
    wait();
    om_data.set_share_ID(om_data.slave_2_id, 2, &om_query_pub);
    wait();
    om_data.set_share_net_id(om_data.slave_1_id, &om_query_pub);
    wait();
    om_data.set_share_net_id(om_data.slave_2_id, &om_query_pub);
    wait();

    //start to run motor
    ROS_INFO("Start to run motor!");
    om_data.drive_wheel_begin(om_data.global_id, &om_query_pub);
    wait();

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    //stop motor
    ROS_INFO("Stop Motor!");
//    om_data.drive_wheel_end(om_data.global_id, &om_query_pub);
//    wait();

    ROS_INFO("End and exit!");

}
