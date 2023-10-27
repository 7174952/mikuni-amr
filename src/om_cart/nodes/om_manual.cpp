#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>
#include "om_cart/om_cart_cmd.h"
#include "om_cart.h"

ros::Publisher cart_cmd_pub;

double VEL_LINE_MAX = 300; // mm/s
const uint16_t STEP_CHANGE = 100; //mm/s
const double VEL_THETA_MAX = M_PI_4; // pi/4 /s

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    om_cart::om_cart_cmd cmd_msg;
    static uint cnt = 0;

    //buttons[1]: [A]
    //buttons[2]: [Y]
    if(joy->buttons[1] == 1)
    {
        ROS_INFO("Entered Emergency Protect Mode!\r\nPress [Y] back to Normal Mode.");
        cmd_msg.type = OPERATE;
        cmd_msg.size = 1;
        cmd_msg.data[0] = S_OFF;
        cart_cmd_pub.publish(cmd_msg);
        return;

    }
    if(joy->buttons[2] == 1)
    {
        ROS_INFO("Normal Mode!");
        cmd_msg.type = OPERATE;
        cmd_msg.size = 1;
        cmd_msg.data[0] = S_ON;
        cart_cmd_pub.publish(cmd_msg);
        return;

    }
    if((int)joy->axes[7] != 0)
    {
        VEL_LINE_MAX += (joy->axes[7] * STEP_CHANGE);
        if(VEL_LINE_MAX > 800) VEL_LINE_MAX = 800;
        if(VEL_LINE_MAX < 200) VEL_LINE_MAX = 200;
        return;
    }

    if((abs((int)joy->axes[0] * 100) > 0) || (abs((int)joy->axes[4] * 100) > 0))
    {
        //set cart speed
        cmd_msg.type = SET_SPEED;
        cmd_msg.size = 2;
        // cmd_msg.data[0] = joy->axes[4] * VEL_LINE_MAX * UNIT_RATIO;    //line speed
        // cmd_msg.data[1] = joy->axes[0] * VEL_THETA_MAX * UNIT_RATIO;   //theta speed
        cmd_msg.data[0] = (joy->axes[4] * VEL_LINE_MAX)/1000;    //line speed
        cmd_msg.data[1] = (joy->axes[0] * VEL_THETA_MAX) ;   //theta speed

        cart_cmd_pub.publish(cmd_msg);
        cnt = 0;
    }
    else if(cnt < 3)
    {
        cmd_msg.type = SET_SPEED;
        cmd_msg.size = 2;
        cmd_msg.data[0] = 0;  //line speed
        cmd_msg.data[1] = 0;  //theta speed
        cart_cmd_pub.publish(cmd_msg);
        cnt += 1;
    }



}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "om_manual");
  ros::NodeHandle nh;

  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy",10, joyCallback);

  cart_cmd_pub = nh.advertise<om_cart::om_cart_cmd>("cart_cmd",10);

  ros::spin();

  return 0;
}
