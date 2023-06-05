#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TeleopWhill
{
public:
    TeleopWhill();
    void joyPublish();
    bool getEmerButton();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    ros:: NodeHandle nh;
    int linear_x;
    int linear_y;
    double l_scale;
    bool bEmerButtonEn;
    ros::Publisher vel_pub;
    ros::Subscriber joy_sub;
    sensor_msgs::Joy joy_data;

};

TeleopWhill::TeleopWhill(): linear_x(0),linear_y(4), l_scale(100),bEmerButtonEn(false)
{
    size_t i;
    nh.param("axis_linear_x", linear_x,linear_x);
    nh.param("axis_linear_y",linear_y,linear_y);
    nh.param("scale_linear",l_scale,l_scale);

    vel_pub = nh.advertise<sensor_msgs::Joy>("/whill/controller/joy",10);
    joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy",10,&TeleopWhill::joyCallback,this);

    //init
    joy_data.axes.resize(9);
    for (i = 0; i < 9; i++)
    {
        joy_data.axes[i] = 0.0;
    }
    joy_data.buttons.resize(12);
    for (i = 0; i < 12; i++)
    {
        joy_data.buttons[i] = 0.0;
    }
    bEmerButtonEn = false;
}

void TeleopWhill::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
//    joy_data.axes[0] = joy->axes[linear_y]*0.7;
//    joy_data.axes[1] = joy->axes[linear_x]*0.5;
    joy_data.axes[0] = joy->axes[linear_y];
    joy_data.axes[1] = joy->axes[linear_x]*0.7;


    //mergency button
    joy_data.buttons[0] = joy->buttons[1]; //A
    joy_data.buttons[1] = joy->buttons[2]; //Y

    //buttons[1]: [A]
    //buttons[2]: [Y]
    if(joy->buttons[1] == 1)
    {
        if(bEmerButtonEn == false)
        {
            ROS_INFO("Entered Emergency Protect Mode!\r\nPress [Y] back to Normal Mode.");
            bEmerButtonEn = true;
       }
    }
    else if(joy->buttons[2] == 1)
    {
        if(bEmerButtonEn == true)
        {
            ROS_INFO("Normal Mode!");
            bEmerButtonEn = false;
        }
    }

}

void TeleopWhill::joyPublish()
{
    vel_pub.publish(joy_data);

}

bool TeleopWhill::getEmerButton()
{
    return bEmerButtonEn;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"hdk_icartmini_controller_manual");
    TeleopWhill teleop_whill;
    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        if(teleop_whill.getEmerButton() == false)
        {
            teleop_whill.joyPublish();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
