#include "usb_io.h"

#include "ros/ros.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "usb_io_node");
    Usb_io usbio;

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        usbio.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
