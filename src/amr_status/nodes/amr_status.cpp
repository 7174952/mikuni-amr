#include <QApplication>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "displaywindow.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "amr_status");
    QApplication app(argc,argv);
    qRegisterMetaType<sensor_msgs::PointCloud>();
    qRegisterMetaType<gnd_msgs::msg_pose2d_stamped>();
    qRegisterMetaType<gnd_particle_localizer::msg_particles_pose2d_stamped>();
    qRegisterMetaType<QPointF>();

    DisplayWindow w;

    return app.exec();

}
