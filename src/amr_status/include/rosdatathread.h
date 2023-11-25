#ifndef ROSDATATHREAD_H
#define ROSDATATHREAD_H

#include <QThread>
#include <QDebug>
#include <QVector>
#include <QMetaType>
#include <QPointF>
#include <QGenericMatrix>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "gnd_msgs/msg_pose2d_stamped.h"
#include "gnd_msgs/msg_vehicle_status.h"
#include "gnd_particle_localizer/msg_particles_pose2d_stamped.h"
#include "gnd_rosutil.h"
#include "gnd-matrix-coordinate.h"
#include "gnd-coord-tree.h"


class RosDataThread : public QThread
{
    Q_OBJECT

public:
    explicit RosDataThread(QObject *parent = nullptr);
    ~RosDataThread();

public:
    void run() override;

signals:
    void robotDataReceived(sensor_msgs::PointCloud, gnd_msgs::msg_pose2d_stamped);
    void statusReceived(int);
    void particlesReceived(gnd_particle_localizer::msg_particles_pose2d_stamped);

public slots:

private:
    void poseCallback(const gnd_msgs::msg_pose2d_stamped::ConstPtr& msg);
    void pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
    void statusCallback(const gnd_msgs::msg_vehicle_status::ConstPtr& msg);
    void particleCallback(const gnd_particle_localizer::msg_particles_pose2d_stamped::ConstPtr& msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber pose_sub;
    ros::Subscriber pointCloud_sub;
    ros::Subscriber status_sub;
    ros::Subscriber particle_sub;

    const uint16_t MAX_SIZE = 1000;
    gnd::data_buff<gnd_msgs::msg_pose2d_stamped> pose_buff;
    gnd_msgs::msg_pose2d_stamped poseMsg;
    gnd::coord_tree  coordinate_tree;
    int coordid_gl;
    int coordid_map;
    int coordid_rbt;

};

Q_DECLARE_METATYPE(sensor_msgs::PointCloud)
Q_DECLARE_METATYPE(gnd_msgs::msg_pose2d_stamped)
Q_DECLARE_METATYPE(gnd_particle_localizer::msg_particles_pose2d_stamped)
Q_DECLARE_METATYPE(QPointF)

#endif // ROSDATATHREAD_H
