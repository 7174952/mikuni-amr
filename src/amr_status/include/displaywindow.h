#ifndef DISPLAYWINDOW_H
#define DISPLAYWINDOW_H

#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsWidget>
#include <QGraphicsItem>
#include <QPushButton>
#include <QGraphicsProxyWidget>
#include <QGraphicsLinearLayout>
#include <QObject>
#include <QPoint>
#include <QDebug>
#include <QString>
#include <QPainter>
#include <QLabel>

#include "gnd_msgs/msg_pose2d_stamped.h"
#include "sensor_msgs/PointCloud.h"
#include "rosdatathread.h"
#include "displayitems.h"

class DisplayWindow : public QObject
{
    Q_OBJECT
public:
    explicit DisplayWindow(QObject *parent = nullptr);
    ~DisplayWindow();

signals:

public slots:
    void showRobotData(sensor_msgs::PointCloud pointCloudMsg, gnd_msgs::msg_pose2d_stamped poseMsg);
    void showRobotStatus(int status);
    void showRobotParticle(gnd_particle_localizer::msg_particles_pose2d_stamped particles);

private:
    RosDataThread *rosThread;
    QGraphicsScene scene;
    QLabel *lPose;
    QLabel *lStatus;
    QPushButton *pButtonExit;
    QGraphicsProxyWidget *labelPose;
    QGraphicsProxyWidget *labelStatus;
    QGraphicsProxyWidget *pushButtonExit;
    QGraphicsLinearLayout *layout;
    QGraphicsWidget *form;
    QGraphicsView *view;
    PointCloudItem *myPointCloud;
    MapItem *mapItem;
    RobotIcon *robot;

};

#endif // DISPLAYWINDOW_H
