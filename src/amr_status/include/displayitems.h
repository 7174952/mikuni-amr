#ifndef DISPLAYITEMS_H
#define DISPLAYITEMS_H

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
#include <QPixmap>
#include <QPainter>
#include <QLabel>
#include <QtMath>

#include "gnd_msgs/msg_pose2d_stamped.h"
#include "sensor_msgs/PointCloud.h"
#include "rosdatathread.h"

class PointCloudItem : public QGraphicsItem
{
public:
    PointCloudItem();
    PointCloudItem(const QPointF &initPos, double maxRangeWidth, double maxRangeHeight);
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    inline void setColor(const QColor &color) { brushColor = color; }
    inline void setPointsPosition(sensor_msgs::PointCloud& msg) {newPointPos = msg;}

private:
    QColor brushColor;
    double width;
    double height;
    sensor_msgs::PointCloud newPointPos;
    QPointF initPos;
    double offsetHeight;

};

class MapItem : public QGraphicsItem
{
public:
    MapItem();
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    inline int imageWidth() {return image.width();}
    inline int imageHeight(){return image.height();}

private:
    QImage image;

};

class RobotIcon : public QGraphicsItem
{
public:
    RobotIcon();
    RobotIcon(const QPointF &initPos, double initTheta,double maxWidth,double maxHeight);
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    inline void setPose(const QPointF &newPos, double newTheta){currentPos = newPos; currentTheta = newTheta ;}
    inline void setParticles(const gnd_particle_localizer::msg_particles_pose2d_stamped &newParticles){ particles = newParticles;}

private:
    QImage robotImage;
    QImage robotImageParticle;
    QPointF currentPos;
    double currentTheta;
    double rangeWidth;
    double rangeHeight;
    double deltHeight;
    QPointF initPos;
    gnd_particle_localizer::msg_particles_pose2d_stamped particles;

};

#endif // DISPLAYITEMS_H
