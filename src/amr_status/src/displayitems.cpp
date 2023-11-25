#include "displayitems.h"

PointCloudItem::PointCloudItem()
{
    brushColor = Qt::green;
    width = 1;
    height = 1;
    initPos = {0,0};
    newPointPos.channels.clear();
    newPointPos.points.clear();

    offsetHeight = 0;
}

PointCloudItem::PointCloudItem(const QPointF &initPos, double maxRangeWidth, double maxRangeHeight)
{
    brushColor = Qt::green;
    this->width = maxRangeWidth;
    this->height = maxRangeHeight;
    this->initPos = initPos;

    newPointPos.channels.clear();
    newPointPos.points.clear();
    offsetHeight = maxRangeHeight - this->initPos.y()*10;
}

QRectF PointCloudItem::boundingRect() const
{
    double penWidth = 1;
    return QRectF(0 - penWidth /2, 0 - penWidth /2, width + penWidth, height + penWidth);
}

void PointCloudItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    painter->setPen(Qt::green);

    for(int i=0; i<newPointPos.points.size();i++)
    {
        painter->drawPoint(initPos.x()*10 + newPointPos.points[i].x * 10, offsetHeight - newPointPos.points[i].y * 10);
    }
}

MapItem::MapItem()
{
    if(!image.load("/home/mikuni/catkin_ws/src/amr_ros/maps/map-image8.bmp"))
    {
        qDebug() << "load map Error";
    }
}

QRectF MapItem::boundingRect() const
{
    qreal penWidth = 1;
    return QRectF(0 - penWidth /2, 0 - penWidth /2, image.width() + penWidth, image.height() + penWidth);
}

void MapItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    painter->setPen(Qt::black);
    painter->drawImage(QPoint(0,0),image);
}

RobotIcon::RobotIcon()
{
    currentPos.rx() = 0;
    currentPos.ry() = 0;
    currentTheta = 0;
    deltHeight = 0;
    rangeWidth = 0;
    rangeHeight = 0;

    robotImage.load("/home/mikuni/catkin_ws/src/amr_status/icon/arrow_3d_16x16.png");
    robotImageParticle.load("/home/mikuni/catkin_ws/src/amr_status/icon/arrow_3d_16x16_particle.png");
}

RobotIcon::RobotIcon(const QPointF &initPos, double initTheta,double maxWidth, double maxHeight)
{
    this->initPos = initPos;
    this->currentTheta = initTheta;
    this->deltHeight = maxHeight - this->initPos.y()*10;
    rangeWidth = maxWidth;
    rangeHeight = maxHeight;

    robotImage.load("/home/mikuni/catkin_ws/src/amr_status/icon/arrow_3d_16x16.png");
    robotImageParticle.load("/home/mikuni/catkin_ws/src/amr_status/icon/arrow_3d_16x16_particle.png");

}

QRectF RobotIcon::boundingRect() const
{
    qreal penWidth = 1;
    return QRectF(0 - penWidth /2, 0 - penWidth /2, rangeWidth + penWidth, rangeHeight + penWidth);
}

void RobotIcon::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    painter->setPen(Qt::transparent);

    painter->save();
    painter->translate(initPos.x()*10 + currentPos.x()*10, deltHeight - currentPos.y()*10);
    painter->rotate(currentTheta* (-180 / M_PI));
    painter->drawImage((-1)*robotImage.width()/2, (-1)*robotImage.height()/2, robotImage);
    painter->restore();

    for(geometry_msgs::Pose2D _pose:particles.poses)
    {
        painter->save();
        painter->translate(initPos.x()*10 + _pose.x*10, deltHeight - _pose.y*10);
        painter->rotate(_pose.theta * (-180 / M_PI));
//        painter->drawImage((-1)*robotImageParticle.width()/2, (-1)*robotImageParticle.height()/2, robotImageParticle);
        painter->drawImage((-1)*robotImage.width()/2, (-1)*robotImage.height()/2, robotImage);
        painter->restore();
    }

}
