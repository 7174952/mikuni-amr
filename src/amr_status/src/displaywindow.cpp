#include "displaywindow.h"

DisplayWindow::DisplayWindow(QObject *parent)
    : QObject{parent}
{
    rosThread = new RosDataThread();

    lPose = new QLabel("x:            y:                theta:            ");
    lStatus = new QLabel("status:               ");
    pButtonExit = new QPushButton("Exit");
    connect(rosThread,&RosDataThread::robotDataReceived,this,&DisplayWindow::showRobotData);
    connect(rosThread,&RosDataThread::statusReceived, this,&DisplayWindow::showRobotStatus);
    connect(rosThread,&RosDataThread::particlesReceived,this,&DisplayWindow::showRobotParticle);

    labelPose = scene.addWidget(lPose);
    labelStatus = scene.addWidget(lStatus);
    pushButtonExit = scene.addWidget(pButtonExit);
    layout = new QGraphicsLinearLayout;
    layout->addItem(labelPose);
    layout->addItem(labelStatus);
    layout->addItem(pushButtonExit);

    form = new QGraphicsWidget();
    form->setWindowFlags(Qt::Window);
    form->setWindowTitle("Robot Status");
    form->setLayout(layout);

    mapItem = new MapItem();
    myPointCloud = new PointCloudItem(QPointF(4.8,25.6), mapItem->imageWidth(), mapItem->imageHeight());
    robot = new RobotIcon(QPointF(4.8,25.6),0,mapItem->imageWidth(), mapItem->imageHeight());

    form->setPos(0,600);
    scene.addItem(mapItem);
    scene.addItem(myPointCloud);
    scene.addItem(robot);
    scene.addItem(form);
    mapItem->setScale(1.5);
    myPointCloud->setScale(1.5);
    robot->setScale(1.5);

    view = new QGraphicsView(&scene);
    //view->setFixedSize(1000,800);

    view->showNormal();

    rosThread->start();
}

DisplayWindow::~DisplayWindow()
{
    rosThread->quit();
    rosThread->wait();
}

void DisplayWindow::showRobotData(sensor_msgs::PointCloud pointCloudMsg, gnd_msgs::msg_pose2d_stamped poseMsg)
{
    lPose->setText(QString("PosX:%1 PoxY:%2 Theta:%3").arg(QString::number(poseMsg.x,'g',2)).arg(QString::number(poseMsg.y,'g',2)).arg(QString::number(poseMsg.theta,'g',2)));
    robot->setPose(QPointF(poseMsg.x, poseMsg.y), poseMsg.theta);

    myPointCloud->setPointsPosition(pointCloudMsg);
    scene.update();

}

void DisplayWindow::showRobotStatus(int status)
{
    lStatus->setText(QString("status:%1").arg(status));

}

void DisplayWindow::showRobotParticle(gnd_particle_localizer::msg_particles_pose2d_stamped particles)
{
    robot->setParticles(particles);
    scene.update();
}
