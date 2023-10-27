/*
 * @file nkm_destination_queue.cpp
 * @brief wait and manage target waypoint from user reuqest
 * @author Hidekazu TAKAHASHI
 * @date 2017/11/07
 * Updated by ryu, 2023/10/20
 *  .Use Qt v5.14 lib to update config file
 */


#include <QtGlobal>
#include <QString>
#include <QtMath>
#include <QQueue>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

#include "gnd_msgs/msg_pose2d_stamped.h"
#include "gnd_msgs/msg_vehicle_status.h"

#include "nkm_destination_queue/srv_add_destination.h"
#include "nkm_destination_queue/srv_delete_destination.h"
#include "nkm_destination_queue/srv_show_queue.h"

#include "nkm_destination_queue_set_dest.h"
#include "nkm_destination_queue_set_dest_config.h"

// 目的地命令データ
struct DestinationInfo
{
    int index;
    std::string destName;
};

// 関数宣言
bool SrvAddDest(nkm_destination_queue::srv_add_destination::Request &req, nkm_destination_queue::srv_add_destination::Response &res);
bool SrvDelDest(nkm_destination_queue::srv_delete_destination::Request &req, nkm_destination_queue::srv_delete_destination::Response &res);
bool SrvShowQueue(nkm_destination_queue::srv_show_queue::Request &req, nkm_destination_queue::srv_show_queue::Response &res);

// 状態
enum ActStatus
{
    ActStatus_Idle   = 0,
    ActStatus_Wait   = 1,
    ActStatus_Move   = 2,
    ActStatus_Arrive = 3,
};

// Variables
double posX = 0.0;
double posY = 0.0;
int latestIndex = 0;
int currentVehicleStatus = 0;
ActStatus currentActStatus = ActStatus_Idle;

QQueue<DestinationInfo> destinationQueue;
DestinationInfo currentDestination;
nkm::NKMSetDest destinationSetter;

double posDestX = 0.0;
double posDestY = 0.0;
double distnaceArriveMax = 0;

// Calculate distance
double DistanceP2P(double x1, double y1, double x2, double y2)
{
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// 目的地追加サービス
bool SrvAddDest(nkm_destination_queue::srv_add_destination::Request &req, nkm_destination_queue::srv_add_destination::Response &res)
{
    if(destinationSetter.IsCorrectWaypoint(req.destination.c_str()))
    {
        DestinationInfo dest_tmp = {++latestIndex, req.destination};

        switch(req.indexInQueue)
        {
            case nkm_destination_queue::srv_add_destination::Request::First:
                destinationQueue.push_front(dest_tmp);
                break;
            case nkm_destination_queue::srv_add_destination::Request::Current:
                destinationQueue.push_front(dest_tmp);
                currentActStatus = ActStatus_Idle;
                break;
            case nkm_destination_queue::srv_add_destination::Request::Last:
                destinationQueue.push_back(dest_tmp);
                break;
            default: //index位置に挿入する仕様っぽく書いてるけどしてないよ
                destinationQueue.push_back(dest_tmp);
                break;
        }

        ROS_INFO("Add Destination: [%s]", req.destination.c_str());
        res.orderID = latestIndex;
    }
    else
    {
        ROS_ERROR("Destination name is wrong.(srvice)(name:%s)", req.destination.c_str());
        return false;
    }

    return true;
}

// 目的地削除サービス
bool SrvDelDest(nkm_destination_queue::srv_delete_destination::Request &req, nkm_destination_queue::srv_delete_destination::Response &res)
{
    switch(req.orderID)
    {
    case nkm_destination_queue::srv_delete_destination::Request::First:
        if(destinationQueue.size() >= 2)
        {
            ROS_INFO("Delete Destination: [%d:%s]", (destinationQueue.begin() + 1)->index, (destinationQueue.begin() + 1)->destName.c_str());
            destinationQueue.erase(destinationQueue.begin());
        }
        else
        {
            ROS_INFO("Cannot Delete Destination.");
            return false;
        }
        break;
    case nkm_destination_queue::srv_delete_destination::Request::Last:
        if(destinationQueue.size() >= 2)
        {
            ROS_INFO("Delete Destination: [%d:%s]", destinationQueue.back().index, destinationQueue.back().destName.c_str());
            destinationQueue.erase(destinationQueue.end());
        }
        else
        {
            ROS_INFO("Cannot Delete Destination.");
            return false;
        }
        break;
    case nkm_destination_queue::srv_delete_destination::Request::AllQueue:
        ROS_INFO("Delete All Queued Destination");
        if(destinationQueue.size() >= 1)
        {
            DestinationInfo inf = destinationQueue.front();
            destinationQueue.clear();
            destinationQueue.push_back(inf);
        }
        break;
    case nkm_destination_queue::srv_delete_destination::Request::Current:
        if(destinationQueue.size() >= 1)
        {
            ROS_INFO("Delete Destination: [%d:%s]", destinationQueue.front().index, destinationQueue.front().destName.c_str());
            destinationQueue.erase(destinationQueue.begin());
            destinationSetter.DeleteDestination();
            currentActStatus = ActStatus_Idle;
        }
        else
        {
            ROS_INFO("Cannot Delete Destination.");
            return false;
        }
        break;
    case nkm_destination_queue::srv_delete_destination::Request::All:
        ROS_INFO("Delete All Destination");
        destinationQueue.clear();
        destinationSetter.DeleteDestination();
        currentActStatus = ActStatus_Idle;
        break;
    default:
        auto r = std::find_if( destinationQueue.begin(), destinationQueue.end(), [req](DestinationInfo &i){ return(i.index == req.orderID);});
        if(r != destinationQueue.end())
        {
            ROS_INFO("Delete Destination: [%d:%s]", r->index, r->destName.c_str());
            if(r == destinationQueue.begin())
            {
                destinationSetter.DeleteDestination();
                currentActStatus = ActStatus_Idle;
            }
            destinationQueue.erase(r);
        }
        else
        {
            ROS_ERROR("Could not find index: [%d]", req.orderID);
            return false;
        }
        break;
    }

    return true;
}

// 目的地一覧
bool SrvShowQueue(nkm_destination_queue::srv_show_queue::Request &req, nkm_destination_queue::srv_show_queue::Response &res)
{
    ROS_INFO("Queue:");
    res.num = destinationQueue.size();
    for(const auto& itr : destinationQueue)
    {
        ROS_INFO("    %d:%s",itr.index, itr.destName.c_str());
        res.destination.emplace_back(itr.destName.c_str());
        res.orderID.emplace_back(itr.index);
    }

    return true;
}

void pose_callback(const gnd_msgs::msg_pose2d_stamped::ConstPtr& msg)
{
    posX = msg->x;
    posY = msg->y;

    return;
}

void vehicle_status_callback(const gnd_msgs::msg_vehicle_status::ConstPtr& msg)
{
    if( msg->status == gnd_msgs::msg_vehicle_status::VEHICLE_STATE_IDLE)
    {
        if(currentActStatus == ActStatus_Move)
        {
            //Arrive anywhere
            if(currentVehicleStatus != msg->status)
            {
                currentActStatus = ActStatus_Arrive;
            }
            //Arrived already?
            else if(DistanceP2P(posX, posY, posDestX, posDestY) < distnaceArriveMax)
            {
                // Arrive at correct waypoint
                currentActStatus = ActStatus_Arrive;
            }
        }
    }
    currentVehicleStatus = msg->status;

    return;
}

int main(int argc, char **argv)
{
    nkm::node_config node_config;
    // start up, read configuration file
    if (argc > 1)
    {
        if (nkm::fread_node_config(argv[1], &node_config) < 0)
        {
            ROS_ERROR("   ... Error: fail to read config file \"%s\"", argv[1]);

            QString fname = QString(argv[1]) + ".tmp";
            // file out configuration file
            if( nkm::fwrite_node_config( fname, &node_config ) >= 0 )
            {
                ROS_INFO("  : output sample configuration file %s", fname.toStdString().c_str());
            }

            return -1;
        }
        else
        {
            ROS_INFO("   ... read config file \"%s\"", argv[1]);
        }
    }

    // ---ROS Initialize---
    ros::init(argc, argv, node_config.node_name.value.at(0).toStdString());
    ros::NodeHandle nh;

    ros::ServiceServer srvRegistNode = nh.advertiseService(node_config.srv_name_add_destination.value.at(0).toStdString(), SrvAddDest);
    ROS_INFO("Service Start [%s]", node_config.srv_name_add_destination.value.at(0).toStdString().c_str());

    ros::ServiceServer srvUnregistNode = nh.advertiseService(node_config.srv_name_delete_destination.value.at(0).toStdString(), SrvDelDest);
    ROS_INFO("Service Start [%s]", node_config.srv_name_delete_destination.value.at(0).toStdString().c_str());

    ros::ServiceServer srvChangePriority = nh.advertiseService(node_config.srv_name_show_queue.value.at(0).toStdString(), SrvShowQueue);
    ROS_INFO("Service Start [%s]", node_config.srv_name_show_queue.value.at(0).toStdString().c_str());

    std_msgs::String msgNextDest;
    ros::Publisher pubNextDest = nh.advertise<std_msgs::String>(node_config.topic_name_pub_destination.value.at(0).toStdString(), 10);

    ros::Subscriber subPose =  nh.subscribe(node_config.topic_name_pose.value.at(0).toStdString(), 100, pose_callback);

    ros::Subscriber subStatue =  nh.subscribe(node_config.topic_name_status.value.at(0).toStdString(), 100, vehicle_status_callback);

    destinationSetter.Initialize(&node_config);

    // ---Main Loop---
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        switch(currentActStatus)
        {
        case ActStatus_Idle:
            if(destinationQueue.size() != 0)
            {
                int st = destinationSetter.SetDestination(destinationQueue.front().destName.c_str());
                if(st == 0)
                {
                    ROS_INFO("Set Destination: [%s]", destinationQueue.front().destName.c_str());
                    currentDestination = destinationQueue.front();
                    destinationSetter.GetWaypointPosition(currentDestination.destName.c_str(), &posDestX, &posDestY);
                    currentActStatus = ActStatus_Move;

                    msgNextDest.data = destinationQueue.front().destName;
                    pubNextDest.publish(msgNextDest);
                }
                else
                {
                    ROS_ERROR("Destination name is wrong.(set)(name:%s)", destinationQueue.front().destName.c_str());
                }
            }
            break;
        case ActStatus_Wait:
            break;
        case ActStatus_Move:
            break;
        case ActStatus_Arrive:
            double x, y;
            if( destinationSetter.GetWaypointPosition(currentDestination.destName.c_str(), &x, &y) >= 0)
            {
                if(DistanceP2P(posX, posY, x, y) < node_config.arrive_decision_distance.value.at(0))
                {
                    // Arrive at correct waypoint
                    auto res = std::find_if( destinationQueue.begin(), destinationQueue.end(), [](DestinationInfo &i){ return(i.index == currentDestination.index);});
                    if(res != destinationQueue.end())
                    {
                        destinationQueue.erase(res);
                    }
                }
                else
                {
                    // Arrive at wrong waypoint
                }
                currentActStatus = ActStatus_Idle;
            }
            else
            {
                ROS_ERROR("Destination name is wrong.(arrive)(name:%s)", destinationQueue.front().destName.c_str());
            }
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



