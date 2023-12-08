#include <cstdio>
#include <cmath>
#include <vector>
#include <fstream>
#include <iostream>
#include <QFile>
#include <QTextStream>

#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include "gnd_msgs/msg_vehicle_status.h"
#include "std_msgs/Bool.h"
#include "auto_navi.h"
#include "gnd_rosutil.h"

typedef struct
{
    int num;
    std::string targetName;
    double stop2TurnAngle;
    double waitTime;

} Target_info;

#define DEMO_SET_TARGET_POINT  0
#define DEMO_WAIT_VEHICLE_RUN  1
#define DEMO_WAIT_VEHICLE_IDLE 2
#define DEMO_WAIT_MANUAL_RUN  3

std::vector<Target_info> target_list;
ros::ServiceClient client_setTarget;
int16_t target_pos = 0;
bool run_by_manual = true;
ros::Publisher  pub_voice;   
nkm::AutoSetDest destinationSetter;


uint8_t vehi_status = gnd_msgs::msg_vehicle_status::VEHICLE_STATE_IDLE;


std::vector<std::string> split(const std::string& str, const std::string& delim)
{
    std::vector<std::string> res;
    if("" == str) return res;

    char *strs = new char[str.length()+1];
    std::strcpy(strs,str.c_str());

    char *d = new char[delim.length()+1];
    std::strcpy(d,delim.c_str());

    char *p = std::strtok(strs,d);
    while(p)
    {
        std::string s = p;
        res.push_back(s);
        p = std::strtok(NULL, d);
    }

    return res;
}

int8_t setTargetPoint()
{
    // nkm_destination_queue::srv_add_destination srv_dest;
    std_msgs::Bool is_arrived;
    is_arrived.data = false;
    int8_t wait_time = -1; //-1 - stop and wait maunal cmd to run, 0 - not stop , >0 - stop to wait n minutes and auto to run to next waypoint

    wait_time = target_list[target_pos].waitTime;
    //send target and start to run
    ROS_INFO("setTargetPoint waypoint: %s", target_list[target_pos].targetName.c_str());

    if(destinationSetter.SetDestination(target_list[target_pos].targetName.c_str(), target_list[target_pos].stop2TurnAngle) < 0)
    {
        ROS_ERROR("Error:Set Target:%s Failed!", target_list[target_pos].targetName.c_str());
    }
    else
    {
        ROS_INFO("Set Target:%s Successful!", target_list[target_pos].targetName.c_str());
    }

    target_pos += 1;
 
    if(target_pos >= target_list.size())
    {
        target_pos -= target_list.size();
    }

    return wait_time;
}

void footSwitchCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data == true)
    {
        run_by_manual = true;
    }
}

void vehicleStatusCallback(const gnd_msgs::msg_vehicle_status::ConstPtr& vehicle_status)
{
    vehi_status = vehicle_status->status;

}

void read_target_list_from_file(std::ifstream *file)
{
    Target_info item_tmp;
    char lineText[100];
        
    while(file->getline(lineText,100))
    {
        //save target to buffer
        std::vector<std::string> res = split(lineText," \r\n;");
        if((res.size() == 0) || (res[0][0]=='#')) continue;
        if(res[0] == "<demo>") 
        {
            continue;
        }

        std::istringstream ss_num;
        ss_num  = std::istringstream(res[0]);
        ss_num >> item_tmp.num;

        item_tmp.targetName = res[1];

        ss_num = std::istringstream(res[2]);
        ss_num >> item_tmp.stop2TurnAngle;
        item_tmp.stop2TurnAngle = gnd_deg2ang(item_tmp.stop2TurnAngle);

        ss_num = std::istringstream(res[3]);
        ss_num >> item_tmp.waitTime;

        target_list.push_back(item_tmp);
       
    }
    file->close();
}

int main(int argc, char** argv)
{
    nkm::node_config			node_config;
    // read configuration
    if( argc > 1 )
    {
        ROS_INFO("=> read configuration file");
        if( nkm::fread_node_config( QString(argv[1]), &node_config ) < 0 )
        {
            ROS_ERROR("... Error: fail to read configuration file %s", argv[1]);
            QString fname = QString(argv[1]) + ".tmp";
            // file out configuration file
            if( nkm::fwrite_node_config( fname, &node_config ) >= 0 )
            {
                ROS_INFO("     : output sample configuration file %s", fname.toStdString().c_str());
            }
            return -1;
        }
        else
        {
            ROS_INFO("   ... ok, read config file %s", argv[1]);
        }
    }

    ros::init(argc,argv,node_config.node_name.value.at(0).toStdString());
    ros::NodeHandle nh;
    ros::Subscriber sub_foot_switch;
    ros::Subscriber sub_vehicle_status;
    uint8_t state;
    int8_t waypoint_status;

    std_msgs::Bool is_arrived;
    is_arrived.data = false;

    destinationSetter.Initialize(&node_config);

    ROS_INFO("Auto Navi node started!");

    //Read target list file
    std::ifstream target_file(node_config.target_list_file.value.at(0).toStdString(),std::ios::in);
    if(!target_file)
    {
        std::cout << "Open target file error!" << std::endl;
        return 0;
    }

    read_target_list_from_file(&target_file);

    //init topic
    sub_foot_switch = nh.subscribe<std_msgs::Bool>("amr_start",10,&footSwitchCallback);
    sub_vehicle_status = nh.subscribe<gnd_msgs::msg_vehicle_status>(node_config.topic_name_status.value.at(0).toStdString(),10,&vehicleStatusCallback);
    pub_voice = nh.advertise<std_msgs::Bool>("voice_arrived",100);
    //init service
    // client_setTarget = nh.serviceClient<nkm_destination_queue::srv_add_destination>("trajectory_tracking/set_move_angle");

    ros::Rate loop_rate(100);

    if(vehi_status == gnd_msgs::msg_vehicle_status::VEHICLE_STATE_IDLE)
    {
        state = DEMO_WAIT_MANUAL_RUN;
    }
    else
    {
        state = DEMO_WAIT_VEHICLE_IDLE;
    }

    while(ros::ok())
    {
        switch(state)
        {
        case DEMO_SET_TARGET_POINT:
            waypoint_status = setTargetPoint();
            state = DEMO_WAIT_VEHICLE_RUN;
            break;
        case DEMO_WAIT_VEHICLE_RUN:
            if(vehi_status > gnd_msgs::msg_vehicle_status::VEHICLE_STATE_IDLE)
            {
                state = DEMO_WAIT_VEHICLE_IDLE;
            }
            break;
        case DEMO_WAIT_VEHICLE_IDLE:
            if(vehi_status == gnd_msgs::msg_vehicle_status::VEHICLE_STATE_IDLE)
            {
                state = (waypoint_status >= 0) ?  DEMO_SET_TARGET_POINT : DEMO_WAIT_MANUAL_RUN;
                if(state == DEMO_WAIT_MANUAL_RUN)
                {
                    is_arrived.data = true;
                    pub_voice.publish(is_arrived);
                }
            }
            break;
        case DEMO_WAIT_MANUAL_RUN:
            if(run_by_manual)
            {
                run_by_manual = false;
                state = DEMO_SET_TARGET_POINT;
            }
            break;
        default:
            state = DEMO_WAIT_VEHICLE_IDLE;
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
