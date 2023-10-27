#include <cstdio>
#include <cmath>
#include <vector>
#include <fstream>
#include <iostream>
#include <QFile>
#include <QTextStream>

#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include "nkm_destination_queue/srv_add_destination.h"
#include "gnd_msgs/msg_vehicle_status.h"
#include "std_msgs/Bool.h"

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
#define DEMO_WAIT_JOY_CMD_RUN  3

std::vector<Target_info> target_list;
ros::ServiceClient client_setTarget;
int16_t target_pos = 0;
bool run_by_joy = true;
ros::Publisher  pub_voice;   


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
    nkm_destination_queue::srv_add_destination srv_dest;
    std_msgs::Bool is_arrived;
    is_arrived.data = false;
    int8_t way_point_status = -1; //-1 - stop and wait joy cmd to run, 0 - not stop , >0 - stop to wait n minutes and auto to run to next waypoint

    way_point_status = target_list[target_pos].waitTime;
    //send stopping angle
    //send target and start to run
    srv_dest.request.destination = target_list[target_pos].targetName;
    srv_dest.request.indexInQueue = 1;
    srv_dest.request.moveTheta = (int32_t)((target_list[target_pos].stop2TurnAngle / 180) * M_PI);
    if(client_setTarget.call(srv_dest))
    {
        ROS_INFO("Set Dest:%s OK", srv_dest.request.destination.c_str());
    }
    else
    {
        ROS_ERROR("Set Dest:%s Failed!", srv_dest.request.destination.c_str());
        return -1;
    }  
    target_pos += 1;
 
    if(target_pos >= target_list.size())
    {
        target_pos -= target_list.size();
    }

    return way_point_status;
}

void footSwitchCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data == true)
    {
        run_by_joy = true;
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

        ss_num = std::istringstream(res[3]);
        ss_num >> item_tmp.waitTime;

        target_list.push_back(item_tmp);
       
    }
    file->close();
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"auto_navi");
    ros::NodeHandle nh;
    ros::Subscriber sub_foot_switch;
    ros::Subscriber sub_vehicle_status;
    uint8_t state;
    int8_t waypoint_status;

    std_msgs::Bool is_arrived;
    is_arrived.data = false;


    ROS_INFO("Auto Navi node started!");

    std::string file_name = "target_list.txt";
    //Read target list file
    if(argc > 1)
    {
        file_name = argv[1];
    }
    std::ifstream target_file(file_name,std::ios::in);
    if(!target_file)
    {
        std::cout << "Open target file error!" << std::endl;
        return 0;
    }

    read_target_list_from_file(&target_file);

    //init topic
    sub_foot_switch = nh.subscribe<std_msgs::Bool>("amr_start",10,&footSwitchCallback);
    sub_vehicle_status = nh.subscribe<gnd_msgs::msg_vehicle_status>("vehicle_status",10,&vehicleStatusCallback);
    pub_voice = nh.advertise<std_msgs::Bool>("voice_arrived",100);
    //init service
    client_setTarget = nh.serviceClient<nkm_destination_queue::srv_add_destination>("trajectory_tracking/set_move_angle");

    ros::Rate loop_rate(100);

    if(vehi_status == gnd_msgs::msg_vehicle_status::VEHICLE_STATE_IDLE)
    {
        state = DEMO_WAIT_JOY_CMD_RUN;
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
                state = (waypoint_status >= 0) ?  DEMO_SET_TARGET_POINT : DEMO_WAIT_JOY_CMD_RUN;
                if(state == DEMO_WAIT_JOY_CMD_RUN)
                {
                    is_arrived.data = true;
                    pub_voice.publish(is_arrived);
                }
            }
            break;
        case DEMO_WAIT_JOY_CMD_RUN:
            if(run_by_joy)
            {
                run_by_joy = false;
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
