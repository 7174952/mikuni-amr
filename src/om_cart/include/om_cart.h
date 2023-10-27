#ifndef OM_CART_H
#define OM_CART_H

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <cmath>

#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_state.h"
#include "gnd_msgs/msg_pose2d_stamped.h"
#include "gnd_msgs/msg_velocity2d_with_covariance_stamped.h"
#include "om_data.h"

typedef struct
{
    long double x;
    long double y;
    long double theta;
} Pose2D;

enum
{
    WHEEL_LEFT_ID  = 1,
    WHEEL_RIGHT_ID = 2,
    CART_ID        = 15,

};

enum
{
    S_OFF   = 0,
    S_ON    = 1,
    ALM_RST = 2,
};

enum
{
    FC_READ       = 3,  //0x03
    FC_WRITE      = 16, //0x10
    FC_READ_WRITE = 23 //0x17
};

enum
{
    UN_USED         = 0,
    OPERATE         = 1,
    SET_SPEED       = 2,
    SET_MOTOR_PARAM = 3,
    SET_CART_PARAM  = 4,
};

enum
{
    CART_STATUS = 1,
    COMM_RESULT = 2,
};

typedef union
{
     struct
     {
        double vel_line;              //0
        double vel_theta;             //1
        double vel_left;              //2
        double vel_right;             //3
        double alm_code_L;            //4
        double alm_code_R;            //5
        double main_power_volt_L;     //6
        double main_power_volt_R;     //7
        double main_power_curr_L;     //8
        double main_power_curr_R;     //9
        double motor_temp_L;          //10
        double motor_temp_R;          //11
        double driver_temp_L;         //12
        double driver_temp_R;         //13
        double emergen_stop;          //14
     } cart_status;

     struct
     {
        double result;
        double error_code;
     }  comm_status;

     double data[30];

} Cart_Status_Info;

const uint8_t CART_STATUS_INFO_SIZE = 30;//14;


typedef union
{
    struct
    {
        double vel_line;
        double vel_theta;
    } set_speed;

    struct
    {
        double set_cmd;
    } operate_cmd;

    double data[30];

} Cart_Cmd_Info;

typedef struct
{
    int32_t vel;                //word[0]
    int32_t alm_code;           //word[1]
    int32_t main_volt;          //word[2]
    int32_t main_curr;          //word[3]
    int32_t motor_temp;         //word[4]
    int32_t driver_temp;        //word[5]
    int32_t driver_IO_INPUT;    //word[6]
    int32_t driver_IO_OUTPUT;   //word[7]
    int32_t reserved_0;         //word[8]
    int32_t reserved_1;         //word[9]
    int32_t reserved_2;         //word[10]
    int32_t reserved_3;         //word[11]
} Motor_Info;

typedef union
{
    struct
    {
        Motor_Info left;
        Motor_Info right;
    } Motor_Status;

    int32_t data[30];
} Cart_Resp_Info;
const uint8_t CART_RESP_INFO_ITEM_SIZE = 12 * 2; //6 * 2; // Left,Right wheel

const uint16_t UNIT_RATIO = 100;
const uint16_t CART_TREAD = 390; //tread = 390 mm




#endif // OM_CART_H
