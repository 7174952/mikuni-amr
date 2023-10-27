#ifndef OM_DATA_H
#define OM_DATA_H

#include "ros/ros.h"
#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_response.h"

class Om_data
{
public:
    Om_data();
    int8_t set_share_ID(uint8_t slave_id, uint8_t sub_id, ros::Publisher *pub);
    int8_t set_share_net_id(uint8_t slave_id, ros::Publisher *pub);
    int8_t reset_alarm(uint8_t slave_id, int8_t val,ros::Publisher *pub);
    int8_t update_share_state(uint8_t update_share_state, ros::Publisher *pub);
    int8_t drive_wheel_begin(uint8_t slave_id, ros::Publisher *pub);
    int8_t drive_wheel_end(uint8_t slave_id, ros::Publisher *pub);
    int8_t drive_wheel_cmd(uint8_t slave_id, int32_t vel_l, int32_t vel_r, ros::Publisher *pub);
    int8_t drive_cart_cmd(uint8_t slave_id, int32_t cart_vel_line, double cart_vel_theta, ros::Publisher *pub);
    int8_t get_lastest_query(om_modbus_master::om_query *query_msg);
    void clear_read_buff();
    void clear_write_buff();
    void clear_buff();
    void set_cmd_ready();
    void clear_cmd_ready();
    bool get_cmd_ready();

public:
    //modbus define
    const uint8_t global_id = 0x0F;      //broadcast all slaves in same group
    const uint8_t slave_1_id = 1;
    const uint8_t slave_2_id = 2;
    const uint8_t slave_1_sub_id = 1;
    const uint8_t slave_2_sub_id = 2;
    const uint8_t f_read = 0;   //read multi bytes from motor
    const uint8_t f_write = 1;   //write multi bytes to mode
    const uint8_t f_read_write = 2; //read and write at same time

private:
    const uint16_t CART_TREAD = 390; //tread = 390 mm

    om_modbus_master::om_query msg;
    om_modbus_master::om_query msg_last;
    bool cmd_ready;


};

#endif // OM_DATA_H
