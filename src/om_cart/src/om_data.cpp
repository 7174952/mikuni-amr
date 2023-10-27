#include "om_data.h"

Om_data::Om_data()
{
    clear_buff();
}

void Om_data::clear_read_buff()
{
    msg.read_addr = 0;
    msg.read_num = 0;

}

void Om_data::clear_write_buff()
{
    msg.write_addr = 0;
    msg.write_num = 0;

    for(int8_t i = 0;i<msg.data.size();i++)
    {
        msg.data[i] = 0;
    }
}

void Om_data::clear_buff()
{
    msg.slave_id = 0;
    msg.func_code = 0;

    clear_read_buff();
    clear_write_buff();

}

int8_t Om_data::set_share_ID(uint8_t slave_id, uint8_t sub_id, ros::Publisher *pub)
{
    // 1軸の設定
    clear_buff();
    msg.slave_id = slave_id;    // 書き込むドライバのスレーブID
    msg.func_code = f_write;    // 1:Write
    msg.write_addr = 0x0980;    // 書き込みの起点：Share Control Global IDのアドレス
    msg.write_num = 3;          // 書き込む数
    msg.data[0] = 15;           // Share control global ID
    msg.data[1] = 2;            // Share control number
    msg.data[2] = sub_id;       // Share control local ID
    pub->publish(msg);          // 配信する

    return 0;
}

int8_t Om_data::set_share_net_id(uint8_t slave_id, ros::Publisher *pub)
{
    // 1軸の設定
    clear_buff();
    msg.slave_id = slave_id;     // 書き込むドライバのスレーブID
    msg.func_code = f_write;     // 1:Write
    msg.write_addr = 0x0990;     // 書き込みの起点：Share Read data[0]
    msg.write_num = 24;          // 書き込むデータ数*軸数=24
    msg.data[0] = 80;     // Share Read data[5] → 検出速度
    msg.data[1] = 64;     // current alarm code
    msg.data[2] = 164;    // main power volt
    msg.data[3] = 155;    // main power current
    msg.data[4] = 125;    // motor temperature
    msg.data[5] = 124;    // driver tempearture
    msg.data[6] = 62;     // driver I/O Input Status
    msg.data[7] = 63;     // driver I/O Output Status
    msg.data[8] = 0;
    msg.data[9] = 0;
    msg.data[10] = 0;
    msg.data[11] = 0;

    msg.data[12] = 45;     // Share Write data[0] → DDO運転方式
    msg.data[13] = 46;     // Share Write data[1] → DDO位置
    msg.data[14] = 47;     // Share Write data[2] → DDO速度
    msg.data[15] = 48;     // Share Write data[3] → DDO加速レート
    msg.data[16] = 49;     // Share Write data[4] → DDO減速レート
    msg.data[17] = 50;     // Share Write data[5] → トグル制限
    msg.data[18] = 51;     // Share Write data[6] →　DDO反映トリガ
    msg.data[19] = 62;     // Share Write data[7] → S-ON
    msg.data[20] = 0;      // Share Write data[8] →
    msg.data[21] = 0;      // Share Write data[9] →
    msg.data[22] = 0;      // Share Write data[10] →
    msg.data[23] = 0;      // Share Write data[11] →
    pub->publish(msg);

    return 0;
}


int8_t Om_data::reset_alarm(uint8_t slave_id, int8_t val, ros::Publisher *pub)
{
    clear_buff();
    msg.slave_id = slave_id;
    msg.func_code = f_write;
    msg.write_addr = 0x0180;
    msg.write_num = 1;
    msg.data[0] = val;// 0->1->0: reset alarm
    pub->publish(msg);

    return 0;
}

int8_t Om_data::update_share_state(uint8_t slave_share_id, ros::Publisher *pub)
{
    clear_buff();
    msg.slave_id = slave_share_id;       // スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
    msg.func_code = f_read;              // 0:Read
    msg.read_addr = 0x0000;              // 読み出すアドレスの起点
    msg.read_num = 12*2; //6*2;                  // 各軸
    pub->publish(msg);                   // 配信する

    return 0;
}

int8_t Om_data::get_lastest_query(om_modbus_master::om_query *query_msg)
{
    query_msg = &msg;

    return 0;
}

int8_t Om_data::drive_wheel_begin(uint8_t slave_id, ros::Publisher *pub)
{
    // 運転指令(S-ONをONする)
    clear_buff();
    msg.slave_id = slave_id;      // スレーブID
    msg.func_code = f_write;      // ファンクションコード: 0:Read 1:Write 2:Read/Write
    msg.write_addr = 14;          // アドレス指定： ドライバ入力指令
    msg.write_num = 2;            // 書き込みデータ数: 1
    msg.data[0] = 1;              // Left:S-ONを立ち上げる
    msg.data[1] = 1;              // Right:S-ONを立ち上げる
    pub->publish(msg);            // 配信

    return 0;
}

int8_t Om_data::drive_wheel_end(uint8_t slave_id, ros::Publisher *pub)
{
    // 運転指令(S-ONをOFFする)
    clear_buff();
    msg.slave_id = slave_id;      // スレーブID
    msg.func_code = f_write;      // ファンクションコード: 0:Read 1:Write 2:Read/Write
    msg.write_addr = 14;          // アドレス指定： ドライバ入力指令
    msg.write_num = 2;            // 書き込みデータ数: 1
    msg.data[0] = 0;              // Left:S-ONをOFF
    msg.data[1] = 0;              // Right:S-ONをOFF
    pub->publish(msg);            // 配信

    return 0;
}

int8_t Om_data::drive_wheel_cmd(uint8_t slave_id, int32_t vel_l, int32_t vel_r, ros::Publisher *pub)
{
    clear_buff();
    // ID Shareモードで各モーターを運転する
    // 120[r/min]で運転させる
    msg.slave_id = slave_id;           // スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
    msg.func_code = f_write;           // 0:read 1:write 2:read/write
    msg.write_addr = 0x0000;           // 書き込むアドレスの起点
    msg.write_num =14;                 // 全軸合わせたデータ項目数を代入する
    // 1軸目のデータ
    msg.data[0] = 16;                  // DDO運転方式 16:連続運転(速度制御)
    msg.data[1] = 0;                   // DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
    msg.data[2] = vel_l;               // DDO運転速度(初期単位：0.01mm/s)
    msg.data[3] = 1000;                // DDO加速レート(初期単位：0.01mm/s/s)
    msg.data[4] = 1000;                // DDO減速レート(初期単位：0.01mm/s/s)
    msg.data[5] = 1000;
    msg.data[6] = 1;                   // DDO運転トリガ設定
    // 2軸目のデータ
    msg.data[7] = 16;                  // DDO運転方式 16:連続運転(速度制御)
    msg.data[8] = 0;                   // DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
    msg.data[9] = vel_r;               // DDO運転速度(初期単位：0.01mm/s)
    msg.data[10] = 1000;               // DDO加速レート(初期単位：0.01mm/s/s)
    msg.data[11] = 1000;               // DDO減速レート(初期単位：0.01mm/s/s)
    msg.data[12] = 1000;
    msg.data[13] = 1;                  // DDO運転トリガ設定
    // 配信
    pub->publish(msg);

    return 0;
}

int8_t Om_data::drive_cart_cmd(uint8_t slave_id, int32_t cart_vel_line, double cart_vel_theta, ros::Publisher *pub)
{
    int32_t vel_left, vel_right;

    vel_left  = cart_vel_line - 0.5 * CART_TREAD * cart_vel_theta;
    vel_right = -(cart_vel_line  + 0.5 * CART_TREAD * cart_vel_theta);

    clear_buff();
    // ID Shareモードで各モーターを運転する
    // 120[r/min]で運転させる
    msg.slave_id = slave_id;           // スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
    msg.func_code = f_write;           // 0:read 1:write 2:read/write
    msg.write_addr = 0x0000;           // 書き込むアドレスの起点
    msg.write_num =14;                 // 全軸合わせたデータ項目数を代入する
    // 1軸目のデータ
    msg.data[0] = 16;                  // DDO運転方式 16:連続運転(速度制御)
    msg.data[1] = 0;                   // DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
    msg.data[2] = vel_left;            // DDO運転速度(初期単位：0.01mm/s)
    msg.data[3] = 700;//1000;          // DDO加速レート(初期単位：ms)
    msg.data[4] = 500;//1000;          // DDO減速レート(初期単位：ms)
    msg.data[5] = 1000;                // トルク制限:0.1%
    msg.data[6] = 1;                   // DDO運転トリガ設定
    // 2軸目のデータ
    msg.data[7] = 16;                  // DDO運転方式 16:連続運転(速度制御)
    msg.data[8] = 0;                   // DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
    msg.data[9] = vel_right;           // DDO運転速度(初期単位：0.01mm/s)
    msg.data[10] = 700;//1000;         // DDO加速レート(初期単位：ms)
    msg.data[11] = 500;//1000;         // DDO減速レート(初期単位：ms)
    msg.data[12] = 1000;               // トルク制限：0.1%
    msg.data[13] = 1;                  // DDO運転トリガ設定
    // 配信
    pub->publish(msg);

    return 0;
}

void Om_data::set_cmd_ready()
{
    cmd_ready = true;
}

void Om_data::clear_cmd_ready()
{
    cmd_ready = false;
}

bool Om_data::get_cmd_ready()
{
    return cmd_ready;
}

