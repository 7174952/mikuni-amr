#ifndef AMR_DEFAULT_GUI_H
#define AMR_DEFAULT_GUI_H

#include <QApplication>
#include <QWidget>
#include <QString>
#include <QStringList>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QVector>
#include <QStringList>
#include <QProcess>
#include <QDebug>
#include <QMessageBox>
#include <QTextDecoder>
#include <QTextCodec>
#include <QTimer>
#include <QtGlobal>
#include <QFileDialog>
#include <QDate>
#include <QTime>
#include <QDir>
#include <QtMultimedia/QSound>
#include <QThread>

#include "ros/ros.h"
#include "om_cart/om_cart_state.h"
#include "om_cart/om_cart_cmd.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/Bool.h"
#include "gnd_msgs/msg_vehicle_status.h"
#include "gnd_msgs/msg_pose2d_stamped.h"

#define gnd_rad2deg(x)			( (x) * ( 180.0 / M_PI ) )

namespace Ui {
class Amr_Default_Gui;
}

class Amr_Default_Gui : public QWidget
{
  Q_OBJECT

public:
  explicit Amr_Default_Gui(QWidget *parent = nullptr);
  ~Amr_Default_Gui();

public:
  void init_config_file(QString fileName);
  void set_serial_info();
  void update_serial_port(QString fileName);
  void start_om_cart();
  void start_usbio_board();
  void execute_app_cmd(QProcess *proc,QString strCmd);
  QString get_output_str(QProcess *proc);
  QString get_err_str(QProcess *proc);
  QString execute_shell_cmd(QString strCmd);
  void kill_all_app_process();
  void cartStatusCallback(const om_cart::om_cart_state::ConstPtr& status);
  void usbioStatusCallback(const std_msgs::ByteMultiArray::ConstPtr& status);
  void amrStatusCallback(const gnd_msgs::msg_vehicle_status::ConstPtr& status);
  void targetArrivedCallback(const std_msgs::Bool::ConstPtr& msg);
  void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
  void pose_Callback(const gnd_msgs::msg_pose2d_stamped::ConstPtr& msg);
  void enable_all_buttons();
  void setup_log();
  void save_log(QString str);
  void save_running_data();
  void setup_running_data_log();
  void close_running_data_log();
  void set_usb_port(uint8_t num, uint8_t val);
  void check_cy_output();

private:
  void show_message(QString msg);
  void check_alarm_state();

  /*-PI < theta < PIに調整する*/
  double trans_q(double theta);

private slots:
  void spinOnce();

  void on_pushButton_chk_port_clicked();

  void on_readoutput_om_cart();
  void on_readerror_om_cart();
  void on_finished_om_cart(int exitCode);

  void on_readoutput0();
  void on_readerror0();
  void on_finished0(int exitCode);
  void on_readoutput1();
  void on_readerror1();
  void on_finished1(int exitCode);
  void on_readoutput2();
  void on_readerror2();
  void on_finished2(int exitCode);

  void on_pushButton_collect_data_clicked();

  void on_pushButton_start_amr_clicked();

  void on_pushButton_make_root_clicked();

  void on_pushButton_run_manual_clicked();

  void on_pushButton_make_map_clicked();

  void on_pushButton_reset_alarm_clicked();

  void on_pushButton_set_s_on_clicked();

private:
  enum
  {
      CART_STATUS = 1,
      COMM_RESULT = 2,
  };

  enum OM_CMD_TYPE
  {
      UN_USED         = 0,
      OPERATE         = 1,
      SET_SPEED       = 2,
      SET_MOTOR_PARAM = 3,
      SET_CART_PARAM  = 4,
  };

  enum OPER_CMD
  {
      S_OFF   = 0,
      S_ON    = 1,
      ALM_RST = 2,
  };

  //usb-io enum
  enum IO_STATE
  {
      IO_OFF = 0,
      IO_ON,
      IO_OPEN =0,
      IO_CLOSE,
  };
  enum IO_NUM_DEVICE
  {
      USB_IO_LAMP_NORMAL = 0,
      USB_IO_LAMP_ALARM,
      USB_IO_CYLINDER_FRONT,
      USB_IO_CYLINDER_BACK,
      USB_IO_FEET_SWITCH = 7,
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

  typedef struct
  {
      const QString node_name                  = "node-name";
      const QString work_path                  = "work-path";
      const QString package_name               = "package-name";
      const QString map_data_name_prefix       = "map-data-name-prefix";
      const QString data_log_name_prefix       = "data-log-name-prefix";
      const QString status_log_name_prefix     = "status-log-name-prefix";
      const QString low_battery_volt           = "low-battery-volt";
      const QString low_battery_volt_threshold = "low-battery-volt-threshold";
      const QString low_battery_volt_stop      = "low-battery-volt-stop";
      const QString low_battery_volt_reset     = "low-battery-volt-reset";
      const QString overload_wait_time         = "overload-wait-time";

  } Config_items_name;

  typedef struct
  {
      QString node_name;
      QString work_path;
      QString package_name;
      QString map_data_name_prefix;
      QString data_log_name_prefix;
      QString status_log_name_prefix;
      qint32 low_battery_volt;
      qint32 low_battery_volt_threshold;
      qint32 low_battery_volt_stop;
      qint32 low_battery_volt_reset;
      qint32 overload_wait_time;

      void init()
      {
          low_battery_volt = 0;
          low_battery_volt_threshold = 0;
          low_battery_volt_stop = 0;
          low_battery_volt_reset = 0;
          overload_wait_time = 0;
      }

  } Config_items_val;

  typedef struct
  {
      Config_items_name items_name;
      Config_items_val items_val;
  } Amr_start_default_config_info;

  //running data log
  typedef struct
  {
      QString rec_time;
      QString vel_line;
      QString vel_theta;
      QString vel_left;
      QString vel_right;
      QString pose_x;
      QString pose_y;
      QString pose_theta;
      QString emergen_stop;
      QString bat_volt;
      QString bat_total_curr;
      QString bat_curr_left;
      QString bat_curr_right;
      QString alarm_info;
      QString alert_info;
  } Log_run_info;

  //define alarm info
  typedef struct
  {
      bool alm_on;
      QString code;
      QString disp;
      bool auto_reset;
      qint32 err_timer;

  } Alm_Item;

  typedef struct
  {
      struct
      {
          Alm_Item imu;
          Alm_Item lidar;
          Alm_Item om_cart;
          Alm_Item joy;
          Alm_Item usb_io;
      } devices;

      Alm_Item low_volt;
      Alm_Item over_load;
      Alm_Item emergen_stop;
      Alm_Item om_driver;
  } Alarm_Info;

  //define amr running state
  enum AMR_STATE
  {
      STOPPED = 0,    //0
      AUTO_RUN,       //1
      COLLECT_DATA,   //2
      MANUAL,         //3
      MAKE_MAP,       //4
      MAKE_ROOT,      //5
      ALARM,          //6
      STOPPING,       //7
  };

  enum DEVICE_KEYWORD
  {
      IMU = 0,
      LIDAR,
      OM_CART,
      JOYSTICK,
      USB_IO,
  };

  typedef struct
  {
      uint16_t amr_state;
      bool cart_move;
      bool alarm_on;
  }Amr_state_info;

//usb-io info
#define USBIO_STATUS_DATA_SIZE  9

  typedef union
  {
      struct
      {
          quint8 lamp_normal;    //0-OFF,1-ON
          quint8 lamp_alarm;     //0-OFF,1-ON
          quint8 cylinder_front; //0-OFF,1-ON
          quint8 cylinder_back;  //0-OFF,1-ON
          quint8 feet_switch;    //0-OFF,1-ON
          quint8 reserved[3];
          quint8 status;         //0-comm ok,1-comm error

      } items;
      quint8 data[USBIO_STATUS_DATA_SIZE];
  } USBIO_status_info;

#define USBIO_CMD_DATA_SIZE  3
  typedef union
  {
      struct
      {
          quint8 io_num;     //io0~io3
          quint8 value;      //0-OFF,1-ON
          quint8 comm_reset; //0-no,1-reset
      }items;
      quint8 data[USBIO_CMD_DATA_SIZE];
  } USBIO_cmd_info;

  typedef struct
  {
      bool amr_start;
      qint16 sw_close_timer;

  }Footswich_start_info;

  typedef struct
  {
      bool ftsw_started;
      bool waypoint_stopped;
      int32_t cy_front_timer;
      int32_t cy_back_timer;
  }Cy_Action_Info;

  //setup sound
  typedef struct
  {
      bool music_run_on;
      bool music_alm_on;
      QSound *music_run;
      QSound *music_alarm;
  }Sound_Info;

private:
  Ui::Amr_Default_Gui *ui;
  Amr_start_default_config_info amr_config_info;

  //define serial port
  const uint8_t ttyUSB0 = 0;
  const uint8_t ttyUSB1 = 1;
  const QVector<QString> device_name = {"Cart", "Lidar"};
  const QVector<QString> device_info = {"cp210x", "FTDI"};
  const QVector<QString> attach_info = {"attached", "disconnected"};
  const QVector<QString> serial_info = {"ttyUSB0", "ttyUSB1"};
  QString cart_port;
  QString lidar_port;
  QString imu_port;
  QString usbio_port;

  QString work_path;
  QProcess *om_process;
  QVector<QProcess *> proc_amr;
  bool is_amr_on;
  qint32 wait_stop_tmr;  //-1:stop, >=0: Counter up
  const qint32 MAX_STOP_TIME = 30 * 100; //max - 30s
  QString func_name;
  QPushButton *pressed_btn;

  //define ros variable
  QTimer *ros_timer;
  ros::NodeHandle nh;
  ros::Subscriber cart_status_sub;
  Cart_Status_Info cart_status_info;
  ros::Subscriber lidar_sub;
  ros::Subscriber imu_sub;
  ros::Publisher om_cmd_pub;
  ros::Subscriber pose_sub;
  //usb-io ros info
  ros::Subscriber usbio_status_sub;
  ros::Publisher usbio_cmd_pub;
  ros::Subscriber amr_status_sub;
  ros::Publisher amr_start_pub;
  std_msgs::Bool msg_amr_start;
  ros::Subscriber target_arrived_sub;


  //Set log file
  const QStringList log_item_name = {
      "Time(s)",
      "vel_line(m/s)",
      "vel_theta(rad/s)",
      "vel_left(m/s)",
      "vel_right(m/s)",
      "pose_x(m)",
      "pose_y(m)",
      "pose_theta(deg)",
      "emergen_stop",
      "bat_volt(V)",
      "bat_total_curr(A)",
      "bat_curr_left(A)",
      "bat_curr_right(A)",
      "alarm_info",
      "alert_info"
  };

  bool is_save_data_log;
  QFile status_log_name;
  Log_run_info log_run_info;
  QFile running_data_file;
  qint32 save_running_data_timer;
  const qint32 DATA_LOG_MAX_TIME = 10; //10*10 - 100ms
  QTime log_start_time;

  //Define alarm and running state info
  const QStringList amr_state_disp = {
    "Stop",
    "Auto Run",
    "Collect Data",
    "Manual",
    "Make Map",
    "Make Root",
    "Alarm",
    "stopping"
  };

  const QStringList amr_keyword = {
    "IMU",
    "LIDAR",
    "OM-CART",
    "joystick",
    "USB-IO"
  };

  const uint16_t MAX_ERROR_TIME = 300; //300*10=3000ms
  Alarm_Info alarm_info;
  Amr_state_info amr_state_info;

  //usb-io info
  const QStringList USBIO_OUT_STATE = {"OFF","ON"};
  const QStringList USBIO_IN_STATE = {"CLOSE","OPEN"};
  USBIO_cmd_info cmd_info;
  USBIO_status_info usbio_info;
  QProcess *usbio_process;
  uint8_t amr_status;
  Footswich_start_info ftsw_start_info;
  const uint8_t KEEP_CLOSED_100_MS = 5; //10ms*10=100ms

  const uint16_t CY_TIMING_FRONT_ON  = 10;              //10ms*100          = 1000ms
  const uint16_t CY_TIMING_FRONT_OFF = (10 + 30);      //100ms*(10+30)    = 4000ms
  const uint16_t CY_TIMING_AMR_START = (10 + 30 + 5); //100ms*(10+30+5) = 4500ms
  const uint16_t CY_TIMING_BACK_ON   = 10;              //100ms*10          = 1000ms
  const uint16_t CY_TIMING_BACK_OFF  = (10 + 30);      //100ms*(10+30)    = 4000ms
  Cy_Action_Info cy_action_info;

  //Running music
  Sound_Info sound_info;

  //Set S_ON,S_OFF
  std_msgs::Bool is_set_s_on;
  ros::Publisher  pub_cart_s_on;

};

#endif // AMR_DEFAULT_GUI_H
