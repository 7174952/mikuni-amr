#ifndef USB_IO_H
#define USB_IO_H

#include <QtSerialPort/QSerialPort>
#include <QMessageBox>
#include <QDebug>
#include <QString>
#include <QByteArray>
#include <QVector>

#include "ros/ros.h"
#include "std_msgs/ByteMultiArray.h"

class Usb_io
{
public:
  Usb_io();
  ~Usb_io();

public:
  void openSerialPort();
  void closeSerialPort();
  bool writeData(const QByteArray& cmd);
  bool writeIomask();
  void readStatus();
  void update();
  void usbio_cmd_callback(const std_msgs::ByteMultiArray::ConstPtr& msg);

private:
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

private:
  //usb-io pin define
  const quint8 BIT_SHIFT_OUT_LAMP_NORMAL = 0; //bit0
  const quint8 BIT_SHIFT_OUT_LAMP_ALARM  = 1; //bit1
  const quint8 BIT_SHIFT_OUT_CY_FRONT    = 2; //bit2
  const quint8 BIT_SHIFT_OUT_CY_BACK     = 3; //bit3
  const quint8 BIT_SHIFT_IN_FEET_SW      = 7; //bit7
  const quint8 BIT_READ_MASK_ALL = 0x8f; //readall: bit0~3,bit7
  const quint8 BIT_MASKED = 0;
  const quint8 BIT_UNMASK = 1;
  const quint8 BIT_ON     = 1;
  const quint8 BIT_OFF    = 0;

  QSerialPort m_serial;
  std::string usbioPort;

  ros::NodeHandle nh;
  ros::Subscriber usbio_sub;
  ros::Publisher usbio_pub;
  std_msgs::ByteMultiArray msg_pub; //byte[0]-lamp normal,byte[1]-lamp alarm,byte[2]-cylinder front,byte[3]-cylinder back,
                                    //byte[4]-feet switch, byte[5-7]-reserved, byte[8]-status,0-OK,1-comm error
  USBIO_status_info usbio_status_info;
  QVector<QString> cmd_buf;
  const QString usbio_iomask_cmd = "gpio iomask 8f \r"; //mask: bit0~3,bit7 enable, bit4~bit6 disable
  const QString usbio_read_cmd = "gpio readall \r";


};

#endif // USB_IO_H
