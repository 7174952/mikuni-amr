#include "usb_io.h"

Usb_io::Usb_io()
{
    //update serialPort number from parameter
    nh.param<std::string>("usb_io_node/usbioPort", usbioPort, "/dev/ttyACM0");

    for(int i = 0; i < USBIO_STATUS_DATA_SIZE;i++)
    {
        usbio_status_info.data[i] = 0;
    }

    //set and open serial Port
    openSerialPort();

    //set ros subscriber and publisher
    usbio_sub = nh.subscribe("usbio_cmd", 10, &Usb_io::usbio_cmd_callback,this);
    usbio_pub = nh.advertise<std_msgs::ByteMultiArray>("usbio_status",10);
    msg_pub.data.resize(USBIO_STATUS_DATA_SIZE);
}

Usb_io::~Usb_io()
{
    closeSerialPort();
}

void Usb_io::usbio_cmd_callback(const std_msgs::ByteMultiArray::ConstPtr& msg)
{
    USBIO_cmd_info cmd_info;
    if(msg->data.size() < USBIO_CMD_DATA_SIZE)
    {
        qDebug() << "Error: USB-IO cmd message size less than " << USBIO_CMD_DATA_SIZE;
        return;
    }

    //check user usb-io command
    for (int i = 0; i < msg->data.size(); ++i)
    {
        cmd_info.data[i] = msg->data.at(i);
    }
    if(cmd_info.items.comm_reset > 0) //need to reset usb-io board
    {
        openSerialPort();
        return;
    }
    else
    {
        QString user_cmd;

        if(cmd_info.items.value > 0)
        {
            user_cmd = "gpio set " + QString::number(cmd_info.items.io_num) + " \r";
        }
        else
        {
            user_cmd = "gpio clear " + QString::number(cmd_info.items.io_num) + " \r";
        }
        cmd_buf.append(user_cmd);
    }
}

void Usb_io::update()
{
    bool result = true;
    QString strCmd;

    if(!cmd_buf.isEmpty())
    {
        //set io port output
        strCmd = cmd_buf.takeFirst();
        result = writeData(strCmd.toUtf8());
        ros::Duration(0.1).sleep();
        if(result)
        {
            result = writeIomask();
            ros::Duration(0.1).sleep();
        }
    }

    if(result)
    {
        //read usb-io board all i/o port status,io0~io7
        readStatus();
    }

}

void Usb_io::openSerialPort()
{
    m_serial.setPortName(QString::fromStdString(usbioPort));
    m_serial.setFlowControl(QSerialPort::NoFlowControl);

    if(m_serial.isOpen())
    {
        m_serial.close();
    }
    //reopen again
    if (!m_serial.open(QIODevice::ReadWrite))
    {
        QMessageBox::critical(NULL, "Open USB-IO Board Error!", m_serial.errorString());
        return;
    }

    //set mask and prepare for readall status
    writeIomask();
}


void Usb_io::closeSerialPort()
{
    if (m_serial.isOpen())
    {
        m_serial.close();
    }
}

bool Usb_io::writeData(const QByteArray &cmd)
{
    bool result;

    m_serial.write(cmd);
    result = m_serial.waitForReadyRead();

    if(result)
    {
        m_serial.readAll();
    }

    return result;
}

bool Usb_io::writeIomask()
{
    bool result;

    m_serial.write(usbio_iomask_cmd.toUtf8());
    result = m_serial.waitForReadyRead();
    if(result)
    {
        m_serial.readAll();
    }

    return result;
}

void Usb_io::readStatus()
{
    m_serial.write(usbio_read_cmd.toUtf8());
    m_serial.waitForReadyRead();

    QStringList data = QString(m_serial.readAll()).replace("\r","").split("\n");
    QString str_val = data.at(1).trimmed();
    if(str_val.isEmpty())
    {
        usbio_status_info.items.status = 1; //comm error
        usbio_status_info.items.lamp_normal    = 0;
        usbio_status_info.items.lamp_alarm     = 0;
        usbio_status_info.items.cylinder_front = 0;
        usbio_status_info.items.cylinder_back  = 0;
        usbio_status_info.items.feet_switch    = 0;

    }
    else
    {
        quint8 val = str_val.toUInt(NULL,16);

        usbio_status_info.items.status = 0; //comm ok
        usbio_status_info.items.lamp_normal    = ((quint8)val >> BIT_SHIFT_OUT_LAMP_NORMAL) & 0x01;
        usbio_status_info.items.lamp_alarm     = ((quint8)val >> BIT_SHIFT_OUT_LAMP_ALARM) & 0x01;
        usbio_status_info.items.cylinder_front = ((quint8)val >> BIT_SHIFT_OUT_CY_FRONT) & 0x01;
        usbio_status_info.items.cylinder_back  = ((quint8)val >> BIT_SHIFT_OUT_CY_BACK) & 0x01;
        usbio_status_info.items.feet_switch    = ((quint8)val >> BIT_SHIFT_IN_FEET_SW) & 0x01;

    }

    //publish usb-io port status
    for(uint i = 0; i < USBIO_STATUS_DATA_SIZE; i++)
    {
        msg_pub.data[i] = usbio_status_info.data[i];
    }

    usbio_pub.publish(msg_pub);

}
