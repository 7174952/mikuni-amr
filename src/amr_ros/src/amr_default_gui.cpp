#include "amr_default_gui.h"
#include "ui_amr_default_gui.h"
#include "opencv2/opencv.hpp"
#include "qcustomplot.h"

Amr_Default_Gui::Amr_Default_Gui(QWidget *parent) : QWidget(parent), ui(new Ui::Amr_Default_Gui)
{
    ui->setupUi(this);

    //init serial port
    cart_port = "None";
    lidar_port = "None";
    imu_port = "None";
    usbio_port = "None";

    om_process = new QProcess(this);
    connect(om_process, &QProcess::readyReadStandardOutput, this, &Amr_Default_Gui::on_readoutput_om_cart);
    connect(om_process, &QProcess::readyReadStandardError, this, &Amr_Default_Gui::on_readerror_om_cart);
    connect(om_process, SIGNAL(finished(int)), this, SLOT(on_finished_om_cart(int)));

    proc_amr.resize(3);
    proc_amr[0] = new QProcess(this);
    connect(proc_amr[0], &QProcess::readyReadStandardOutput,this,&Amr_Default_Gui::on_readoutput0);
    connect(proc_amr[0], &QProcess::readyReadStandardError, this, &Amr_Default_Gui::on_readerror0);
    connect(proc_amr[0], SIGNAL(finished(int)), this, SLOT(on_finished0(int)));

    proc_amr[1] = new QProcess(this);
    connect(proc_amr[1], &QProcess::readyReadStandardOutput,this,&Amr_Default_Gui::on_readoutput1);
    connect(proc_amr[1], &QProcess::readyReadStandardError, this, &Amr_Default_Gui::on_readerror1);
    connect(proc_amr[1], SIGNAL(finished(int)), this, SLOT(on_finished1(int)));

    proc_amr[2] = new QProcess(this);
    connect(proc_amr[2], &QProcess::readyReadStandardOutput,this,&Amr_Default_Gui::on_readoutput2);
    connect(proc_amr[2], &QProcess::readyReadStandardError, this, &Amr_Default_Gui::on_readerror2);
    connect(proc_amr[2], SIGNAL(finished(int)), this, SLOT(on_finished2(int)));

    is_amr_on = false;
    wait_stop_tmr = -1;
    amr_config_info.items_val.init();

    // setup the timer that will signal ros stuff to happen
    ros_timer = new QTimer(this);
    connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
    ros_timer->start(10);  // set the rate to 10ms  You can change this if you want to increase/decrease update rate
    cart_status_sub = nh.subscribe<om_cart::om_cart_state>("cart_status", 100, &Amr_Default_Gui::cartStatusCallback, this);
    lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 100, &Amr_Default_Gui::lidar_callback, this);
    imu_sub = nh.subscribe<sensor_msgs::Imu>("imu/data_raw", 100, &Amr_Default_Gui::imu_callback, this);
    om_cmd_pub = nh.advertise<om_cart::om_cart_cmd>("cart_cmd",10);
    pose_sub = nh.subscribe<gnd_msgs::msg_pose2d_stamped>("pose_particle_localizer", 10, &Amr_Default_Gui::pose_Callback, this);

    //setup usbio ros info
    usbio_process = new QProcess(this);
    usbio_status_sub = nh.subscribe("usbio_status",100, &Amr_Default_Gui::usbioStatusCallback, this);
    usbio_cmd_pub = nh.advertise<std_msgs::ByteMultiArray>("usbio_cmd",10);
    amr_status_sub = nh.subscribe<gnd_msgs::msg_vehicle_status>("vehicle_status", 100, &Amr_Default_Gui::amrStatusCallback, this);
    target_arrived_sub = nh.subscribe<std_msgs::Bool>("voice_arrived",10, &Amr_Default_Gui::targetArrivedCallback, this);
    amr_status = gnd_msgs::msg_vehicle_status::VEHICLE_STATE_IDLE;
    ftsw_start_info = {false,0};
    amr_start_pub = nh.advertise<std_msgs::Bool>("amr_start",1);
    msg_amr_start.data = false;
    cy_action_info = {false, false, 0, 0};

    is_save_data_log = false;

    //Init alarm info
    //alm_on code display auto_reset
    alarm_info.devices.imu     = {false, "D-100", "", false, -1};
    alarm_info.devices.lidar   = {false, "D-200", "", false, -1};
    alarm_info.devices.om_cart = {false, "D-300", "", false, -1};
    alarm_info.devices.joy     = {false, "D-400", "", true,  -1};
    alarm_info.devices.usb_io  = {false, "D-500", "", false, -1};
    alarm_info.low_volt        = {false, "B-100", "", true,  -1};
    alarm_info.over_load       = {false, "S-100", "", false, -1};
    alarm_info.emergen_stop    = {false, "E-100", "", true,  -1};
    alarm_info.om_driver       = {false, "ML-", "", false,   -1}; //ML-100,MR-100, ML-100:MR-100

    //Init running state info.
    amr_state_info = {AMR_STATE::STOPPED, false, false};

    //Setup MultiMedia
    sound_info.music_run = new QSound("/home/mikuni/catkin_ws/src/amr_ros/sound/music_running.wav");
    sound_info.music_run->setLoops(QSound::Infinite);
    sound_info.music_run_on = false;
    sound_info.music_alarm = new QSound("/home/mikuni/catkin_ws/src/amr_ros/sound/music_alarm.wav");
    sound_info.music_alarm->setLoops(QSound::Infinite);
    sound_info.music_alm_on = false;

    //set cart S_ON
    pub_cart_s_on = nh.advertise<std_msgs::Bool>("cart_s_on",1);
    is_set_s_on.data = true;

}

Amr_Default_Gui::~Amr_Default_Gui()
{
    if(status_log_name.isOpen())
    {
        status_log_name.close();
    }

    if(om_process->processId() > 0)
    {
        execute_shell_cmd("kill " + QString::number(om_process->processId()));
    }

    if(usbio_process->processId() > 0)
    {
        execute_shell_cmd("kill " + QString::number(usbio_process->processId()));
    }

    execute_shell_cmd("killall roslaunch");

    delete ui;
}

void Amr_Default_Gui::start_om_cart()
{
    if(om_process->processId() == 0)
    {
        execute_app_cmd(om_process,"roslaunch amr_ros om_cart_default.launch");
        alarm_info.devices.om_cart.err_timer = 0;
    }
}

void Amr_Default_Gui::start_usbio_board()
{
    if(usbio_process->processId() == 0)
    {
        execute_app_cmd(usbio_process,"roslaunch usbio_driver usbio_driver.launch");
        alarm_info.devices.usb_io.err_timer = 0;
    }
}

void Amr_Default_Gui::init_config_file(QString fileName)
{
    QFile file;
    file.setFileName(fileName);
    QString strTmp;

    //Open file
    if(file.exists())
    {
        file.open(QIODevice::ReadOnly  | QIODevice::Text);
    }
    else
    {
        //Show Error Message
        save_log("Init config file Error!");
        return;
    }

    //Set parameters from config file
    QTextStream inFile(&file);
    while(!inFile.atEnd())
    {
        if((strTmp = inFile.readLine().trimmed()) == "") continue;
        if(strTmp.at(0) == '#') continue;

        QString str = (strTmp.split("#")).at(0);
        QStringList item_info = str.split("=");
        //ros node
        if(amr_config_info.items_name.node_name.contains(item_info.at(0)))
        {
            amr_config_info.items_val.node_name = item_info.at(1);
            continue;
        }
        if(amr_config_info.items_name.work_path.contains(item_info.at(0)))
        {
            amr_config_info.items_val.work_path = item_info.at(1);
            continue;
        }
        if(amr_config_info.items_name.package_name.contains(item_info.at(0)))
        {
            amr_config_info.items_val.package_name = item_info.at(1);
            continue;
        }
        if(amr_config_info.items_name.map_data_name_prefix.contains(item_info.at(0)))
        {
            amr_config_info.items_val.map_data_name_prefix = item_info.at(1);
            continue;
        }
        //log file path
        if(amr_config_info.items_name.data_log_name_prefix.contains(item_info.at(0)))
        {
            amr_config_info.items_val.data_log_name_prefix = item_info.at(1);
            continue;
        }
        if(amr_config_info.items_name.status_log_name_prefix.contains(item_info.at(0)))
        {
            amr_config_info.items_val.status_log_name_prefix = item_info.at(1);
            continue;
        }
        //low battery
        if(amr_config_info.items_name.low_battery_volt.contains(item_info.at(0)))
        {
            amr_config_info.items_val.low_battery_volt = item_info.at(1).toInt();
            continue;
        }
        if(amr_config_info.items_name.low_battery_volt_threshold.contains(item_info.at(0)))
        {
            amr_config_info.items_val.low_battery_volt_threshold = item_info.at(1).toInt();
            continue;
        }
        if(amr_config_info.items_name.low_battery_volt_stop.contains(item_info.at(0)))
        {
            amr_config_info.items_val.low_battery_volt_stop = item_info.at(1).toInt();
            continue;
        }
        if(amr_config_info.items_name.low_battery_volt_reset.contains(item_info.at(0)))
        {
            amr_config_info.items_val.low_battery_volt_reset = item_info.at(1).toInt();
            continue;
        }
        //overload
        if(amr_config_info.items_name.overload_wait_time.contains(item_info.at(0)))
        {
            amr_config_info.items_val.overload_wait_time = item_info.at(1).toInt();
            continue;
        }

    }

    work_path = amr_config_info.items_val.work_path;

    if(file.isOpen())
    {
        file.close();
    }
    setup_log();
    save_log("Init config file Completed!");
}

void Amr_Default_Gui::setup_log()
{
    //Make Dir
    QDir dir;
    int year, month, day;
    year = QDate::currentDate().year();
    month = QDate::currentDate().month();
    day = QDate::currentDate().day();

    QString dir_path = work_path + "/" + amr_config_info.items_val.package_name + "/log/" + QString::number(year) +"_" + QString::number(month) + "_" + QString::number(day);
    dir.mkpath(dir_path);

    //create log file
    QTime curr_time = QTime::currentTime();
    status_log_name.setFileName(dir_path + "/" + amr_config_info.items_val.status_log_name_prefix + "_"+ QString::number(curr_time.hour())
                                + "_" + QString::number(curr_time.minute()) + "_" + QString::number(curr_time.second()) + ".txt");
    status_log_name.open(QIODevice::NewOnly | QIODevice::Text);

}

void Amr_Default_Gui::save_log(QString str)
{
    QTextStream outFile(&status_log_name);

    outFile << QTime::currentTime().toString() << ":" << str << "\r\n";
}

void Amr_Default_Gui::setup_running_data_log()
{
    QTime curr_time = QTime::currentTime();
    QString fileName = work_path + "/" +amr_config_info.items_val.package_name + "/log/"
                      + QString::number(QDate::currentDate().year())+"_"+QString::number(QDate::currentDate().month()) + "_"
                      + QString::number(QDate::currentDate().day()) + "/" + amr_config_info.items_val.data_log_name_prefix + "_"
                      + QString::number(curr_time.hour()) + "_" + QString::number(curr_time.minute()) + "_" + QString::number(curr_time.second()) + ".csv";

    running_data_file.setFileName(fileName);
    //create new file
    running_data_file.open(QIODevice::NewOnly | QIODevice::WriteOnly | QIODevice::Text);
    QTextStream outFile(&running_data_file);

    //save items name
    for(QString strTmp : log_item_name)
    {
        outFile << strTmp << ";";
    }
    outFile << "\n";

    log_start_time = QTime::currentTime();
    is_save_data_log = true;
    save_running_data_timer = 0;

}

void Amr_Default_Gui::close_running_data_log()
{
    if(running_data_file.exists())
    {
        running_data_file.close();
    }
    is_save_data_log = false;
}

void Amr_Default_Gui::save_running_data()
{
    QTextStream outFile(&running_data_file);

    log_run_info.rec_time = QString::number((QTime::currentTime().msecsSinceStartOfDay() - log_start_time.msecsSinceStartOfDay()) / 1000.0);
    outFile << log_run_info.rec_time << ";";
    outFile << log_run_info.vel_line << ";" << log_run_info.vel_theta << ";";
    outFile << log_run_info.vel_left << ";" << log_run_info.vel_right << ";";
    outFile << log_run_info.pose_x << ";"   << log_run_info.pose_y << ";" ;
    outFile << log_run_info.pose_theta << ";" << log_run_info.emergen_stop << ";";
    outFile << log_run_info.bat_volt << ";" << log_run_info.bat_total_curr << ";";
    outFile << log_run_info.bat_curr_left << ";" << log_run_info.bat_curr_right << ";";
    outFile << log_run_info.alarm_info << ";" << log_run_info.alert_info << ";" << "\r\n";

}

void Amr_Default_Gui::set_serial_info()
{
    QString path = work_path + "/" + amr_config_info.items_val.package_name +  "/config/serial_port_info.conf";

    execute_shell_cmd("dmesg | grep ttyUSB* > " + path );
    execute_shell_cmd("dmesg | grep ttyACM* >> " + path );
    execute_shell_cmd("lsusb >> " + path );
    update_serial_port(path);

}

void Amr_Default_Gui::update_serial_port(QString fileName)
{
    QFile file;
    file.setFileName(fileName);
    QString strTmp;
    bool imu_exist = false;
    bool usbio_exist = false;

    //Open or create new file
    if(file.exists())
    {
        file.open(QIODevice::ReadOnly  | QIODevice::Text);
    }
    else
    {
        //todo
    }

    /*---------------------------------------------------
     * cp210x - Lidar - ttyUSB0/ttyUSB1/disconnected
     * FTDI - Cart - ttyUSB0/ttyUSB1/disconnected
     * ------------------------------------------------*/
    QTextStream inFile(&file);
    while(!inFile.atEnd())
    {
        if((strTmp = inFile.readLine().trimmed()) == "") continue;
        if(strTmp.indexOf(":") <= 0) continue;

        QString str = (strTmp.split(":")).at(0);
        //check ttyUSB devices
        if(str.contains("usb"))
        {
            str = (strTmp.split(":")).at(1);

            if(str.contains("cp210x")) //Lidar
            {
                if(str.contains("attached"))
                {
                    if(str.contains("ttyUSB0"))
                    {
                        lidar_port = "/dev/ttyUSB0";
                    }
                    else if(str.contains("ttyUSB1"))
                    {
                        lidar_port = "/dev/ttyUSB1";
                    }
                }
                else if(str.contains("disconnected"))
                {
                    lidar_port = "None";
                }
            }
            else if(str.contains("FTDI")) //cart
            {
                if(str.contains("attached"))
                {
                    if(str.contains("ttyUSB0"))
                    {
                        cart_port = "/dev/ttyUSB0";
                    }
                    else if(str.contains("ttyUSB1"))
                    {
                        cart_port = "/dev/ttyUSB1";
                    }
                }
                else if(str.contains("disconnected"))
                {
                    cart_port = "None";
                }
            }
        }
#if 1 //note PC
        //check ttyACM device
        else if(str.contains("cdc_acm")) //keyword: cdc-acm
        {
            QStringList strlist = strTmp.split(":");
            if(strlist.at(0).contains("-2.3")) //usb-io
            {
                usbio_port = "/dev/" + strlist.at(2).trimmed();
            }
            if(strlist.at(0).contains("-2.1")) //imu
            {
                imu_port = "/dev/" + strlist.at(2).trimmed();
            }
        }
#else
        //check ttyACM device
        else if(str.contains("cdc_acm")) //keyword: cdc-acm
        {
            QStringList strlist = strTmp.split(":");
            if(strlist.at(0).contains("-1.3")) //usb-io
            {
                usbio_port = "/dev/" + strlist.at(2).trimmed();
            }
            if(strlist.at(0).contains("-1.1")) //imu
            {
                imu_port = "/dev/" + strlist.at(2).trimmed();
            }
        }
#endif        //check imu or usb-io exist or not
        else if(str.contains("Bus") && str.contains("Device"))
        {
            QStringList strid = strTmp.split(":");
            if(strid.at(1).contains("2b72") && strid.at(2).contains("0003")) //imu id: 2b72-0003
            {
                imu_exist = true;
            }
            if(strid.at(1).contains("2a19")&& strid.at(2).contains("0800")) //usb-io id: 2a19-0800
            {
                usbio_exist = true;
            }
        }
    }

    //Update ros parameters
    nh.setParam("om_modbusRTU_1/init_com",cart_port.toStdString());  //cart com port
    nh.setParam("rplidarNode/serial_port",lidar_port.toStdString()); //lidar com port
    if(imu_exist)
    {
        nh.setParam("/rt_usb_9axisimu_driver/port",imu_port.toStdString()); //imu comm port
    }
    if(usbio_exist)
    {
        nh.setParam("/usb_io_node/usbioPort", usbio_port.toStdString()); //usbio comm port
    }

    //Show on GUI
    if(cart_port.contains("None"))
    {
        ui->label_cart_port->setText("CART: " + cart_port);
    }
    else
    {
        ui->label_cart_port->setText("CART: " + cart_port.right(7));
    }

    if(lidar_port.contains("None"))
    {
        ui->label_Lidar_port->setText("LIDAR: " + lidar_port);
    }
    else
    {
        ui->label_Lidar_port->setText("LIDAR: " + lidar_port.right(7));
    }

    if(imu_exist)
    {
        ui->label_imu_port->setText("IMU: " + imu_port.right(7));
    }
    else
    {
        ui->label_imu_port->setText("IMU: None");
    }

    if(usbio_exist)
    {
        ui->label_usbio_port->setText("USB-IO: " + usbio_port.right(7));
    }
    else
    {
        ui->label_usbio_port->setText("USB-IO: None");
    }

    if(file.isOpen())
    {
        file.close();
    }
}

void Amr_Default_Gui::spinOnce()
{
    //Wait for stopping
    if(wait_stop_tmr >= 0)
    {
        wait_stop_tmr++;

        //check waiting conditions
        if((wait_stop_tmr > MAX_STOP_TIME) || (   (proc_amr[0]->processId() == 0)
                                               && (proc_amr[1]->processId() == 0)
                                               && (proc_amr[2]->processId() == 0)))
        {
            //Force to complete, Show error message
            if(wait_stop_tmr >= MAX_STOP_TIME)
            {
                ui->label_sys_msg->setText("Note: Stop [" + func_name + "] App Failed!");
            }
            else
            {
                ui->label_sys_msg->setText("Note: Stop [" + func_name + "] App Completed!");
            }
            wait_stop_tmr = -1;
            is_amr_on = false;
            pressed_btn->setText(func_name);
            amr_state_info.amr_state = AMR_STATE::STOPPED;
            enable_all_buttons();
        }

    }

    //Save data to log
    if(is_save_data_log == true)
    {
        save_running_data_timer++;
        if(save_running_data_timer > DATA_LOG_MAX_TIME)
        {
            save_running_data_timer = 0;
            save_running_data();
        }
    }

    //check and set cylinder output state
    check_cy_output();

    //check and set error
    check_alarm_state();

    if(ros::ok())
    {
        ros::spinOnce();
    }
    else
        QApplication::quit();
}

void Amr_Default_Gui::check_cy_output()
{
    //When to start at waypoint
    if(cy_action_info.ftsw_started)
    {
        cy_action_info.cy_front_timer++;

        if(cy_action_info.cy_front_timer > CY_TIMING_AMR_START) //To start AMR
        {
            cy_action_info.ftsw_started = false;
            cy_action_info.cy_front_timer = 0;
            //publish to start amr
            msg_amr_start.data = true;
            amr_start_pub.publish(msg_amr_start);

        }
        else if(cy_action_info.cy_front_timer > CY_TIMING_FRONT_OFF) //To set cy front channel to OFF
        {
            set_usb_port(IO_NUM_DEVICE::USB_IO_CYLINDER_FRONT, IO_STATE::IO_OFF);
        }
        else if(cy_action_info.cy_front_timer > CY_TIMING_FRONT_ON) //To set cy front channel to ON
        {
            set_usb_port(IO_NUM_DEVICE::USB_IO_CYLINDER_FRONT, IO_STATE::IO_ON);
        }
        else
        {
            //Do Nothing
        }
    }

    //when to stop at waypoint
    if(cy_action_info.waypoint_stopped)
    {
        cy_action_info.cy_back_timer++;

        if(cy_action_info.cy_back_timer > CY_TIMING_BACK_OFF) //To set back channel to OFF
        {
            cy_action_info.waypoint_stopped = false;
            cy_action_info.cy_back_timer = 0;
            set_usb_port(IO_NUM_DEVICE::USB_IO_CYLINDER_BACK, IO_STATE::IO_OFF);
        }
        else if(cy_action_info.cy_back_timer > CY_TIMING_BACK_ON) //To set cy front channel to OFF
        {
            set_usb_port(IO_NUM_DEVICE::USB_IO_CYLINDER_BACK, IO_STATE::IO_ON);
        }
        else
        {
            //Do Nothing
        }
    }
}

void Amr_Default_Gui::check_alarm_state()
{
    QVector<Alm_Item> alarm_list;

    //check om-cart communication error
    if(alarm_info.devices.om_cart.err_timer >= 0)
    {
        alarm_info.devices.om_cart.err_timer++;
        if(alarm_info.devices.om_cart.err_timer > MAX_ERROR_TIME)
        {
            alarm_info.devices.om_cart.err_timer = -1; //stop to monitor communication error
            alarm_info.devices.om_cart.alm_on = true;
            alarm_info.devices.om_cart.code = "D-300";
            ui->label_cart_status->setText("NG");
            ui->label_cart_status->setStyleSheet("color:red");
            ui->textBrowser->append(alarm_info.devices.om_cart.code);
            ui->textBrowser->append("OM-Cart Communication Error.\n Check Cart Power or Communication Cabel.");
            ui->textBrowser->setStyleSheet("color:red");

            //stop motor first.
            if(om_process->processId() != 0)
            {
                execute_shell_cmd("kill " + QString::number(om_process->processId()));
                ui->pushButton_reset_alarm->setDisabled(true);
                ui->label_sys_msg->setText("Note: Stopping om_cart driver. Please wait.");
            }
            alarm_list.append(alarm_info.devices.om_cart);
        }
    }
    //check lidar communication error
    if(alarm_info.devices.lidar.err_timer >= 0)
    {
        alarm_info.devices.lidar.err_timer++;
        if(alarm_info.devices.lidar.err_timer > MAX_ERROR_TIME)
        {
            alarm_info.devices.lidar.err_timer = -1; //stop to monitor communication error
            if(amr_state_info.amr_state != AMR_STATE::STOPPING)
            {
                alarm_info.devices.lidar.alm_on = true;
                alarm_info.devices.lidar.code = "D-200";
                ui->label_lidar_status->setText("NG");
                ui->label_lidar_status->setStyleSheet("color:red");

                ui->textBrowser->append(alarm_info.devices.lidar.code);
                ui->textBrowser->append("Lidar Communication Error.\n Check Lidar Power or Communication Cabel.");
                ui->textBrowser->setStyleSheet("color:red");
                alarm_list.append(alarm_info.devices.lidar);
            }
        }
    }

    //check imu communication error
    if(alarm_info.devices.imu.err_timer >= 0)
    {
        alarm_info.devices.imu.err_timer++;
        if(alarm_info.devices.imu.err_timer > MAX_ERROR_TIME)
        {
            alarm_info.devices.imu.err_timer = -1; //stop to monitor communication error
            if(amr_state_info.amr_state != AMR_STATE::STOPPING)
            {
                alarm_info.devices.imu.alm_on = true;
                alarm_info.devices.imu.code = "D-100";
                ui->label_imu_status->setText("NG");
                ui->label_imu_status->setStyleSheet("color:red");

                ui->textBrowser->append(alarm_info.devices.imu.code);
                ui->textBrowser->append("IMU Communication Error.\n Check IMU Power or Communication Cabel.");
                ui->textBrowser->setStyleSheet("color:red");
                alarm_list.append(alarm_info.devices.imu);
            }
        }
    }

    //check usb-io board communication error
    if(alarm_info.devices.usb_io.err_timer >= 0)
    {
        alarm_info.devices.usb_io.err_timer++;
        if(alarm_info.devices.usb_io.err_timer > MAX_ERROR_TIME)
        {
            alarm_info.devices.usb_io.err_timer = -1; //stop to monitor communication error
            alarm_info.devices.usb_io.alm_on = true;
            alarm_info.devices.usb_io.code = "D-500";
            ui->label_usbio_status->setText("NG");
            ui->label_usbio_status->setStyleSheet("color:red");

            ui->textBrowser->append(alarm_info.devices.usb_io.code);
            ui->textBrowser->append("USB-IO Board Communication Error.\n Check USB-IO Board Power or Communication Cabel.");
            ui->textBrowser->setStyleSheet("color:red");
            //stop usbio driver
            if(usbio_process->processId() != 0)
            {
                execute_shell_cmd("kill " + QString::number(usbio_process->processId()));
            }
            alarm_list.append(alarm_info.devices.usb_io);
        }
    }

    //Action
    //stop amr and show error message
    if(   (amr_state_info.amr_state == AMR_STATE::AUTO_RUN)
       || (amr_state_info.amr_state == AMR_STATE::COLLECT_DATA)
       || (amr_state_info.amr_state == AMR_STATE::MANUAL) )
    {
        if(alarm_info.devices.om_cart.alm_on)
        {
            //stop app
            if(amr_state_info.amr_state == AMR_STATE::AUTO_RUN)
            {
                on_pushButton_start_amr_clicked();
            }
            else if(amr_state_info.amr_state == AMR_STATE::COLLECT_DATA)
            {
                on_pushButton_collect_data_clicked();
            }
            else if(amr_state_info.amr_state == AMR_STATE::MANUAL)
            {
                on_pushButton_run_manual_clicked();
            }

            amr_state_info.alarm_on = true;
        }

        if(alarm_info.devices.lidar.alm_on && ((amr_state_info.amr_state == AMR_STATE::AUTO_RUN) || (amr_state_info.amr_state == AMR_STATE::COLLECT_DATA)))
        {
            //stop app
            if(amr_state_info.amr_state == AMR_STATE::AUTO_RUN)
            {
                on_pushButton_start_amr_clicked();
            }
            else if(amr_state_info.amr_state == AMR_STATE::COLLECT_DATA)
            {
                on_pushButton_collect_data_clicked();
            }

            amr_state_info.alarm_on = true;
        }

        if(alarm_info.devices.imu.alm_on && (amr_state_info.amr_state == AMR_STATE::COLLECT_DATA))
        {
            on_pushButton_run_manual_clicked();
            amr_state_info.alarm_on = true;
        }

        if(alarm_info.devices.usb_io.alm_on)
        {
            //stop app
            if(amr_state_info.amr_state == AMR_STATE::AUTO_RUN)
            {
                on_pushButton_start_amr_clicked();
            }
            else if(amr_state_info.amr_state == AMR_STATE::COLLECT_DATA)
            {
                on_pushButton_collect_data_clicked();
            }
            amr_state_info.alarm_on = true;
        }

    }
    else if(   (amr_state_info.amr_state == AMR_STATE::STOPPED)
            || (amr_state_info.amr_state == AMR_STATE::STOPPING))
    {
        //check om_cart and usb-io board in stopped state
        if(alarm_info.devices.om_cart.alm_on || alarm_info.devices.usb_io.alm_on)
        {
            amr_state_info.alarm_on = true;

        }
    }

    //set lamp ON/OFF
    if(amr_state_info.alarm_on == true)
    {
        if(usbio_info.items.lamp_alarm == IO_STATE::IO_OFF)
        {
            set_usb_port(IO_NUM_DEVICE::USB_IO_LAMP_NORMAL, IO_STATE::IO_OFF);
            set_usb_port(IO_NUM_DEVICE::USB_IO_LAMP_ALARM, IO_STATE::IO_ON);
            usbio_info.items.lamp_alarm = IO_STATE::IO_ON;
        }
        if(sound_info.music_alm_on == false)
        {
            sound_info.music_run->stop();
            sound_info.music_run_on = false;
            sound_info.music_alm_on = true;
            sound_info.music_alarm->play();

        }

    }
    else
    {
        if(usbio_info.items.lamp_normal == IO_STATE::IO_OFF)
        {
            set_usb_port(IO_NUM_DEVICE::USB_IO_LAMP_ALARM, IO_STATE::IO_OFF);
            set_usb_port(IO_NUM_DEVICE::USB_IO_LAMP_NORMAL, IO_STATE::IO_ON);
            usbio_info.items.lamp_normal == IO_STATE::IO_ON;
        }
        if(sound_info.music_alm_on == true)
        {
            sound_info.music_alm_on = false;
            sound_info.music_alarm->stop();
        }

    }

    //show error code
    for(Alm_Item tmp : alarm_list)
    {
        if(tmp.alm_on)
        {
            ui->label_error_code_num->setText(tmp.code);
            ui->label_error_code_num->setStyleSheet("color:red");
        }
    }
    alarm_list.clear();
}

void Amr_Default_Gui::set_usb_port(uint8_t num, uint8_t val)
{
    USBIO_cmd_info cmd_info;
    std_msgs::ByteMultiArray msg_pub;

    cmd_info.items.io_num = num;
    cmd_info.items.value = val;
    cmd_info.items.comm_reset = 0;

    msg_pub.data.resize(USBIO_CMD_DATA_SIZE);
    for(int i = 0; i < USBIO_CMD_DATA_SIZE; i++)
    {
        msg_pub.data[i] = cmd_info.data[i];
    }

    usbio_cmd_pub.publish(msg_pub);
}

double Amr_Default_Gui::trans_q(double theta)
{
    while (theta > M_PI)
        theta -= 2.0 * M_PI;
    while (theta < -M_PI)
        theta += 2.0 * M_PI;

    return theta;
}

void Amr_Default_Gui::pose_Callback(const gnd_msgs::msg_pose2d_stamped::ConstPtr& msg)
{
    log_run_info.pose_x = QString::number(msg->x,'f',3);
    log_run_info.pose_y = QString::number(msg->y,'f',3);
    log_run_info.pose_theta = QString::number(gnd_rad2deg(trans_q(msg->theta)),'f',2);
}

void Amr_Default_Gui::amrStatusCallback(const gnd_msgs::msg_vehicle_status::ConstPtr& status)
{
    amr_status = status->status;

}

void Amr_Default_Gui::targetArrivedCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data == true)
    {
        if(amr_state_info.amr_state == AMR_STATE::AUTO_RUN)
        {
            cy_action_info.waypoint_stopped = true;
            //music off
            if(sound_info.music_run_on == true)
            {
                sound_info.music_run_on = false;
                sound_info.music_run->stop();
            }

        }
    }
}

void Amr_Default_Gui::usbioStatusCallback(const std_msgs::ByteMultiArray::ConstPtr& status)
{    
    alarm_info.devices.usb_io.err_timer = 0; //reset timer

    for(int i = 0; i < USBIO_STATUS_DATA_SIZE; i++)
    {
        usbio_info.data[i] = status->data.at(i);
    }

    //show status on GUI
    if(usbio_info.items.status != 0)
    {
        //Todo
    }
    else
    {
        //lamp normal
        ui->label_usbio_normal_status->setText(USBIO_OUT_STATE.at(usbio_info.items.lamp_normal));
        if(usbio_info.items.lamp_normal == 0)
        {
            ui->label_usbio_normal_status->setStyleSheet("color:black");
        }
        else
        {
            ui->label_usbio_normal_status->setStyleSheet("color:green");
        }
        //lamp alarm
        ui->label_usbio_alarm_status->setText(USBIO_OUT_STATE.at(usbio_info.items.lamp_alarm));
        if(usbio_info.items.lamp_alarm == 0)
        {
            ui->label_usbio_alarm_status->setStyleSheet("color:black");
        }
        else
        {
            ui->label_usbio_alarm_status->setStyleSheet("color:red");
        }
        //cylinder front
        ui->label_usbio_cy_front_status->setText(USBIO_OUT_STATE.at(usbio_info.items.cylinder_front));
        if(usbio_info.items.cylinder_front == 0)
        {
            ui->label_usbio_cy_front_status->setStyleSheet("color:black");
        }
        else
        {
            ui->label_usbio_cy_front_status->setStyleSheet("color:green");
        }
        //cylinder back
        ui->label_usbio_cy_back_status->setText(USBIO_OUT_STATE.at(usbio_info.items.cylinder_back));
        if(usbio_info.items.cylinder_back == 0)
        {
            ui->label_usbio_cy_back_status->setStyleSheet("color:black");
        }
        else
        {
            ui->label_usbio_cy_back_status->setStyleSheet("color:green");
        }
        //foot switch
        ui->label_usbio_feet_sw_status->setText(USBIO_IN_STATE.at(usbio_info.items.feet_switch));
        if(usbio_info.items.feet_switch > 0) //switch open
        {
            ui->label_usbio_feet_sw_status->setStyleSheet("color:black");
            //check foot switch state to startup AMR
            if(ftsw_start_info.sw_close_timer > KEEP_CLOSED_100_MS) //start: when CLOSED=>OPEN
            {
                ftsw_start_info.amr_start = true;
                if((amr_state_info.alarm_on == false) && (   (amr_status == gnd_msgs::msg_vehicle_status::VEHICLE_STATE_IDLE)
                                                          || (amr_status == gnd_msgs::msg_vehicle_status::VEHICLE_STATE_RUN) ))
                {
                    if(amr_state_info.amr_state == AMR_STATE::STOPPED)
                    {
                        on_pushButton_start_amr_clicked();
                        //publish to start amr
                        msg_amr_start.data = true;
                        amr_start_pub.publish(msg_amr_start);
                    }
                    else if(amr_state_info.amr_state == AMR_STATE::AUTO_RUN)
                    {
                        cy_action_info.ftsw_started = true;
                        //music on
                        if(sound_info.music_run_on == false)
                        {
                            sound_info.music_run_on = true;
                            sound_info.music_run->play();
                        }
                    }
                }

            }
            msg_amr_start.data = false;
            ftsw_start_info.sw_close_timer = 0;
        }
        else //foot switch close
        {
            ui->label_usbio_feet_sw_status->setStyleSheet("color:green");
            ftsw_start_info.sw_close_timer++;

        }

    }

}

void Amr_Default_Gui::cartStatusCallback(const om_cart::om_cart_state::ConstPtr& status)
{
    double power_volt;

    alarm_info.devices.om_cart.err_timer = 0; //reset timer

    if(status->type != CART_STATUS)
    {
        return; //skip all non-status message
    }

    for(int i = 0; i < status->size; i++)
    {
        cart_status_info.data[i] = status->data[i];
    }

    //Update battery info
    power_volt = (qRound(cart_status_info.cart_status.main_power_volt_L + cart_status_info.cart_status.main_power_volt_R)) * 1.0 / (10*2);
    ui->label_volt_main_val->setText(QString::number(power_volt,'f',1));
    ui->label_curr_main_val->setText(QString::number(qRound((cart_status_info.cart_status.main_power_curr_L + cart_status_info.cart_status.main_power_curr_R)/10) * 1.0 / 100.0,'f',2));
    ui->label_curr_left_val->setText(QString::number(qRound(cart_status_info.cart_status.main_power_curr_L / 10) * 1.0 / 100.0,'f',2));
    ui->label_curr_right_val->setText(QString::number(qRound(cart_status_info.cart_status.main_power_curr_R /10) * 1.0 / 100.0,'f',2));

    //check motor driver of emergency stop input
    if((int)cart_status_info.cart_status.emergen_stop != 0)
    {
        ui->label_emergen_stop_status->setText("ON");
        ui->label_emergen_stop_status->setStyleSheet("color:red");
    }
    else
    {
        ui->label_emergen_stop_status->setText("Normal");
        ui->label_emergen_stop_status->setStyleSheet("color:black");
    }

    //check om motor error
    if(((int)cart_status_info.cart_status.alm_code_L > 0) || ((int)cart_status_info.cart_status.alm_code_R > 0))
    {
        alarm_info.om_driver.alm_on = true;
        alarm_info.om_driver.code = "ML-" + QString::number((int)cart_status_info.cart_status.alm_code_L,16) + ":" +
                                    "MR-" + QString::number((int)cart_status_info.cart_status.alm_code_R,16);
        ui->textBrowser->append(alarm_info.om_driver.code);
        ui->label_error_code_num->setText(alarm_info.om_driver.code);
        ui->label_error_code_num->setStyleSheet("color:red");
    }

    //check battery low voltage
    if(power_volt < (amr_config_info.items_val.low_battery_volt - amr_config_info.items_val.low_battery_volt_threshold) * 1.0 / 1000.0)
    {
        alarm_info.low_volt.alm_on = true;
        alarm_info.low_volt.code = "B-100";
        ui->label_low_volt_status->setText("ON");
        ui->label_low_volt_status->setStyleSheet("color:red");
        //show error code
        ui->textBrowser->append(alarm_info.low_volt.code);
        ui->textBrowser->append("Low Battery.");
        ui->label_error_code_num->setText(alarm_info.low_volt.code);
        ui->label_error_code_num->setStyleSheet("color:red");

    }
    else if(power_volt > (amr_config_info.items_val.low_battery_volt + amr_config_info.items_val.low_battery_volt_threshold) * 1.0 / 1000.0)
    {
        alarm_info.low_volt.alm_on = false;
        alarm_info.low_volt.code = "-";
        ui->label_low_volt_status->setText("Normal");
        ui->label_low_volt_status->setStyleSheet("color:black");
    }

    //Save data log
    log_run_info.vel_line = QString::number(cart_status_info.cart_status.vel_line,'f',2);
    log_run_info.vel_theta = QString::number(cart_status_info.cart_status.vel_theta,'f',2);
    log_run_info.vel_left = QString::number(cart_status_info.cart_status.vel_left / 1000,'f',2);
    log_run_info.vel_right = QString::number(cart_status_info.cart_status.vel_right / 1000,'f',2);
    log_run_info.bat_volt = ui->label_volt_main_val->text();
    log_run_info.bat_total_curr = ui->label_curr_main_val->text();
    log_run_info.bat_curr_left = ui->label_curr_left_val->text();
    log_run_info.bat_curr_right = ui->label_curr_right_val->text();
    log_run_info.emergen_stop = ui->label_emergen_stop_status->text();
    log_run_info.alarm_info = "ML-"+QString::number((int)cart_status_info.cart_status.alm_code_L)+":" + "MR-:"+QString::number((int)cart_status_info.cart_status.alm_code_R);

}

void Amr_Default_Gui::lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    alarm_info.devices.lidar.err_timer = 0; //reset
}

void Amr_Default_Gui::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    alarm_info.devices.imu.err_timer = 0; //reset
}

QString Amr_Default_Gui::get_output_str(QProcess *proc)
{
    QByteArray encodeString = proc->readAllStandardOutput();
    QTextCodec *codec = QTextCodec::codecForLocale();
    QTextDecoder *decoder = codec->makeDecoder();
    QString str = decoder->toUnicode(encodeString);

    return str;

}

QString Amr_Default_Gui::get_err_str(QProcess *proc)
{
    QByteArray encodeString = proc->readAllStandardError();
    QTextCodec *codec = QTextCodec::codecForLocale();
    QTextDecoder *decoder = codec->makeDecoder();
    QString str =  decoder->toUnicode(encodeString);

    return str;
}

void Amr_Default_Gui::on_readoutput_om_cart()
{

}

void Amr_Default_Gui::on_readerror_om_cart()
{

}

void Amr_Default_Gui::on_finished_om_cart(int exitCode)
{
    qDebug() << "OM-CART:finished!";
    ui->pushButton_reset_alarm->setEnabled(true);
    ui->label_sys_msg->setText("Note: Stop om_cart completed.");

}

void Amr_Default_Gui::on_readoutput0()
{
    QString str = get_output_str(proc_amr[0]);

}

void Amr_Default_Gui::on_readerror0()
{   
    QString str = get_err_str(proc_amr[0]);
    QString error_info = "";

    //check all device error
    if(str.contains(amr_keyword.at(DEVICE_KEYWORD::IMU)))
    {
        if(amr_state_info.amr_state == AMR_STATE::COLLECT_DATA)
        {
            //imu error
            alarm_info.devices.imu.alm_on = true;
            ui->label_imu_status->setText("NG");
            ui->label_imu_status->setStyleSheet("color:red");
            alarm_info.devices.imu.code = "D-101";

            error_info = alarm_info.devices.imu.code + ":IMU Error. Please check imu status.\n";
        }
    }
    else if(str.contains(amr_keyword.at(DEVICE_KEYWORD::LIDAR)))
    {
        if((amr_state_info.amr_state == AMR_STATE::COLLECT_DATA) || (amr_state_info.amr_state == AMR_STATE::AUTO_RUN))
        {
            //lidar error
            alarm_info.devices.lidar.alm_on = true;

            ui->label_lidar_status->setText("NG");
            ui->label_lidar_status->setStyleSheet("color:red");
            alarm_info.devices.lidar.code = "D-201";
            error_info = alarm_info.devices.lidar.code + ":Lidar Error. Please check Lidar status.\n";
        }
    }
    else if(str.contains(amr_keyword.at(DEVICE_KEYWORD::JOYSTICK)))
    {
        if((amr_state_info.amr_state == AMR_STATE::COLLECT_DATA) || (amr_state_info.amr_state == AMR_STATE::COLLECT_DATA))
        {
            if(str.contains("ERROR"))
            {
                //joystick error
                alarm_info.devices.joy.alm_on = true;
                ui->label_joystick_status->setText("NG");
                ui->label_joystick_status->setStyleSheet("color:red");
                alarm_info.devices.joy.code = "D-401";
                error_info = alarm_info.devices.joy.code + ":Joystick Error. Please Reconnect or Check Receiver Power.\n";
            }
            else
            {
                //joystick return Normal
                alarm_info.devices.joy.alm_on = false;
                ui->label_joystick_status->setText("OK");
                ui->label_joystick_status->setStyleSheet("color:black");
                alarm_info.devices.joy.code = "-";
                ui->textBrowser->append("Joystick Return to Normal!");
                ui->textBrowser->setStyleSheet("color:black");

            }
        }

    }
    else if(str.contains(amr_keyword.at(DEVICE_KEYWORD::USB_IO)))
    {
        //usb_io error
        alarm_info.devices.usb_io.alm_on = true;
        ui->label_usbio->setText("NG");
        ui->label_usbio->setStyleSheet("color:red");
        alarm_info.devices.usb_io.code = "D-501";
        error_info = alarm_info.devices.usb_io.code + ":USB_IO Board Error. Please Check usb-io board Power.\n";
    }

    if(error_info.size() > 0)
    {
        ui->textBrowser->append(error_info);
    }
}

void Amr_Default_Gui::on_finished0(int exitCode)
{
    qDebug() << "process 0:finished!" << "exitCode:" << exitCode;
}

void Amr_Default_Gui::on_readoutput1()
{
//    QString str = get_output_str(proc_amr[1]);

}

void Amr_Default_Gui::on_readerror1()
{
    QString str = get_err_str(proc_amr[1]);

}

void Amr_Default_Gui::on_finished1(int exitCode)
{
    qDebug() << "process 1:finished!" << "exitCode:" << exitCode;

}

void Amr_Default_Gui::on_readoutput2()
{
    QString str = get_output_str(proc_amr[2]);
    ui->textBrowser->append(str);

}

void Amr_Default_Gui::on_readerror2()
{
    QString str = get_err_str(proc_amr[2]);

}

void Amr_Default_Gui::on_finished2(int exitCode)
{
    qDebug() << "process 2:finished! " << "exitCode:" << exitCode;

}

void Amr_Default_Gui::execute_app_cmd(QProcess *proc, QString strCmd)
{
    proc->start("bash", QStringList() << "-c" << strCmd);
    proc->waitForStarted();
}

QString Amr_Default_Gui::execute_shell_cmd(QString strCmd)
{
    QProcess proc;
    proc.start("bash", QStringList() << "-c" << strCmd);
    proc.waitForFinished();
    QString strResult = proc.readAllStandardOutput();

    return strResult;
}

void Amr_Default_Gui::kill_all_app_process()
{
    for(QProcess *p_amr : proc_amr)
    {
        if(p_amr->processId() != 0)
        {
            qDebug() << "kill self pid:" << p_amr->processId();
            execute_shell_cmd("kill " + QString::number(p_amr->processId()));

        }
    }
}

void Amr_Default_Gui::enable_all_buttons()
{
    ui->pushButton_start_amr->setEnabled(true);
    ui->pushButton_chk_port->setEnabled(true);
    ui->pushButton_collect_data->setEnabled(true);
    ui->pushButton_make_map->setEnabled(true);
    ui->pushButton_make_root->setEnabled(true);
    ui->pushButton_run_manual->setEnabled(true);

    amr_state_info.cart_move = false;
}

void Amr_Default_Gui::show_message(QString msg)
{
    QMessageBox msgBox;
    msgBox.setText(msg);
    msgBox.exec();
}

void Amr_Default_Gui::on_pushButton_chk_port_clicked()
{
    QString path = work_path + "/" + amr_config_info.items_val.package_name +  "/config/serial_port_info.conf";

    execute_shell_cmd("dmesg | grep ttyUSB* > " + path);
    execute_shell_cmd("dmesg | grep ttyACM* >> " + path);
    execute_shell_cmd("lsusb >> " + path);
    update_serial_port(path);
}

void Amr_Default_Gui::on_pushButton_collect_data_clicked()
{
    if(is_amr_on)
    {
        amr_state_info.amr_state = AMR_STATE::STOPPING;
        alarm_info.devices.imu.err_timer = -1;   //stop monitor communication error
        alarm_info.devices.lidar.err_timer = -1; //stop monitor communication error
        kill_all_app_process();

        wait_stop_tmr = 0;
        ui->pushButton_collect_data->setText("Stopping");
        ui->pushButton_collect_data->setDisabled(true);

        func_name = "Collect Data";
        pressed_btn = ui->pushButton_collect_data;
        save_log("Stopped Collect data!");
    }
    else
    {
        if((amr_state_info.alarm_on == true) && (amr_state_info.cart_move == false))
        {
            show_message("Reset Alarm and Start Again !");
            return;
        }

        is_amr_on = true;
        ui->pushButton_collect_data->setText("Stop Collect");

        ui->pushButton_chk_port->setDisabled(true);
        ui->pushButton_start_amr->setDisabled(true);
        ui->pushButton_make_map->setDisabled(true);
        ui->pushButton_make_root->setDisabled(true);
        ui->pushButton_run_manual->setDisabled(true);
        ui->textBrowser->setText("");

        execute_app_cmd(proc_amr[0],"roslaunch amr_ros om_hw_proxies.launch");

        QString bagName = work_path + "/" + amr_config_info.items_val.package_name + "/maps/" + amr_config_info.items_val.map_data_name_prefix;
        execute_app_cmd(proc_amr[1],"rosbag record -o " + bagName + " vehicle_vel scan imu/data_raw");

        save_log("Started Collect data!");
        amr_state_info.amr_state = AMR_STATE::COLLECT_DATA;
        amr_state_info.cart_move = true;
        alarm_info.devices.lidar.err_timer = 0; //Start to monitor lidar commnunication error
        alarm_info.devices.imu.err_timer = 0;   //Start to monitor imu commnunication error
    }

}

void Amr_Default_Gui::on_pushButton_start_amr_clicked()
{
    if(is_amr_on)
    {
        amr_state_info.amr_state = AMR_STATE::STOPPING;
        alarm_info.devices.lidar.err_timer = -1; //stop to monitor communication error
        kill_all_app_process();

        wait_stop_tmr = 0;
        ui->pushButton_start_amr->setText("Stopping");
        ui->pushButton_start_amr->setDisabled(true);

        func_name = "START AMR";
        pressed_btn = ui->pushButton_start_amr;

        close_running_data_log();
        save_log("Stop Amr!");

        //music off
        if(sound_info.music_run_on == true)
        {
            sound_info.music_run_on = false;
            sound_info.music_run->stop();
        }

    }
    else
    {
        if((amr_state_info.alarm_on == true) && (amr_state_info.cart_move == false))
        {
            show_message("Reset Alarm and Start Again !");
            return;
        }

        is_amr_on = true;
        ui->textBrowser->setText("");
        ui->pushButton_start_amr->setText("STOP AMR");

        ui->pushButton_chk_port->setDisabled(true);
        ui->pushButton_collect_data->setDisabled(true);
        ui->pushButton_make_map->setDisabled(true);
        ui->pushButton_make_root->setDisabled(true);
        ui->pushButton_run_manual->setDisabled(true);
        ui->textBrowser->setText("");

        execute_app_cmd(proc_amr[0],"roslaunch amr_ros om_demo_marker_L.launch");
        QThread::msleep(5000);

        execute_app_cmd(proc_amr[1],"roslaunch amr_ros om_demo_navigation.launch");
        QThread::msleep(1000);

        execute_app_cmd(proc_amr[2],"roslaunch amr_ros om_demo_auto_run.launch");

        setup_running_data_log();

        save_log("Started Amr!");
        amr_state_info.amr_state = AMR_STATE::AUTO_RUN;
        amr_state_info.cart_move = true;
        alarm_info.devices.lidar.err_timer = 0; //Start to monitor lidar commnunication error
        //music on
        if(sound_info.music_run_on == false)
        {
            sound_info.music_run_on = true;
            sound_info.music_run->play();
        }
    }

}


void Amr_Default_Gui::on_pushButton_make_root_clicked()
{
    if(is_amr_on)
    {    
        //Copy new.path to maps folder
        if(QFile::exists("/home/mikuni/new.path"))
        {
            QFile::copy("/home/mikuni/new.path", amr_config_info.items_val.work_path + "/amr_ros/maps/maps.path");
            QFile::remove("/home/mikuni/new.path");
        }
        else
        {
            show_message("Please Save Route File first by Press <ctrl+s> !");
            return;
        }

        amr_state_info.amr_state = AMR_STATE::STOPPING;
        kill_all_app_process();

        wait_stop_tmr = 0;
        ui->pushButton_make_root->setText("Stopping");
        ui->pushButton_make_root->setDisabled(true);

        func_name = "Make Root";
        pressed_btn = ui->pushButton_make_root;

        save_log("Stop Make root!");

    }
    else
    {
        is_amr_on = true;
        ui->textBrowser->setText("");
        ui->pushButton_make_root->setText("Stop Make");

        ui->pushButton_chk_port->setDisabled(true);
        ui->pushButton_start_amr->setDisabled(true);
        ui->pushButton_collect_data->setDisabled(true);
        ui->pushButton_make_map->setDisabled(true);
        ui->pushButton_run_manual->setDisabled(true);

        execute_app_cmd(proc_amr[0],"roslaunch amr_ros route_editor.launch");
        save_log("Started Make root!");
        amr_state_info.amr_state = AMR_STATE::MAKE_ROOT;
        amr_state_info.cart_move = false;

    }

}

void Amr_Default_Gui::on_pushButton_run_manual_clicked()
{
    if(is_amr_on)
    {
        amr_state_info.amr_state = AMR_STATE::STOPPING;
        kill_all_app_process();

        wait_stop_tmr = 0;
        ui->pushButton_run_manual->setText("Stopping");
        ui->pushButton_run_manual->setDisabled(true);

        func_name = "Run by Joy";
        pressed_btn = ui->pushButton_run_manual;
        save_log("Stop manual!");
    }
    else
    {
        if((amr_state_info.alarm_on == true) && (amr_state_info.cart_move == false))
        {
            show_message("Reset Alarm and Start Again !");
            return;
        }

        is_amr_on = true;
        ui->textBrowser->setText("");
        ui->pushButton_run_manual->setText("Stop Run");

        ui->pushButton_chk_port->setDisabled(true);
        ui->pushButton_start_amr->setDisabled(true);
        ui->pushButton_collect_data->setDisabled(true);
        ui->pushButton_make_root->setDisabled(true);
        ui->pushButton_make_map->setDisabled(true);

        execute_app_cmd(proc_amr[0],"roslaunch amr_ros om_manual.launch");

        save_log("Start manual!");
        amr_state_info.amr_state = AMR_STATE::MANUAL;
        amr_state_info.cart_move = true;
    }

}


void Amr_Default_Gui::on_pushButton_make_map_clicked()
{
    if(is_amr_on)
    {
        amr_state_info.amr_state = AMR_STATE::STOPPING;
        kill_all_app_process();

        wait_stop_tmr = 0;
        ui->pushButton_make_map->setText("Stopping");
        ui->pushButton_make_map->setDisabled(true);

        func_name = "Make Map";
        pressed_btn = ui->pushButton_make_map;
        save_log("Stop Make map!");
    }
    else
    {
        QString path = work_path + "/" + amr_config_info.items_val.package_name + "/maps";

        QString fileName = QFileDialog::getOpenFileName(this, tr("Open bag data"), path, tr("Bag Files (*.bag)"));

        if(fileName.isEmpty())
        {
            //ros bag data not exist
            return;
        }

        is_amr_on = true;
        ui->textBrowser->setText("");
        ui->pushButton_make_map->setText("Stop Make");

        ui->pushButton_chk_port->setDisabled(true);
        ui->pushButton_start_amr->setDisabled(true);
        ui->pushButton_collect_data->setDisabled(true);
        ui->pushButton_make_root->setDisabled(true);
        ui->pushButton_run_manual->setDisabled(true);

        execute_app_cmd(proc_amr[0],"roslaunch amr_ros om_hw_simulation.launch");

        execute_app_cmd(proc_amr[1],"rosbag play " + fileName);

        save_log("Start Make map!");
        amr_state_info.amr_state = AMR_STATE::MAKE_MAP;
        amr_state_info.cart_move = false;

    }
}


void Amr_Default_Gui::on_pushButton_reset_alarm_clicked()
{
    //reset om_cart alarm
    if(alarm_info.devices.om_cart.alm_on)
    {        
        //Restart om driver first
        start_om_cart();

        alarm_info.devices.om_cart.alm_on = false;
        alarm_info.devices.om_cart.code = "-";
        ui->label_cart_status->setText("OK");
        ui->label_cart_status->setStyleSheet("color:black");

        //Send command to Reset om driver alarm
        om_cart::om_cart_cmd msg_cart;
        msg_cart.type = OM_CMD_TYPE::OPERATE;
        msg_cart.size = 1;
        msg_cart.data[0] = OPER_CMD::ALM_RST;
        om_cmd_pub.publish(msg_cart);

    }
    //reset lidar alarm
    if(alarm_info.devices.lidar.alm_on)
    {
        alarm_info.devices.lidar.alm_on = false;
        alarm_info.devices.lidar.code = "-";
        ui->label_lidar_status->setText("OK");
        ui->label_lidar_status->setStyleSheet("color:black");
    }
    //reset imu alarm
    if(alarm_info.devices.imu.alm_on)
    {
        alarm_info.devices.imu.alm_on = false;
        alarm_info.devices.imu.code = "-";
        ui->label_imu_status->setText("OK");
        ui->label_imu_status->setStyleSheet("color:black");
    }
    //reset usb-io board alarm
    if(alarm_info.devices.usb_io.alm_on)
    {
        start_usbio_board();
        alarm_info.devices.usb_io.alm_on = false;
        alarm_info.devices.usb_io.code = "-";
        ui->label_usbio_status->setText("OK");
        ui->label_usbio_status->setStyleSheet("color:black");
    }

    //reset joystick alarm
    if(alarm_info.devices.joy.alm_on)
    {
        alarm_info.devices.joy.alm_on = false;
        alarm_info.devices.joy.code = "-";
        ui->label_joystick_status->setText("OK");
        ui->label_joystick_status->setStyleSheet("color:black");
    }

    //reset lamp
    amr_state_info.alarm_on = false;
    ui->textBrowser->clear();
    ui->label_sys_msg->setText("Note: ");
    ui->label_error_code_num->setText("-");
    ui->label_error_code_num->setStyleSheet("color:black");
}


void Amr_Default_Gui::on_pushButton_set_s_on_clicked()
{
    is_set_s_on.data = (is_set_s_on.data == true) ? false : true ;

    if(is_set_s_on.data)
    {
       ui->pushButton_set_s_on->setText("Set S_OFF");
    }
    else
    {
        ui->pushButton_set_s_on->setText("Set S_ON");
    }
    pub_cart_s_on.publish(is_set_s_on);

}


