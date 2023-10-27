#!/bin/bash

#setup usb-rs485 adapter modbus cable
sudo -S su << EOF
mikuni
EOF
sudo su -c 'modprobe ftdi_sio vendor=0x06ce product=0x8331'
sudo su -c 'echo 06ce 8331 > /sys/bus/usb-serial/drivers/ftdi_sio/new_id'

sudo chmod 777 /dev/ttyUSB*
sudo chmod 777 /dev/ttyACM*

#start start/stop gui
#update ros env
source /opt/ros/noetic/setup.bash
#Workbanch package
source ~/catkin_ws/devel/setup.bash

killall om_cart
killall om_modbusRTU_no
killall roslaunch

roslaunch amr_ros amr_default_start.launch
