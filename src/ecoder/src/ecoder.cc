#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Geometry> 
#include <chrono>
#include <locale>
#include <tuple>
#include <algorithm>
#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <boost/assert.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include "std_msgs/Float64.h"

extern "C" {
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <time.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <unistd.h> //  close
#include <string.h> //  strerror
}


using namespace std;

static int data_length = 81;


boost::asio::serial_port* serial_port = 0;
static const uint8_t stop[6] = {0xA5, 0x5A, 0x04, 0x02, 0x06, 0xAA};
static const uint8_t mode[6] = {0xA5, 0x5A, 0x04, 0x01, 0x05, 0xAA};
static uint8_t data_raw[200];
static std::vector<uint8_t> buffer_;
static std::deque<uint8_t> queue_;
static std::string name, frame_id;
static sensor_msgs::Imu msg;
static sensor_msgs::MagneticField msg_mag;
static sensor_msgs::NavSatFix msg_gps;
static int fd_ = -1;
static ros::Publisher pub;
static uint8_t tmp[81];

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ecoder");
    ros::NodeHandle n("~");

    name = ros::this_node::getName();

    std::string port;
    if (n.hasParam("port"))
    n.getParam("port", port);
    else
    {
        ROS_ERROR("%s: must provide a port", name.c_str());
        return -1;
    }

    int baud;
    if (n.hasParam("baud"))
    n.getParam("baud", baud);
    else
    {
        ROS_ERROR("%s: must provide a baudrate", name.c_str());
        return -1;
    }

    ROS_WARN("Baudrate set to %d", baud);

    double delay;
    n.param("delay", delay, 0.0);

    boost::asio::io_service io_service;
    serial_port = new boost::asio::serial_port(io_service);
    try
    {
        serial_port->open(port);
    }
    catch (boost::system::system_error &error)
    {
        ROS_ERROR("%s: Failed to open port %s with error %s",
                name.c_str(), port.c_str(), error.what());
        return -1;
    }

    if (!serial_port->is_open())
    {
        ROS_ERROR("%s: failed to open serial port %s",
                name.c_str(), port.c_str());
        return -1;
    }

    typedef boost::asio::serial_port_base sb;

    sb::baud_rate baud_option(baud);
     sb::flow_control flow_control(sb::flow_control::none);
    sb::parity parity(sb::parity::none);
    sb::stop_bits stop_bits(sb::stop_bits::one);

    serial_port->set_option(baud_option);
    serial_port->set_option(flow_control);
    serial_port->set_option(parity);
    serial_port->set_option(stop_bits);

    const char *path = port.c_str();
    fd_ = open(path, O_RDWR);
    if(fd_ < 0)
    {
        ROS_ERROR("Port Error!: %s", path);
        return -1;
    }

    int kk = 0;
    double vyaw_sum = 0;
    double vyaw_bias = 0;
    pub = n.advertise<std_msgs::Float64>("/currant_vel", 50);

    ROS_WARN("Streaming Ecoder Data...");

    ros::Rate r(100);

    while (n.ok())
    {
        read(fd_, tmp, sizeof(uint8_t) * data_length);
        memcpy(data_raw, tmp, sizeof(uint8_t) * data_length);
        // ROS_INFO("working \n");
        bool found = false;

            if(data_raw[0]== 0x00)//帧头
            {
                std_msgs::Float64 currant_vel;
                /*--------------校验------------*/
                uint8_t DATA1,DATA2,DATA3,JY;
                uint32_t DATA;
                DATA1=data_raw[1];
                DATA2=data_raw[2];
                DATA3=data_raw[3];
                DATA=DATA1*255+DATA2;
                JY=DATA%7+DATA/7;
                ROS_INFO("DATA1:%d| DATA2:%d |DATA3: %d",DATA1,DATA2,DATA);
                if(JY == DATA3)
                {
                /*--------------读数------------*/
                currant_vel.data=(std::float_t)DATA;
                currant_vel.data=currant_vel.data/(51*255);
                ROS_INFO("currant_vel:%f\n",currant_vel.data);
                }
                /*--------------发布------------*/
                pub.publish(currant_vel);
                
                found = true;
            }
            ros::spinOnce();                   // Handle ROS events
            r.sleep();
    }

    // Stop continous and close device
    ROS_WARN("Wait 0.1s");
    ros::Duration(0.1).sleep();
    ::close(fd_);

    return 0;
}