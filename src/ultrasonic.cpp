//serial_demo.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <sstream>

// 封装的函数，接收一个字符串类型的报文，返回4个探头的距离值
std::vector<int> get_distances(std::string serial_data) {
    // 将数据按照字节解析出来
    std::string frame_head = serial_data.substr(0, 2);
    std::string address = serial_data.substr(2, 2);
    std::string command = serial_data.substr(4, 2);
    int data_1h = std::stoi(serial_data.substr(6, 2), nullptr, 16);
    int data_1l = std::stoi(serial_data.substr(8, 2), nullptr, 16);
    int data_2h = std::stoi(serial_data.substr(10, 2), nullptr, 16);
    int data_2l = std::stoi(serial_data.substr(12, 2), nullptr, 16);
    int data_3h = std::stoi(serial_data.substr(14, 2), nullptr, 16);
    int data_3l = std::stoi(serial_data.substr(16, 2), nullptr, 16);
    int data_4h = std::stoi(serial_data.substr(18, 2), nullptr, 16);
    int data_4l = std::stoi(serial_data.substr(20, 2), nullptr, 16);
    std::string checksum = serial_data.substr(22, 2);

    // 计算4个探头的距离值
    int distance_1 = (data_1h << 8) + data_1l;
    int distance_2 = (data_2h << 8) + data_2l;
    int distance_3 = (data_3h << 8) + data_3l;
    int distance_4 = (data_4h << 8) + data_4l;

    // 将距离值存储在向量中
    std::vector<int> distances = {distance_1, distance_2, distance_3, distance_4};

    // 返回距离值
    return distances;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;
    
    //创建一个serial对象
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(9600);
    //串口设置timeout
    sp.setTimeout(to);

    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }
    
    ros::Rate loop_rate(500);
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t n = sp.available();
        if(n!=0)
        {
            uint8_t buffer[1024];
            //读出数据
            n = sp.read(buffer, n);

            // if(n > 0)

            for(int i=0; i<n; i++)
            {
                //16进制的方式打印到屏幕
                std::cout << std::hex << (buffer[i] & 0xff) << " ";
            }
            std::cout << std::endl;
            //把数据发送回去
            sp.write(buffer, n);
        }
        loop_rate.sleep();
    }
    
    //关闭串口
    sp.close();
 
    return 0;
}
