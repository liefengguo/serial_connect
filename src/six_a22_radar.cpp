#include <ros/ros.h>
#include <serial/serial.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <std_msgs/Int32.h>
#include "../include/serial_connect/add_CRC.h"
#include <serial_connect/a22_data.h>
// 串口参数
#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE 115200

// 控制指令
#define CMD_LENGTH 8
uint8_t CMD_01[CMD_LENGTH] = {0x01, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85,0xF6};
uint8_t CMD_02[CMD_LENGTH] = {0x02, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85,0xF6};
uint8_t CMD_03[CMD_LENGTH] = {0x03, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85,0xF6};
uint8_t CMD_04[CMD_LENGTH] = {0x04, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85,0xF6};
uint8_t CMD_05[CMD_LENGTH] = {0x05, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85,0xF6};
uint8_t CMD_06[CMD_LENGTH] = {0x06, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85,0xF6};

// 接收数据帧结构
#define FRAME_LENGTH 7
#define HEADER 0x01

typedef struct {
    uint8_t header;  // 帧头
    uint8_t address;  // 地址
    uint8_t command;  // 指令
    uint8_t data[2];  // 数据
    uint16_t  checksum; // 校验位
} Frame;


// 发送指令线程函数
void sendThreadFunc(serial::Serial &ser, std::mutex &mutex, std::condition_variable &cv, bool &isRunning) {
    while (isRunning) {
        // 发送控制指令
        {
            std::lock_guard<std::mutex> lock(mutex);
            ser.write(CMD_01, CMD_LENGTH);
            ser.write(CMD_02, CMD_LENGTH);
            ser.write(CMD_03, CMD_LENGTH);
            ser.write(CMD_04, CMD_LENGTH);
            ser.write(CMD_05, CMD_LENGTH);
            ser.write(CMD_06, CMD_LENGTH);

        }
        cv.notify_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// 接收指令线程函数
void receiveThreadFunc(serial::Serial &ser, std::mutex &mutex, std::condition_variable &cv, bool &isRunning, ros::Publisher &pub) {
    while (isRunning) {
        // 等待发送指令
        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock);
        }
        uint8_t buffer[FRAME_LENGTH];

        // 读取返回数据
        int count = ser.read(buffer, FRAME_LENGTH);
         if (count != FRAME_LENGTH) {
             ROS_ERROR("Failed to read response.");
             continue; // 读取失败，跳过此次循环
         }
//        for (int i = 0; i < FRAME_LENGTH; ++i) {
//            cout<< " buffer :"<<hex <<static_cast<int>(buffer[i])<<std::endl;
//        }

        // 解析数据
        Frame *frame = (Frame *)buffer;
        if (frame->address != 0x03) {
             int count = ser.read(buffer, FRAME_LENGTH-1);
             ROS_ERROR("Invalid response header.");
             continue; // 解析失败，跳过此次循环
        }
        uint16_t data1 ,data2,data3,data4,data5,data6;
        switch (frame->header)
        {
        case HEADER:
            /* code */
            data1 = (frame->data[0] << 8) | frame->data[1];
            break;
        case 0x02:
            /* code */
            data2 = (frame->data[0] << 8) | frame->data[1];
            break;
        case 0x03:
            /* code */
            data3 = (frame->data[0] << 8) | frame->data[1];
            break;
        case 0x04 :
            /* code */
            data4 = (frame->data[0] << 8) | frame->data[1];
            break;
        case 0x05:
            /* code */
            data5 = (frame->data[0] << 8) | frame->data[1];
            break;
        case 0x06:
            /* code */
            data6 = (frame->data[0] << 8) | frame->data[1];
            break;
        default:
            break;
        }
        
        // uint16_t crc = usMBCRC16(buffer,FRAME_LENGTH - 2);
        // if(frame->checksum != crc){
        //     int count = ser.read(buffer, FRAME_LENGTH-3);
        //     ROS_ERROR("Invalid CRC.");
        //     continue; // 解析失败，跳过此次循环
        //  }

        // 发布ROS话题
        serial_connect::a22_data a22_data;
        std::vector<int32_t> datas;
        datas.push_back(data1);
        datas.push_back(data2);
        datas.push_back(data3);
        datas.push_back(data4);
        datas.push_back(data5);
        datas.push_back(data6);

//        uint16_t data2 = (frame->data[2] << 8) | frame->data[3];
//        uint16_t data3 = (frame->data[4] << 8) | frame->data[5];
//        uint16_t data4 = (frame->data[6] << 8) | frame->data[7];

        std::cout<<"1号："<<data1<<"2号："<<data2<<"3号："<<data3<<"4号："<<data4<<"5号："<<data5<<"6号："<<data6<<std::endl;
        // std::cout<<"1号："<<data1<< " 03:"<< static_cast<int>(frame->data[0]) << " 04:"<<static_cast<int>(frame->data[1])<<std::endl;
        a22_data.a22_datas = datas;
        pub.publish(a22_data);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "a22_radar_node");
    ros::NodeHandle nh;
    uint8_t pucCRCHi , pucCRCLo; 

    // 创建ROS话题
    ros::Publisher pub = nh.advertise<serial_connect::a22_data>("a22_radar", 1000);
   uint16_t result =  usMBCRC16(CMD_01,CMD_LENGTH - 2,pucCRCHi ,pucCRCLo);
    CMD_01[CMD_LENGTH - 1] = pucCRCHi;
    CMD_01[CMD_LENGTH - 2] = pucCRCLo;
    cout<<hex<<static_cast<int>(pucCRCHi)<<static_cast<int>(pucCRCLo)<<"result:"<<result<<endl;
    // 初始化串口
    serial::Serial ser(SERIAL_PORT, BAUDRATE, serial::Timeout::simpleTimeout(100));


    // 启动发送和接收线程
    std::mutex mutex;
    std::condition_variable cv;
    bool isRunning = true;
    std::thread sendThread(sendThreadFunc, std::ref(ser), std::ref(mutex), std::ref(cv), std::ref(isRunning));
    std::thread receiveThread(receiveThreadFunc, std::ref(ser), std::ref(mutex), std::ref(cv), std::ref(isRunning), std::ref(pub));

    // 等待程序结束
    ros::spin();

    // 停止发送和接收线程
    isRunning = false;
    cv.notify_all();
    sendThread.join();
    receiveThread.join();

    return 0;
}