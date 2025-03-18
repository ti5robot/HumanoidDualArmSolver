#ifndef TI5CAN_DRIVER_H
#define TI5CAN_DRIVER_H

#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>
#include <map>
#include <algorithm>
#include <unordered_map>


#include "controlcan.h"
#include "Ti5BASIC.h"

#define GET_MOTOR_PERSIONS  8 //获取电机当前位置指令（1字节），转化为减速机角度公式：(返回值/65536/减速比)*360
#define GET_MOTOR_ERROR     10  //获取电机错误指令（1字节）
#define CLEAR_ERROR         11 //清除电机错误指令（1字节）
#define SET_MOTOR_CURRENTS  28 //设置电机为电流模式，并设置目标电流指令（5字节）
#define SET_MOTOR_SPEED     29  //设置电机为速度模式，并设置目标速度指令（5字节），下发参数为：(目标转速（度每秒）*减速比*100)/360
#define SET_MOTOR_POSITION  30 //设置电机为位置模式，并设置目标位置指令（5字节），下发参数为：(减速机目标角度/360)*减速比*65536
#define SET_MOTOR_MAX_SPEED 36 //设置电机最大正向允许速度指令（5字节），下发参数为：(目标转速（度每秒）*减速比*100)/360
#define SET_MOTOR_MIN_SPEED 37  //设置电机最小负向允许速度指令（5字节），下发参数为：(目标转速（度每秒）*减速比*100)/360


extern int CanNum;

// 查询 CAN 设备并绑定序列号与设备索引
std::map<std::string, int> query_can_with_index();
 /*登录并初始化can设备
    参数：无
    返回值
      true：成功
      false：失败
  */
bool Start();
// void Start();

bool compare_serial_numbers(const std::string &a, const std::string &b);
std::unordered_map<std::string, int> get_device_map();
void operate_multiple_can_devices();

/*登出can设备
    参数：无
    返回值
      true：成功
      false：失败
  */
bool Exit();

int32_t convertHexArrayToDecimal(const uint8_t hexArray[4]);

void toIntArray(int number, int *res, int size);

/*发送1字节指令
  参数：
    DeviceInd：can设备索引 （一个为0,2个为1）
    CANInd：can通道索引 （通道1：0,通道2：1）
    numOfActuator：发送指令的电机数量
    canIdList：电机canId列表
    commandList：指令列表
    dataList：接收数据列表
*/
void sendSimpleCanCommand(int DeviceInd, int CANInd,uint8_t numOfActuator, uint8_t *canIdList, uint8_t command,int32_t *dataList);

void sendCanCommand(int DeviceInd,int CANInd,uint8_t numOfActuator, uint8_t *canIdList, uint8_t command, uint32_t *parameterList);
#endif // TEST_CAN_H



// // 比较函数，用于按字典序排序设备序列号
// bool compare_serial_numbers(const std::string &a, const std::string &b)
// {
//     return a < b;  // 字典序比较
// }

// std::unordered_map<std::string, int> get_device_map()
// {
//     VCI_BOARD_INFO pinfo[50];
//     std::unordered_map<std::string, int> deviceMap;
//     int num = VCI_FindUsbDevice2(pinfo); // 查询所有CAN设备

//     // 提取所有设备的序列号
//     std::vector<std::string> serialNumbers;
//     for (int i = 0; i < num; i++)
//     {
//         std::string serialNumber = "";
//         for (int j = 0; j < 20; j++) // 提取序列号
//         {
//             if (pinfo[i].str_Serial_Num[j] == '\0') // 结尾判断
//                 break;
//             serialNumber += pinfo[i].str_Serial_Num[j];
//         }
//         serialNumbers.push_back(serialNumber);
//     }

//     // 按照字典序对序列号进行排序
//     std::sort(serialNumbers.begin(), serialNumbers.end(), compare_serial_numbers);

//     // 将排序后的序列号和索引进行绑定
//     for (int i = 0; i < serialNumbers.size(); i++)
//     {
//         deviceMap[serialNumbers[i]] = i;
//     }

//     return deviceMap; // 返回序列号与设备索引的映射关系
// }

// void operate_multiple_can_devices()
// {
//     auto deviceMap = get_device_map(); // 获取设备序列号与设备索引的映射

//     // 遍历所有设备
//     for (const auto &pair : deviceMap)
//     {
//         const std::string &serialNumber = pair.first;  // 设备序列号
//         int nDeviceInd = pair.second;                  // 设备索引
//         int nDeviceType = 4;                           // 根据实际设备类型设置
//         DWORD dwRel;
//         VCI_INIT_CONFIG vic;

//         std::cout << "正在操作设备: " << serialNumber << " (索引: " << nDeviceInd << ")" << std::endl;

//         // 打开设备
//         dwRel = VCI_OpenDevice(nDeviceType, nDeviceInd, 0);
//         if (dwRel != 1)
//         {
//             std::cout << "无法打开设备 " << serialNumber << std::endl;
//             continue;
//         }

//         // 初始化 CAN 通道
//         vic.AccCode = 0x80000008;
//         vic.AccMask = 0xFFFFFFFF;
//         vic.Filter = 1;
//         vic.Timing0 = 0x00;
//         vic.Timing1 = 0x14;
//         vic.Mode = 0;

//         if (VCI_InitCAN(nDeviceType, nDeviceInd, 0, &vic) != 1)
//         {
//             std::cout << "初始化设备 " << serialNumber << " 的 CAN 通道 0 失败" << std::endl;
//             VCI_CloseDevice(nDeviceType, nDeviceInd);
//             continue;
//         }

//         if (VCI_InitCAN(nDeviceType, nDeviceInd, 1, &vic) != 1)
//         {
//             std::cout << "初始化设备 " << serialNumber << " 的 CAN 通道 1 失败" << std::endl;
//             VCI_CloseDevice(nDeviceType, nDeviceInd);
//             continue;
//         }

//         // 启动 CAN 通道
//         if (VCI_StartCAN(nDeviceType, nDeviceInd, 0) != 1)
//         {
//             std::cout << "启动设备 " << serialNumber << " 的 CAN 通道 0 失败" << std::endl;
//             VCI_CloseDevice(nDeviceType, nDeviceInd);
//             continue;
//         }

//         if (VCI_StartCAN(nDeviceType, nDeviceInd, 1) != 1)
//         {
//             std::cout << "启动设备 " << serialNumber << " 的 CAN 通道 1 失败" << std::endl;
//             VCI_CloseDevice(nDeviceType, nDeviceInd);
//             continue;
//         }

//         std::cout << "设备 " << serialNumber << " 的所有 CAN 通道启动成功!" << std::endl;
//     }
// }
