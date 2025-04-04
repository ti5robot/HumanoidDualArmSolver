#ifndef Ti5BASIC_H
#define Ti5BASIC_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <sstream>
#include <map>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <vector>
#include <regex>
#include <iomanip>
#include <cstring>

#include "Ti5LOGIC.h"
#include "Ti5MOVE.h"

// #include "can/tcontrolcanfactor.h"

#define MAX_IP_ADDR_LEN 256 // 存储IP地址的最大长度


// extern float arr_s[6];

extern bool flag;

extern string log_path; // log文件

extern char LogInfo[100]; // 存储写入log文件的信息

extern char Info_Str[100]; // 定义一个字符数组用于存储 flag 的字符串表示
class ArmController;       // 声明ArmController类

extern uint8_t l_id[IDNUM], r_id[IDNUM]; // 声明左臂和右臂的canID数组
extern ArmController l_controller;       // 声明TH_L对象
extern ArmController r_controller;       // 声明TH_R对象

extern class humanoidLeftArm l_solver;  // 数学上的解算器
extern class humanoidRightArm r_solver; // 解算器


enum ArmSide
{
  LEFT_ARM, // 左臂
  RIGHT_ARM // 右臂
};

extern "C"
{ // 添加extern "C"

  // 写入调试信息到文件
  void writeDebugInfoToFile(const char *func_name, const char *info);

  // 输出数组的调试信息
  void printArrayDebugInfo(float arr[], int size, const char *arr_name);

  /* 获取本机IP地址*/
  std::string ip_address();

  /*std::string query_can();
  查询can设备号
  */
  std::vector<std::string> query_can();

  void receive_fully(int sock, void *buffer, size_t size);
  /*socket通信
  参数：
    deviceInd：can设备号
    canInd：can通道
    port：端口号
  发送端发送数据格式：{标识位，数据}
        标识位	
        0x01	左臂pos 
        0x02	右臂pos 
        0x03	左臂joint 
        0x04	右臂joint 
    */
  int ti5_socket_server(int deviceInd, int canInd,int port);

  /*获取电机错误状态
  参数：
    side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
    deviceInd：can设备号
    canInd：can通道
    dataList:接收数据的数组，错误状态
  返回值：为电机错误
    0：无错误
    1：软件错误
    2：过压
    4：欠压
    16：启动错误
*/
  // int get_mechanicalarm_Maschinen_status(ArmSide side, int deviceInd, int canInd);
  void get_mechanicalarm_status(ArmSide side, int deviceInd, int canInd, int32_t *dataList);

  /*清除电机错误
    参数：
      side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
      deviceInd：can设备号
      canInd：can通道
  */
  void clear_elc_error(ArmSide side, int deviceInd, int canInd);

  /*机械臂刹车
  参数：
    side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
    canInd：can通道
    deviceInd：can设备号
  返回值：
    true：成功
    false：失败
  */
  bool brake(ArmSide side, int deviceInd, int canInd);

  /*将数据记录下来写入文件
参数：
    pj_flag：角度或者位姿标识,1为角度 ，0为坐标
    filename：存储文件名
    array[6]：被保存的值
*/
  void write_value(int pj_flag, string filename, float array[7]);

  /*机械臂回到原点
  参数：
    side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
    canInd：can通道
    deviceInd：can设备号
  返回值：无
  */
  void mechanical_arm_origin(ArmSide side, int deviceInd, int canInd);

  /*机械臂关节运动
  参数：
    side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
    goal_j：目标关节角
    canInd：can通道
    DeviceInd：can设备号
  返回值：无*/
  void joint_to_move(ArmSide side, float *goal_j, int deviceInd, int canInd);
  void new_joint_to_move(ArmSide side, float *goal_j, int deviceInd, int canInd);

  /*机械臂关节运动，同时获取当前位置
    参数：
        side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
        goal_j：目标关节角
        CUrrentJointPosition：存储当前位置
        canInd：can通道
        deviceInd：can设备号
    返回值：无
  */
  void GetP_joint_to_move(ArmSide side, float *goal_j, float *CUrrentJointPosition, int deviceInd, int canInd);

  /*pos运动
  参数：
      side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
      pos：目标位置
      value：dim的值
      dim：-1~2 代表x,y,z， -1的时候是没有臂角约束，只会接收末端位姿，其他参数忽略
      absolute：true的时候是绝对位置（以胸部原点位置），false的时候是相对位置（以当前点胳膊轴位置）
      canInd：can通道
      deviceInd：can设备号
  返回值：无
  */
  // void pos_to_move(ArmSide side, float *pos, float value, int dim, bool absolute, int deviceInd, int canInd);
  bool pos_to_move(ArmSide side, float *pos, int deviceInd, int canInd);

  /*获取当前角度
  参数：
      side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
      goal_j：存储角度的数组
      canInd：can通道
      deviceInd：can设备号
    */
  void get_current_angle(ArmSide side, float goal_j[7], int deviceInd, int canInd);

  /*获取当前位姿
     参数：
         side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
         posz：存储位姿的数组
         canInd：can通道
         deviceInd：can设备号
     */
  void get_current_pose(ArmSide side, float posz[7], int deviceInd, int canInd);

  /*设置为电流模式，并设置目标电流
    参数：
         side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
         current: 目标电流
         canInd：can通道
         deviceInd：can设备号
  */
  void set_current_mode(ArmSide side, uint32_t current[7], int deviceInd, int canInd);

  /*设置键盘控制模式
    参数：
      canInd：can通道
      deviceInd：can设备号
  */
  void LLL_keyboard_controller_J(int deviceInd, int canInd);
  void LLL_keyboard_controller_P(int deviceInd, int canInd);
  void RRR_keyboard_controller_J(int deviceInd, int canInd);
  void RRR_keyboard_controller_P(int deviceInd, int canInd);
  // void L_keyboard_controller(int deviceInd, int canInd);
  // void R_keyboard_controller(int deviceInd, int canInd);

  /*获取电机当前位置
    参数：
      canInd：can通道
      deviceInd：can设备号
      MotorsTotal：电机数量
      MotorsIDlist：电机ID列表
      data：存储电机位置数据的数组
  */
  void get_motor_position(int deviceInd, int canInd, int MotorsTotal,uint8_t *MotorsIDlist,int32_t *MotorPosition);

  /*设置电机当前位置
    参数：
      canInd：can通道
      deviceInd：can设备号
      MotorsTotal：电机数量
      MotorsIDlist：电机ID列表
      data：存储电机位置数据的数组
  */
  void set_motor_position(int deviceInd, int canInd, int MotorsTotal,uint8_t *MotorsIDlist,int32_t *MotorPosition);


  // 设定终端为非阻塞模式
  void setNonBlocking(bool enable) ;

  // 设定文件描述符为非阻塞
  void setNonBlockingInput() ;
  // 清空键盘缓冲区
  void clearStdinBuffer();

  // 读取键盘输入
  char ssscanKeyboard();

} // 添加extern "C"
#endif