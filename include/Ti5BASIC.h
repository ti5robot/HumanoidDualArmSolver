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

#include "Ti5LOGIC.h"
#include "Ti5MOVE.h"

// #include "can/tcontrolcanfactor.h"

#define MAX_IP_ADDR_LEN 256 // 存储IP地址的最大长度

// extern float arr_s[6];

extern bool flag;

extern string log_path; // log文件

extern char LogInfo[100]; // 存储写入log文件的信息

extern char Info_Str[100]; // 定义一个字符数组用于存储 flag 的字符串表示
class ArmController;  // 声明ArmController类

extern uint8_t l_id[IDNUM],r_id[IDNUM]; // 声明左臂和右臂的canID数组
extern ArmController l_controller;  // 声明TH_L对象
extern ArmController r_controller;  // 声明TH_R对象

extern class humanoidLeftArm l_solver;//数学上的解算器
extern class humanoidRightArm r_solver; //解算器

enum ArmSide {
    LEFT_ARM, //左臂
    RIGHT_ARM //右臂
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

    /*获取电机错误状态
    参数：
      side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
      deviceInd：can设备号
      canInd：can通道
    返回值：为电机错误
      0：无错误
      1：软件错误
      2：过压
      4：欠压
      16：启动错误
  */
  int get_elektrische_Maschinen_status(ArmSide side,int deviceInd,int canInd);

  /*清除电机错误
    参数：
      side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
      deviceInd：can设备号
      canInd：can通道
  */
  void clear_elc_error(ArmSide side,int deviceInd,int canInd);

  /*机械臂刹车
  参数：
    side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
    canInd：can通道
    deviceInd：can设备号
  返回值：
    true：成功
    false：失败
  */
    bool brake(ArmSide side,int deviceInd,int canInd);

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
    void mechanical_arm_origin(ArmSide side,int deviceInd,int canInd);

  /*机械臂关节运动
  参数：
    side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
    goal_j：目标关节角
    canInd：can通道
    DeviceInd：can设备号
  返回值：无*/
	void joint_to_move(ArmSide side,float *goal_j,int deviceInd,int canInd);

  /*机械臂关节运动，同时获取当前位置
    参数：
        side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
        goal_j：目标关节角
        CUrrentJointPosition：存储当前位置
        canInd：can通道
        deviceInd：can设备号
    返回值：无
  */
  void GetP_joint_to_move(ArmSide side,float *goal_j,float *CUrrentJointPosition,int deviceInd,int canInd);

  /*pos运动
  参数：
      side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
      pos：目标位置
      value：dim的值
      dim：0~2 代表x,y,z
      absolute：true的时候是绝对位置（以胸部原点位置），false的时候是相对位置（以当前点胳膊轴位置）
      canInd：can通道
      deviceInd：can设备号
  返回值：无
  */
  void pos_to_move(ArmSide side,float *pos,float value,int dim,bool absolute,int deviceInd,int canInd);

  /*获取当前角度
  参数：
      side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
      goal_j：存储角度的数组
      canInd：can通道
      deviceInd：can设备号
    */
    void get_current_angle(ArmSide side,float goal_j[7],int deviceInd,int canInd);
  
 /*获取当前位姿
    参数：
        side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
        posz：存储位姿的数组
        canInd：can通道
        deviceInd：can设备号
    */
    void get_current_pose(ArmSide side,float posz[7],int deviceInd,int canInd);

} // 添加extern "C"
#endif
