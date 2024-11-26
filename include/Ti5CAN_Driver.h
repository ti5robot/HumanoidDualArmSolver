#ifndef TI5CAN_DRIVER_H
#define TI5CAN_DRIVER_H

#include <iostream>
#include <string>
#include <unistd.h>
// #include "can/controlcan.h"
#include "controlcan.h"

 /*登录并初始化can设备
    参数：无
    返回值
      true：成功
      false：失败
  */
bool Start();

/*登出can设备
    参数：无
    返回值
      true：成功
      false：失败
  */
bool Exit();

int32_t convertHexArrayToDecimal(const uint8_t hexArray[4]);

void toIntArray(int number, int *res, int size);

void sendSimpleCanCommand(int DeviceInd, int CANInd,uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList,int32_t *data_test);

void sendCanCommand(int DeviceInd,int CANInd,uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, uint32_t *parameterList);
#endif // TEST_CAN_H
