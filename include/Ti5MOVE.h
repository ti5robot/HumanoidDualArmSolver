#ifndef TI5MOVE_H
#define TI5MOVE_H

#include <unistd.h>
#include <cstdlib>
#include <chrono>
#include <thread>
// #include "can/SingleCaninterface.h"
// #include "can/motortypehelper.h"
#include <vector>
#include "tool.h"
#include "Ti5LOGIC.h"
#include <time.h>
#include <mutex>
#include "Ti5CAN_Driver.h"

#define USLEEPTIME 3000

extern "C"{ //添加：extern C
	class ArmController{
	private:
		float AG = 0.1;   //启停时变速的采样间距（秒）
		// float AG = 0.05;   //启停时变速的采样间距（秒）
		float mvtime = 0;
		float min_time = 0;
		bool jstp = false;
		uint8_t canidList[IDNUM];
		//
		float scale[IDNUM] = {101,101,101,101,101,101,101}; //电机内圈与外圈的速度比
		float kj2n[2]={50/pi,200/pi};
		float kj2p[2]={32768/pi,131072/pi};  
		// int coder[IDNUM]={0,0,0,0,0,0,0};//双边码器是1，单边码器是0
		// int coder[IDNUM]={0,0,0,0,0,1,1};//双边码器是1，单边码器是0
		int coder[IDNUM]={1,1,1,1,1,1,1};//全双编
		float j2n[IDNUM]; //原来的也删掉
		// float j2p[IDNUM];
		//
		float nplL[4][4]; //add 用于linear_move函数机械臂直线运动
		void setn(int npL[IDNUM],int deviceInd,int canInd);
		void ACTmove(float *a,float *b,float T0,int deviceInd,int canInd);// 实际运动
	public:
		ArmController(uint8_t canid[IDNUM]);
		void plan_move(int deviceInd, int canInd,float tgtj[IDNUM]);
		// void new_plan_move(int deviceInd,int canInd,float crtj[IDNUM]);//mfs add 2024-12-3
		void GETP_plan_move(float tgtj[IDNUM],float *CUrrentJointPosition,int deviceInd,int canInd);
		float max_vel=1.5;//外壳旋转最大旋转速度 1.5 弧度/s  删掉NMAX
		float j2p[IDNUM];
	};
}//添加：extern C

#endif

		