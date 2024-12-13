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
		float AG = 0.005;   //启停时变速的采样间距（秒
		float scale = 101; //电机内圈与外圈的速度比
		float n2p = 655.36; //内圈转速到步速的转化系数
		float mvtime = 0;
		// float j2p = scale * 65536 / 2 / pi;   //电机外圈角度到内圈步数的转化
		float min_time = 0;
		bool jstp = false;
		uint8_t canidList[IDNUM];
		float nplL[4][4]; //add 用于linear_move函数机械臂直线运动
		void setn(int npL[IDNUM],int deviceInd,int canInd);
		void ACTmove(float *a,float *b,float T0,int deviceInd,int canInd);// 实际运动
	public:
		ArmController(uint8_t canid[IDNUM]);
        // void plan_move(float crtj[IDNUM],int canInd,int deviceInd);
		void plan_move(int deviceInd, int canInd,float crtj[IDNUM]);
		void new_plan_move(int deviceInd,int canInd,float crtj[IDNUM]);//mfs add 2024-12-3
		void GETP_plan_move(float crtj[IDNUM],float *CUrrentJointPosition,int deviceInd,int canInd);
		float NMAX = 3000;  //所有电机内核最大转速值（(NMAX/100)圈/秒）
		float j2p = scale * 65536 / 2 / pi;   //电机外圈角度到内圈步数的转化
	};
}//添加：extern C

#endif