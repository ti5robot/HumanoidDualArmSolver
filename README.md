#Ti5robot

欢迎您使用Ti5robot机械臂，并感谢您的购买。

本文档记载了有关机械臂的安装、调试、以及如何基于API进行二次开发的相关信息。

机械臂使用人员应充分了解风险，使用前必须认真阅读本手册，严格遵守手册中的规范和要求。

## 简介

机械臂具有开放性的程序接口和拓展接口，机械臂末端可快速换装不同执行器；能够用于电商物流、新消费、日常生活等多种场景。

## 注意事项

1.请务必按照本手册中的要求安装机械臂、连接线缆

2.确保机械臂的活动范围内不会碰撞到人或其他物品，以免发生意外

3.在使用前，需要专业的人员进行调试

4.在使用SDK时，必须确保输入的参数和操作流程是正确的

5.请注意机械臂运行速度，过快时务必小心

6.机械臂使用结束后，请务必断电

7.机械臂断电后，请务必将控制程序关闭

8.避免在潮湿或粉尘的环境下使用机械臂

9.请务必将机械臂存放、安装到儿童碰不到的地方，以免发生危险


# SDK介绍

机械臂控制的代码code中，分别是`include`，`src`，`log`以及`usrlib`。

+ [include] 存储着机械臂所需的头文件。
+ [src] 一般控制机械臂的文件放在此处，其中`main.cpp`是一个示例程序。
+ [log]sdk中存放log的文件夹。
+ [usrlib]包含SDK所需的so文件

## 1. include

除以下提到的文件外，用户无需查看该文件夹下的其他文件。

### 1.1 Ti5LOGIC.h

该文件是机械臂的算法库，包括正逆解，碰撞检测，规划路径等。
使用方法：根据需求在规划机械臂运动的时候可以调用该库的函数。

### 1.2 Ti5MOVE.h

机械臂运动控制库。

### 1.3 mathfunc.h
机械臂的数学模型函数

### 1.4 Ti5CAN_Driver.h
CAN通信相关
+ bool Start();
```
  函数功能：登录并初始化can设备
  返回值：true 或者 false
  参数：无
  示例：
      if(Start())
      {
	  cout<<"init can success!"<<endl;
      }
	  mechanical_arm_origin(LEFT_ARM,0,0);
	  mechanical_arm_origin(RIGHT_ARM,1,0);
      Exit();
```

+ bool Exit();
```
  函数功能：断开can设备
  参数：无
  返回值：true 或者 false
  示例：
      if(Start())
      {
	  cout<<"init can success!"<<endl;
      }
	  mechanical_arm_origin(LEFT_ARM,0,0);
	  mechanical_arm_origin(RIGHT_ARM,1,0);
      Exit();
```

### 1.5 Ti5BASIC.h

机械臂控制基础库，包含了基本控制以及信息，用户在使用时需要根据自身使用方式自行选择调用。
```
机械臂参数
enum ArmSide {
    LEFT_ARM, //左臂
    RIGHT_ARM //右臂
};
```

+ void writeDebugInfoToFile(const char *func_name, const char *info);
  ```
  函数功能：将信息写入log中
  返回值：无
  参数：
      *func_name：函数名字
      *info：要写入log的信息内容
  示例：
      void func_eg(){
      	for (int i = 0; i < 6; i++)
      	{
            cout << "电机" << i << "最大负向位置: " << static_cast<int32_t>(reg_min_app_position[i]) << endl;
            sprintf(LogInfo, "电机%d最大正向位置: %d", i, static_cast<int32_t>(reg_min_app_position[i]));
            writeDebugInfoToFile(__func__, LogInfo);
      	}
      	cout << endl;
       }
  ```

+ std::vector<std::string> query_can();
  ```
  函数功能：查询can设备号
  返回值：无
  参数：无
  示例：
    vector<string> productSerialNumbers = query_can();
    if (productSerialNumbers.empty()) {
    cout << RED<<"未找到任何 USB 设备，请插入设备后重试！" <<RESET << endl;
    exit(0);
    } else {
  	cout <<CYAN<< "找到的 CAN 设备序列号：" <<RESET;
  	for (const string& serialNumber : productSerialNumbers) {
  		cout << CYAN <<serialNumber <<RESET << endl;
  	}
    }
  ```

+ int get_elektrische_Maschinen_status(ArmSide side,int deviceInd,int canInd);
  ```
  函数功能：获取电机错误状态
  返回值：为电机错误
      0：无错误
      1：软件错误
      2：过压
      4：欠压
      16：启动错误
  参数：
      side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
      deviceInd：can设备号
      canInd：can通道
  示例：
      int main()
      {
  	Start();
  	get_elektrische_Maschinen_status(LEFT_ARM,0,0);
  	get_elektrische_Maschinen_status(RIGHT_ARM,0,0);
  	Exit();
  	return 0;
       }
  ```

+ void clear_elc_error(ArmSide side,int deviceInd,int canInd);
  ```
  函数功能：清除电机错误
  返回值：无
  参数：
    side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
    deviceInd：can设备号
    canInd：can通道
  示例：
      int main()
      {
  	Start();
  	clear_elc_error(LEFT_ARM,0,0);
  	clear_elc_error(RIGHT_ARM,1,0);
  	Exit();
  	return 0;
      }
  ```

+ bool brake(ArmSide side,int deviceInd,int canInd);
   ```
  函数功能：机械臂刹车
  返回值：成功返回true
  参数：
    side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
    deviceInd：can设备号
    canInd：can通道
  示例：
     void signalHandler(int signum)
     {
   	char aaa;
   	cout << "Interrupt signal (" << signum << ") received.\n";
   	brake(LEFT_ARM0,0,0);
   	brake(RIGHT_ARM,1,0);
   	cout << "stop!!" << endl;
   	inspect_brake();
   	Exit();
   	exit(signum);
     }
  ```

+ void mechanical_arm_origin(ArmSide side,int deviceInd,int canInd);
  ```
  函数功能：机械臂初始化位置
  返回值：无
  参数：
      side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
      canInd：can通道
      deviceInd：can设备号
  示例：
    int main()
    {
  	Start();
  	mechanical_arm_origin(LEFT_ARM,0,0);
  	mechanical_arm_origin(RIGHT_ARM,0,0);
  	Exit();
  	return 0;
    }
  ```
  
+ void joint_to_move(ArmSide side,float *goal_j,int deviceInd,int canInd);
  ```
    函数功能：机械臂关节运动
    返回值：无
    参数：
  	side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
  	goal_j：目标关节角
  	canInd：can通道
  	deviceInd：can设备号
  示例：
     int main()
     {
  	Start();
  	float l_goal_j[7] = {-1, -1.3, -0.3, -1.4, 0.3, -0.1, 0.4};
  	float r_goal_j[7] = {1, 1.3, 0.3, 1.4, -0.3, 0.1, -0.4};
  	joint_to_move(LEFT_ARM,l_goal_j,0,0);
  	joint_to_move(RIGHT_ARM,r_goal_j,0,0);
  	Exit();
  	return 0;
     }
  ```
  
+ void GetP_joint_to_move(ArmSide side,float *goal_j,float *CUrrentJointPosition,int deviceInd,int canInd);
  ```
     函数功能：机械臂关节运动，同时获取当前位置
     返回值：无
	 参数：
  	  side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
  	  goal_j：目标关节角
  	  CUrrentJointPosition：存储当前位置
  	  canInd：can通道
  	  deviceInd：can设备号
  示例：
	 int main()
	 {
		Start();
		float l_goal_j[7] = {-1, -1.3, -0.3, -1.4, 0.3, -0.1, 0.4};
		float r_goal_j[7] = {1, 1.3, 0.3, 1.4, -0.3, 0.1, -0.4};
		float CUrrentJointPosition[7];
		GetP_joint_to_move(LEFT_ARM, l_goal_j, CUrrentJointPosition,0,0);
		for (int i = 0; i < 7; i++)
		{
			cout << "postion[" << i << "]=" << fixed << setprecision(3) << CUrrentJointPosition[i] << endl;
		}
		sleep(1);
		GetP_joint_to_move(RIGHT_ARM, r_goal_j, CUrrentJointPosition,0,0);
		for (int i = 0; i < 7; i++)
		{
			cout << "postion[" << i << "]=" << fixed << setprecision(3) << CUrrentJointPosition[i] << endl;
		}
		
		Exit();
		return 0;
	 }
  ```
  
+ void pos_to_move(ArmSide side,float *pos,float value,int dim,bool absolute,int deviceInd,int canInd);
  ```
    函数功能：机械臂位姿运动
    返回值：无
    参数：
  	side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
  	pos：目标位置
  	value：dim的值
  	dim：0~2 代表x,y,z，-1的时候是没有臂角约束，只会接收末端位姿，其他参数忽略
  	absolute：true的时候是绝对位置（以胸部原点位置），false的时候是相对位置（以当前点胳膊轴位置）
  	canInd：can通道
  	deviceInd：can设备号
  示例：
     int main()
     {
  	Start();
  	float l_pos[6] = {347.931, 319.393, 56.2954, -1.21035, -2.73428, -0.883535}, r_pos[6] = {347.931, -319.393, 56.2954, -1.93124, -0.407309, -2.25806};
  	pos_to_move(LEFT_ARM, l_pos, 202.702, 0, true,0,0);
  	pos_to_move(RIGHT_ARM, r_pos, 202.702, 0, true,0,0);
  	Exit();
  	return 0;
     }
  ```

     
+ void get_current_angle(ArmSide side,float goal_j[7],int deviceInd,int canInd);
  ```
  函数功能：获取当前角度
  返回值：无
  参数：
	side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
	goal_j：存储角度的数组
  	canInd：can通道
  	deviceInd：can设备号
  示例：
     int main()
     {
		Start();
		float savejoint[7];
        	get_current_angle(LEFT_ARM, savejoint,0,0);
		cout << "左臂：";
		for (int i = 0; i < 7; i++)
		{
			cout << "angle[" << i << "]=" << fixed << setprecision(3) << savejoint[i] << " ";
		}
		cout << endl;

		get_current_angle(RIGHT_ARM, savejoint,0,0);
		cout << "右臂：";
		for (int i = 0; i < 7; i++)
		{
			cout << "angle[" << i << "]=" << fixed << setprecision(3) << savejoint[i] << " ";
		}
		cout << endl;
		Exit();
		return 0;
     }
  ```

+ void get_current_pose(ArmSide side,float posz[7],int deviceInd,int canInd);
  ```
    函数功能：获取当前位姿
    返回值：无
    参数：
  	side：左臂或右臂 （LEFT_ARM 左臂，RIGHT_ARM 右臂）
  	posz：存储位姿的数组
  	canInd：can通道
  	deviceInd：can设备号
  示例：
     int main()
     {
  	Start();
  	float savepos[6]={0};
  	get_current_pose(LEFT_ARM, savepos,0,0);
  	for (int i = 0; i < 6; i++)
  	{
  		printf("左臂pos[%d]=%.2f\n", i, savepos[i]);
  	}
  	get_current_pose(RIGHT_ARM, savepos,0,0);
  	for (int i = 0; i < 6; i++)
  	{
  		printf("右臂pos[%d]=%.2f\n", i, savepos[i]);
  	}
  	Exit();
  	return 0;
     }
  ```




### 1.5 tool.h

该文件是一些`tool`，具体函数使用及参数请查看该文件。

### 1.6. can
该文件夹包含`can`通讯的头文件，机械臂是通过can通讯与控制机联通的，具体函数功能及参数请查看里面所包含的文件中注释了解函数作用。

## 3.src
### 3.1 main.cpp

该文件是一个简单的示例程序，调用了`Ti5BASIC.h`中的`query_can()`函数，首先查找是否连接了can设备，然后调用`mechanical_arm_origin()`让机械臂回到初始位置,然后有一个小动作。

### 3.2 gcc.sh

该文件中的内容是编译命令，编译的时候可以使用该命令直接编译，也可以使用g++命令+对应参数直接编译

### 3.2 编译

首先需要安装依赖：
```
sudo apt update
sudo apt install -y libspdlog-dev libopencv-dev libudev-dev libfmt-dev
```
然后将usrlib中的libcontrolcan.so  libmylibscan.so libmylibti5.so文件拷贝到/usr/lib/下
```
cd ~/your_name_folder/usrlib
sudo cp * /usr/lib
如果是2004的系统的话：
cd ~/your_name_folder/usrlib/2004
sudo cp * /usr/lib
```

最后执行`gcc.sh`文件进行编译或通过以下命令进行编译生成可执行文件`move_sov`。(注意：以下路径是默认路径，如果修改了路径要替换成自己的)
```
cd ~/your_name_folder/src
export CPLUS_INCLUDE_PATH=/your_name_folder/include:$CPLUS_INCLUDE_PATH
g++ main.cpp  -L./include -lmylibti5_2004 -L./include/can -lmylibscan -lcontrolcan  -lfmt -ludev -o move_sov(在ubuntu2004下)
g++ main.cpp  -L./include -lmylibti5 -L./include/can -lmylibscan -lcontrolcan  -lfmt -ludev -o move_sov(在ubuntu2204下)
```
**运行**:
```
sudo ./move_sov
```
注意机械臂处在一个安全的环境中

## 开发须知
由于电机内部的算法，指令发送的值与实际执行的值有微小误差。

