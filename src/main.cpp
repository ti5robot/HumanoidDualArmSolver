#include <stdio.h>
#include <cstring>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <map>
#include <termios.h>
#include <fcntl.h>
#include <algorithm>
#include <csignal>
#include <mutex>
#include <shared_mutex>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "Ti5CAN_Driver.h"
#include "Ti5MOVE.h"
#include "Ti5BASIC.h"
#include "Ti5LOGIC.h"
#include "mathfunc.h"
#include "tool.h"

#define RESET "\033[0m"
#define RED "\033[31m"     /* Red */
#define GREEN "\033[1;32m"    /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */

#define SERVER_PORT 8888    //socket端口号

using namespace std;
struct Coordinate {
    double x;
    double y;
    double z;
    double r;
	double p;
	double ry;
    double rt;
};

void signalHandler(int signum)
{
    char aaa;
    cout <<RED<< "Interrupt signal (" << signum << ") received.\n"<<RESET ;
    brake(LEFT_ARM,0,0);
    brake(RIGHT_ARM,0,0);
    cout <<RED<< "stop!!" << RESET << endl;
    Exit();
    exit(signum);
}

void test1()
{
    float l_goal_j[7] = {-1, -1.3, -0.3, -1.4, 0.3, -0.1, 0.4};

    joint_to_move(LEFT_ARM,l_goal_j,0,1);

}
void test2()
{
    float r_goal_j[7] = {1, 1.3, 0.3, 1.4, -0.3, 0.1, -0.4};

    joint_to_move(RIGHT_ARM,r_goal_j,0,0);

}

int main()
{
    l_controller.max_vel=0.8;
    r_controller.max_vel=0.8;
    l_solver.gap=2.5;
    r_solver.gap=3.14;
    vector<string> productSerialNumbers = query_can();
    if (productSerialNumbers.empty())
    {
        cout << RED << "未找到任何 USB 设备，请插入设备后重试！" << RESET << endl;
        exit(0);
    }
    else
    {
        cout << CYAN << "找到的 CAN 设备序列号：" << RESET<<endl;
        for (const string &serialNumber : productSerialNumbers)
        {
            cout << CYAN << serialNumber << RESET << endl;
        }
    }

    string ip = ip_address();
    cout << MAGENTA << "ip=" << ip << RESET << endl;

    signal(SIGINT, signalHandler);
    int deviceInd_0 = 0,canInd_0 = 0, deviceInd_1 = 1,canInd_1 = 1;

    if(Start())
    {
        cout<<GREEN<<"init can success!"<<RESET<<endl;
    }
    
    mechanical_arm_origin(LEFT_ARM,0,0);
    mechanical_arm_origin(RIGHT_ARM,0,0);
    sleep(1);


    /////////////////////////////////////获取当前角度///////////////////////////////////////////////
    float test_2[7];
    get_current_angle(LEFT_ARM, test_2,0,0);
    cout << "左臂：";
    for (int i = 0; i < 7; i++)
    {
        cout << "angle[" << i << "]=" << fixed << setprecision(3) << test_2[i] << " ";
    }
    cout << endl;

    get_current_angle(RIGHT_ARM, test_2,0,0);
    cout << "右臂：";
    for (int i = 0; i < 7; i++)
    {
        cout << "angle[" << i << "]=" << fixed << setprecision(3) << test_2[i] << " ";
    }
    cout << endl;
    /////////////////////////////////////获取当前角度///////////////////////////////////////////////
    

    Exit();
    return 0;
}