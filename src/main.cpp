#include <stdio.h>
#include <cstring>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include "Ti5MOVE.h"
#include "Ti5BASIC.h"
#include "Ti5LOGIC.h"
#include "mathfunc.h"
#include "tool.h"
#include <csignal>
#include <mutex>
#include <shared_mutex>
#include "Ti5CAN_Driver.h"


#define RESET "\033[0m"
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */

using namespace std;


void signalHandler(int signum)
{
    char aaa;
    cout <<RED<< "Interrupt signal (" << signum << ") received.\n"<<RESET ;
    brake(LEFT_ARM,0,0);
    brake(RIGHT_ARM,1,1);
    cout <<RED<< "stop!!" << RESET << endl;
    Exit();
    exit(signum);
}

void test1()
{
    float l_goal_j[7] = {-1, -1.3, -0.3, -1.4, 0.3, -0.1, 0.4};

    joint_to_move(LEFT_ARM,l_goal_j,0,0);

}
void test2()
{
    float r_goal_j[7] = {1, 1.3, 0.3, 1.4, -0.3, 0.1, -0.4};

    joint_to_move(RIGHT_ARM,r_goal_j,1,0);

}


int main()
{
    l_controller.NMAX=1500;
    r_controller.NMAX=1500;
    l_solver.gap=0.5;
    r_solver.gap=0.5;
    vector<string> productSerialNumbers = query_can();
    if (productSerialNumbers.empty())
    {
        cout << RED << "未找到任何 USB 设备，请插入设备后重试！" << RESET << endl;
        exit(0);
    }
    else
    {
        cout << CYAN << "找到的 CAN 设备序列号：" << RESET;
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
        cout<<"init can success!"<<endl;
    }

    // LLL_keyboard_controller_J(0,0);
    LLL_keyboard_controller_P(0,0);
    // RRR_keyboard_controller_J(0,0);
    // RRR_keyboard_controller_P(0,0);
    
    mechanical_arm_origin(LEFT_ARM,0,0);
    mechanical_arm_origin(RIGHT_ARM,1,0);
    /////////////////////////////////////获取当前角度///////////////////////////////////////////////
    float test_2[7];
    get_current_angle(LEFT_ARM, test_2,0,0);
    cout << "左臂：";
    for (int i = 0; i < 7; i++)
    {
        cout << "angle[" << i << "]=" << fixed << setprecision(3) << test_2[i] << " ";
    }
    cout << endl;

    get_current_angle(RIGHT_ARM, test_2,1,0);
    cout << "右臂：";
    for (int i = 0; i < 7; i++)
    {
        cout << "angle[" << i << "]=" << fixed << setprecision(3) << test_2[i] << " ";
    }
    cout << endl;
    /////////////////////////////////////获取当前角度///////////////////////////////////////////////
    
    float l_goal_j[7] = {-1, -1.3, -0.3, -1.4, 0.3, -0.1, 0.4};
    float r_goal_j[7] = {1, 1.3, 0.3, 1.4, -0.3, 0.1, -0.4};

    // joint_to_move(LEFT_ARM,l_goal_j);
    // joint_to_move(RIGHT_ARM,r_goal_j);
    float test[7];
    GetP_joint_to_move(LEFT_ARM, l_goal_j, test,0,0);
    for (int i = 0; i < 7; i++)
    {
        cout << "postion[" << i << "]=" << fixed << setprecision(3) << test[i] << endl;
        // cout << fixed << setprecision(3) << goal_j[i] << " ";
    }
    sleep(1);
    GetP_joint_to_move(RIGHT_ARM, r_goal_j, test,1,0);
    for (int i = 0; i < 7; i++)
    {
        cout << "postion[" << i << "]=" << fixed << setprecision(3) << test[i] << endl;
    }
    float r_goal_j1[7] = {1.2, 1.3, 0.3, 1.4, -0.3, 0.1, -0.4};
    GetP_joint_to_move(RIGHT_ARM, r_goal_j1, test,1,0);
    for (int i = 0; i < 7; i++)
    {
        cout << "postion[" << i << "]=" << fixed << setprecision(3) << test[i] << endl;
    }

    mechanical_arm_origin(LEFT_ARM,0,0);
    mechanical_arm_origin(RIGHT_ARM,1,0);

    std::thread right_arm_thread(test1);
    std::thread left_arm_thread(test2);
    // 等待两个线程完成
    right_arm_thread.join();
    left_arm_thread.join();
    
    // float l_goal_j[7] = {-1, -1.3, -0.3, -1.4, 0.3, -0.1, 0.4};

    // joint_to_move(LEFT_ARM,l_goal_j,0,0);
    //  float r_goal_j[7] = {1, 1.3, 0.3, 1.4, -0.3, 0.1, -0.4};

    // joint_to_move(RIGHT_ARM,r_goal_j,1,0);
    // Exit(0);
    // get_elektrische_Maschinen_status(LEFT_ARM,0,0);
    // clear_elc_error(LEFT_ARM,0,0);
    // clear_elc_error(RIGHT_ARM,1,0);
    sleep(2);
    mechanical_arm_origin(LEFT_ARM,0,0);
    mechanical_arm_origin(RIGHT_ARM,1,0);
    /////////////////////////////////////位姿运动并获取当前位姿///////////////////////////////////////////////
    float l_pos[6] = {347.931, 319.393, 56.2954, -1.21035, -2.73428, -0.883535}, r_pos[6] = {347.931, -319.393, 56.2954, -1.93124, -0.407309, -2.25806};
    // float test[6] = {0};
    pos_to_move(LEFT_ARM, l_pos, 202.702, 0, true,0,0);
    pos_to_move(RIGHT_ARM, r_pos, 202.702, 0, true,1,0);

    get_current_pose(LEFT_ARM, test,0,0);
    for (int i = 0; i < 6; i++)
    {
        printf("左臂pos[%d]=%.2f\n", i, test[i]);
    }

    get_current_pose(RIGHT_ARM, test,1,0);
    for (int i = 0; i < 6; i++)
    {
        printf("右臂pos[%d]=%.2f\n", i, test[i]);
    }
    /////////////////////////////////////位姿运动并获取当前位姿///////////////////////////////////////////////

    /////////////////////////////////////获取当前角度///////////////////////////////////////////////
    get_current_angle(LEFT_ARM, test,0,0);
    cout << "左臂：";
    for (int i = 0; i < 7; i++)
    {
        cout << "angle[" << i << "]=" << fixed << setprecision(3) << test[i] << " ";
    }
    cout << endl;

    get_current_angle(RIGHT_ARM, test,1,0);
    cout << "右臂：";
    for (int i = 0; i < 7; i++)
    {
        cout << "angle[" << i << "]=" << fixed << setprecision(3) << test[i] << " ";
    }
    cout << endl;
    /////////////////////////////////////获取当前角度///////////////////////////////////////////////
    Exit();
    return 0;
}