import ti5_robot_py as ti5
import sys
import time

def main():
    # 获取本机IP
    ip = ti5.ip_address()
    print("IP Address:", ip)
    # help(ti5.joint_to_move)
    # 查询CAN设备列表

    cans = ti5.query_can()
    if not cans:
        print("\033[31m未找到任何 USB 设备，请插入设备后重试！\033[0m")
        sys.exit(0)
    else:
        print("\033[36m找到的 CAN 设备序列号：\033[0m")
        for serialNumber in cans:
            print(f"\033[36m{serialNumber}\033[0m")

    if ti5.Start():
        print("Started successfully!")
    else:
        print("Start failed.")
        sys.exit(1)   # 退出程序，状态码1表示异常退出

    # 机械臂回到零位
    ti5.mechanical_arm_origin(ti5.ArmSide.LEFT_ARM, 0, 0)
    ti5.mechanical_arm_origin(ti5.ArmSide.RIGHT_ARM, 0, 0)

    # 机械臂关节运动
    l_goal_j = [-1, -1.3, -0.3, -1.4, 0.3, -0.1, 0.4]
    r_goal_j = [1, 1.3, 0.3, 1.4, -0.3, 0.1, -0.4]
    time.sleep(1)
    ti5.joint_to_move(ti5.ArmSide.LEFT_ARM, l_goal_j, 0, 0)
    ti5.joint_to_move(ti5.ArmSide.RIGHT_ARM, r_goal_j, 0, 0)

    #机械臂获取当前角度
    get_currentangle_l = ti5.get_current_angle(ti5.ArmSide.LEFT_ARM, 0, 0)
    get_currentangle_r = ti5.get_current_angle(ti5.ArmSide.RIGHT_ARM, 0, 0)
    print("左臂当前角度：", end="")
    for i in range(7):
        print(f"{get_currentangle_l[i]:.2f}", end="  ")
    print()
    print("右臂当前角度：", end="")
    for i in range(7):
        print(f"{get_currentangle_r[i]:.2f}", end="  ")
    print()
    
    time.sleep(1)

    #机械臂pos运动
    l_pos=[361.27,325,57.8,-1.25,-2.7,-0.87]
    r_pos=[361.27,-325,57.8,1.21,-2.74,0.88]
    ti5.pos_to_move(ti5.ArmSide.LEFT_ARM, l_pos, 0, 0)
    ti5.pos_to_move(ti5.ArmSide.RIGHT_ARM, r_pos, 0, 0)

    #获取机械臂当前坐标
    get_currentpos_l = ti5.get_current_pose(ti5.ArmSide.LEFT_ARM, 0, 0)
    get_currentpos_r = ti5.get_current_pose(ti5.ArmSide.RIGHT_ARM, 0, 0)
    print("左臂坐标：", end="")
    for i in range(6):
        print(f"{i+1}:{get_currentpos_l[i]:.2f}", end="  ")
    print()
    print("右臂坐标：", end="")
    for i in range(6):
        print(f"{i+1}:{get_currentpos_r[i]:.2f}", end="  ")
    print()

    # # 调用机械臂状态查询
    # status = ti5.get_mechanicalarm_status(ti5.ArmSide.LEFT_ARM, 0, 0)
    # print("机械臂状态:", status)

    # # 设置0电流模式示例
    # current_values = [0, 0, 0, 0, 0, 0, 0]
    # ti5.set_current_mode(ti5.ArmSide.LEFT_ARM, current_values, 0, 0)
    # ti5.set_current_mode(ti5.ArmSide.RIGHT_ARM, current_values, 0, 0)

    #左臂键盘运动
    ti5.LLL_keyboard_controller_J(0,0)
    # # 机械臂刹车
    # success = ti5.brake(ti5.ArmSide.LEFT_ARM, 0, 0)
    # print("刹车结果:", success)


    if ti5.Exit():
        print("Exited successfully!")
    else:
        print("Exit failed.")

if __name__ == "__main__":
    main()
