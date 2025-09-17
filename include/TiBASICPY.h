#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <algorithm>
#include <cstdint>
#include <vector>
#include <string>
#include "Ti5BASIC.h"
#include "Ti5CAN_Driver.h"

namespace py = pybind11;

// 辅助模板：Python list 转 std::vector<T>
template<typename T>
std::vector<T> list_to_vector(const py::list& lst) {
    std::vector<T> vec;
    vec.reserve(lst.size());
    for (auto item : lst) {
        vec.push_back(item.cast<T>());
    }
    return vec;
}

// 常量定义，根据你的API，调整长度
constexpr int DATA_LIST_LEN = 20;
constexpr int ARM_ARR_LEN = 7;

PYBIND11_MODULE(ti5_bindings_new, m) {
    m.doc() = R"pbdoc(
        Python bindings for Ti5 BASIC API
        -------------------------------
        提供机械臂控制的 Python 接口
    )pbdoc";

    // 枚举绑定
    py::enum_<ArmSide>(m, "ArmSide")
        .value("LEFT_ARM", LEFT_ARM)
        .value("RIGHT_ARM", RIGHT_ARM)
        .export_values();

    m.def("Start", &Start,
          R"pbdoc(
          初始化并启动机械臂系统。

          返回:
              bool: 启动是否成功。

          示例:
              success = ti5.Start()
              if not success:
                  print("启动失败")
          )pbdoc");

    m.def("Exit", &Exit,
          R"pbdoc(
          关闭并退出机械臂系统。

          返回:
              bool: 退出是否成功。

          示例:
              success = ti5.Exit()
              if not success:
                  print("退出失败")
          )pbdoc");

    m.def("writeDebugInfoToFile", &writeDebugInfoToFile,
          py::arg("func_name"), py::arg("info"),
          R"pbdoc(
          将调试信息写入日志文件。

          参数:
              func_name (str): 调用函数名。
              info (str): 要写入的调试信息。

          示例:
              ti5.writeDebugInfoToFile("joint_to_move", "开始移动")
          )pbdoc");

    m.def("printArrayDebugInfo", [](py::list arr, int size, const std::string& arr_name) {
        auto vec = list_to_vector<float>(arr);
        std::vector<float> arr_c(size, 0.f);
        int n = std::min(static_cast<int>(vec.size()), size);
        std::copy_n(vec.begin(), n, arr_c.begin());
        printArrayDebugInfo(arr_c.data(), size, arr_name.c_str());
    }, py::arg("arr"), py::arg("size"), py::arg("arr_name"),
    R"pbdoc(
    打印数组调试信息。

    参数:
        arr (List[float]): 要打印的浮点数组。
        size (int): 数组长度。
        arr_name (str): 数组名称。

    示例:
        ti5.printArrayDebugInfo([1.0, 2.0, 3.0], 3, "goal_joints")
    )pbdoc");

    m.def("ip_address", &ip_address,
          R"pbdoc(
          获取本机IP地址。

          返回:
              str: IP地址字符串。

          示例:
              ip = ti5.ip_address()
              print(ip)
          )pbdoc");

    m.def("query_can", &query_can,
          R"pbdoc(
          查询已连接的 CAN 设备序列号列表。

          返回:
              List[str]: CAN设备序列号列表。

          示例:
              devices = ti5.query_can()
              for dev in devices:
                  print(dev)
          )pbdoc");

    m.def("ti5_socket_server", &ti5_socket_server,
          py::arg("deviceInd"), py::arg("canInd"), py::arg("port"),
          R"pbdoc(
          启动 CAN Socket 服务器。

          参数:
              deviceInd (int): 设备号。
              canInd (int): CAN通道号。
              port (int): 端口号。

          返回:
              int: 服务器状态码。

          示例:
              status = ti5.ti5_socket_server(1, 0, 8080)
          )pbdoc");

    m.def("get_mechanicalarm_status", [](ArmSide side, int deviceInd, int canInd) {
        int32_t dataList[ARM_ARR_LEN] = {0};
        get_mechanicalarm_status(side, deviceInd, canInd, dataList);
        return std::vector<int32_t>(dataList, dataList + ARM_ARR_LEN);
    }, py::arg("side"), py::arg("deviceInd"), py::arg("canInd"),
    R"pbdoc(
    获取机械臂电机错误状态。

    参数:
        side (ArmSide): 左臂或右臂。
        deviceInd (int): 设备号。
        canInd (int): CAN通道号。

    返回:
        List[int]: 长度7的错误状态列表。

    示例:
        errors = ti5.get_mechanicalarm_status(ti5.ArmSide.LEFT_ARM, 1, 0)
        print(errors)
    )pbdoc");

    m.def("clear_elc_error", &clear_elc_error,
          py::arg("side"), py::arg("deviceInd"), py::arg("canInd"),
          R"pbdoc(
          清除机械臂电机错误。

          参数:
              side (ArmSide): 左臂或右臂。
              deviceInd (int): 设备号。
              canInd (int): CAN通道号。

          示例:
              ti5.clear_elc_error(ti5.ArmSide.LEFT_ARM, 1, 0)
          )pbdoc");

    m.def("brake", &brake,
          py::arg("side"), py::arg("deviceInd"), py::arg("canInd"),
          R"pbdoc(
          机械臂刹车。

          参数:
              side (ArmSide): 左臂或右臂。
              deviceInd (int): 设备号。
              canInd (int): CAN通道号。

          返回:
              bool: 是否刹车成功。

          示例:
              success = ti5.brake(ti5.ArmSide.RIGHT_ARM, 1, 0)
          )pbdoc");

    m.def("write_value", [](int pj_flag, const std::string& filename, py::list array) {
        auto vec = list_to_vector<float>(array);
        float arr_c[ARM_ARR_LEN] = {0};
        for (int i = 0; i < ARM_ARR_LEN && i < (int)vec.size(); i++) {
            arr_c[i] = vec[i];
        }
        write_value(pj_flag, filename, arr_c);
    }, py::arg("pj_flag"), py::arg("filename"), py::arg("array"),
    R"pbdoc(
    将角度或位姿数据写入文件。

    参数:
        pj_flag (int): 标志，1为角度，0为坐标。
        filename (str): 文件名。
        array (List[float]): 长度7的数值列表。

    示例:
        ti5.write_value(1, "data.txt", [0, 10, 20, 30, 40, 50, 60])
    )pbdoc");

    m.def("mechanical_arm_origin", &mechanical_arm_origin,
          py::arg("side"), py::arg("deviceInd"), py::arg("canInd"),
          R"pbdoc(
          机械臂回到原点。

          参数:
              side (ArmSide): 左臂或右臂。
              deviceInd (int): 设备号。
              canInd (int): CAN通道号。

          示例:
              ti5.mechanical_arm_origin(ti5.ArmSide.LEFT_ARM, 1, 0)
          )pbdoc");

    m.def("joint_to_move", [](ArmSide side, py::list goal_j, int deviceInd, int canInd) {
        auto vec = list_to_vector<float>(goal_j);
        float arr[ARM_ARR_LEN] = {0};
        for (int i = 0; i < ARM_ARR_LEN && i < (int)vec.size(); i++) arr[i] = vec[i];
        joint_to_move(side, arr, deviceInd, canInd);
    }, py::arg("side"), py::arg("goal_j"), py::arg("deviceInd"), py::arg("canInd"),
    R"pbdoc(
    让机械臂关节运动。

    参数:
        side (ArmSide): 左臂或右臂。
        goal_j (List[float]): 目标关节角列表，长度7。
        deviceInd (int): 设备号。
        canInd (int): CAN通道号。

    示例:
        ti5.joint_to_move(ti5.ArmSide.LEFT_ARM, [0, 30, 45, 0, 15, 0, 0], 1, 0)
    )pbdoc");

    m.def("pos_to_move", [](ArmSide side, py::list pos, int deviceInd, int canInd) {
        auto vec = list_to_vector<float>(pos);
        float arr[ARM_ARR_LEN] = {0};
        for (int i = 0; i < ARM_ARR_LEN && i < (int)vec.size(); i++) arr[i] = vec[i];
        return pos_to_move(side, arr, deviceInd, canInd);
    }, py::arg("side"), py::arg("pos"), py::arg("deviceInd"), py::arg("canInd"),
    R"pbdoc(
    让机械臂位姿运动。

    参数:
        side (ArmSide): 左臂或右臂。
        pos (List[float]): 目标位置列表，长度7。
        deviceInd (int): 设备号。
        canInd (int): CAN通道号。

    返回:
        bool: 运动是否成功。

    示例:
        success = ti5.pos_to_move(ti5.ArmSide.RIGHT_ARM, [0.1,0.2,0.3,0,0,0,0], 1, 0)
    )pbdoc");

    m.def("get_current_angle", [](ArmSide side, int deviceInd, int canInd) {
        float angles[ARM_ARR_LEN] = {0};
        get_current_angle(side, angles, deviceInd, canInd);
        return std::vector<float>(angles, angles + ARM_ARR_LEN);
    }, py::arg("side"), py::arg("deviceInd"), py::arg("canInd"),
    R"pbdoc(
    获取当前机械臂关节角度。

    参数:
        side (ArmSide): 左臂或右臂。
        deviceInd (int): 设备号。
        canInd (int): CAN通道号。

    返回:
        List[float]: 当前关节角度列表，长度7。

    示例:
        angles = ti5.get_current_angle(ti5.ArmSide.LEFT_ARM, 1, 0)
        print(angles)
    )pbdoc");

    m.def("get_current_pose", [](ArmSide side, int deviceInd, int canInd) {
        float pose[ARM_ARR_LEN] = {0};
        get_current_pose(side, pose, deviceInd, canInd);
        return std::vector<float>(pose, pose + ARM_ARR_LEN);
    }, py::arg("side"), py::arg("deviceInd"), py::arg("canInd"),
    R"pbdoc(
    获取当前机械臂位姿。

    参数:
        side (ArmSide): 左臂或右臂。
        deviceInd (int): 设备号。
        canInd (int): CAN通道号。

    返回:
        List[float]: 当前位姿列表，长度7。

    示例:
        pose = ti5.get_current_pose(ti5.ArmSide.RIGHT_ARM, 1, 0)
        print(pose)
    )pbdoc");

    m.def("set_current_mode", [](ArmSide side, py::list current, int deviceInd, int canInd) {
        auto vec = list_to_vector<uint32_t>(current);
        uint32_t arr[ARM_ARR_LEN] = {0};
        for (int i = 0; i < ARM_ARR_LEN && i < (int)vec.size(); i++) arr[i] = vec[i];
        set_current_mode(side, arr, deviceInd, canInd);
    }, py::arg("side"), py::arg("current"), py::arg("deviceInd"), py::arg("canInd"),
    R"pbdoc(
    设置机械臂电流模式，并设置目标电流。

    参数:
        side (ArmSide): 左臂或右臂。
        current (List[int]): 目标电流列表，长度7。
        deviceInd (int): 设备号。
        canInd (int): CAN通道号。

    示例:
        ti5.set_current_mode(ti5.ArmSide.LEFT_ARM, [100,100,100,100,100,100,100], 1, 0)
    )pbdoc");

    m.def("LLL_keyboard_controller_J", &LLL_keyboard_controller_J,
          py::arg("deviceInd"), py::arg("canInd"),
          R"pbdoc(
          左臂键盘关节控制模式启动。

          参数:
              deviceInd (int): 设备号。
              canInd (int): CAN通道号。
          )pbdoc");

    m.def("LLL_keyboard_controller_P", &LLL_keyboard_controller_P,
          py::arg("deviceInd"), py::arg("canInd"),
          R"pbdoc(
          左臂键盘位置控制模式启动。

          参数:
              deviceInd (int): 设备号。
              canInd (int): CAN通道号。
          )pbdoc");

    m.def("RRR_keyboard_controller_J", &RRR_keyboard_controller_J,
          py::arg("deviceInd"), py::arg("canInd"),
          R"pbdoc(
          右臂键盘关节控制模式启动。

          参数:
              deviceInd (int): 设备号。
              canInd (int): CAN通道号。
          )pbdoc");

    m.def("RRR_keyboard_controller_P", &RRR_keyboard_controller_P,
          py::arg("deviceInd"), py::arg("canInd"),
          R"pbdoc(
          右臂键盘位置控制模式启动。

          参数:
              deviceInd (int): 设备号。
              canInd (int): CAN通道号。
          )pbdoc");

    m.def("setNonBlocking", &setNonBlocking, py::arg("enable"),
          R"pbdoc(
          设置终端非阻塞模式。

          参数:
              enable (bool): 是否启用非阻塞模式。

          示例:
              ti5.setNonBlocking(True)
          )pbdoc");

    m.def("setNonBlockingInput", &setNonBlockingInput,
          R"pbdoc(
          设置文件描述符为非阻塞模式。

          无参数。
          )pbdoc");

    m.def("clearStdinBuffer", &clearStdinBuffer,
          R"pbdoc(
          清空键盘缓冲区。

          无参数。
          )pbdoc");

    m.def("ssscanKeyboard", &ssscanKeyboard,
          R"pbdoc(
          读取键盘输入。

          返回:
              char: 输入的字符。

          示例:
              ch = ti5.ssscanKeyboard()
              print(ch)
          )pbdoc");
}
