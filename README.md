# 🦾 基于双目相机的二维码识别与 Panda 机械臂抓取系统

本项目实现了一个完整的基于 **ROS** 与 **MoveIt!** 的视觉抓取系统。  
系统使用 **双目相机** 获取贴有二维码的三个物块的空间位置信息，将坐标发布为 ROS 话题，最后由 **Panda 机械臂** 自动移动至目标位置并完成抓取任务。

---

## 📚 目录

- [📦 包简介](#-包简介)
- [🧠 功能概述](#-功能概述)
- [🗂️ 目录结构](#️-目录结构)
- [🚀 运行说明](#-运行说明)

---

## 📦 包简介

`my_arm_controller` 是一个基于 ROS 的自定义功能包，包含了二维码识别、深度计算、坐标发布与机械臂控制等核心模块。

---

## 🧠 功能概述

- 📷 **二维码识别**：使用 `pyzbar` 库识别相机画面中的二维码，并解析二维码 ID。  
- 🧭 **深度信息计算**：基于双目相机的视差原理，计算二维码中心点的三维坐标。  
- 📡 **坐标发布**：以自定义消息 `QRCode.msg` 格式发布识别到的物块坐标。  
- 🤖 **机械臂控制**：通过 MoveIt! 控制 Panda 机械臂移动至目标坐标并完成抓取。  
- 🧱 **仿真模型支持**：内置了用于 Gazebo 仿真的桌面、相机与三个带二维码的物块模型。

---

## 🗂️ 目录结构

```bash
my_arm_controller/
├── CMakeLists.txt
├── package.xml
├── launch/                     # 启动文件
│   ├── arm_control.launch       # 启动机械臂控制节点
│   ├── empytworld_withbox.launch# 启动 Gazebo 仿真场景（桌面与物块）
│   ├── qrcode.launch            # 启动二维码识别与坐标发布节点
│   └── spawn.launch             # 启动模型生成节点
├── models/                     # 仿真模型文件夹
│   ├── arm_small_box_1/         # 带二维码的物块模型 1
│   ├── arm_small_box_2/         # 带二维码的物块模型 2
│   ├── arm_small_box_3/         # 带二维码的物块模型 3
│   ├── stereo_camera/           # 双目相机模型
│   └── table/                   # 桌面模型
├── msg/
│   └── QRCode.msg               # 自定义消息类型 (ID + 坐标)
├── scripts/                     # Python 脚本
│   ├── arm_control.py           # 控制 Panda 机械臂抓取
│   ├── create_qrcode.py         # 生成二维码图片工具
│   ├── get_deep_info.py         # 双目深度计算
│   └── recognize_qrcode.py      # 识别二维码并发布坐标
└── src/                         # (预留) C++ 源文件目录
```

---

## 🚀 运行说明

1.项目编译  
```bash
cd ~/arm_ws
catkin_make
```

2.添加环境变量  
```bash
cd ~/.bashrc
```
在最后一行添加
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/arm_ws/src/my_arm_controller/models
```
保存后
```bash
source ~/.bashrc
```

3.修改python解释器路径   
将/home/lwz/arm_ws/src/my_arm_controller/launch下的所有launch文件中的  
```bash
<env name="PYTHONPATH" value="/home/lwz/arm_ws/venv/lib/python3.8/site-packages:$(env PYTHONPATH)" />
```
value值修改为自己的python解释器路径   

4.运行  
启动gazebo仿真环境  
```bash
roslaunch panda_moveit_config demo_gazebo.launch 
```
启动机械臂抓取进程  
```bash
roslaunch my_arm_controller arm_control.launch 
```


