# DannyControl

## 项目结构

### configs

主要修改的文件

#### camera

相机的配置文件，现在只支持realsense摄像机

1. camera_info

   摄像机配置信息

2. oculus_cam

   需要在vr眼镜中显示地摄像机id

3. num_cams

   相机总数

#### network

1. host_address：主机ip
2. xarm_ip：机器臂的ip

### robot

机器人操控的配置文件夹

### src

项目的代码主体

## Run

1. 启动摄像头

   ```
   python robot_camera.py
   ```

   

2. 启动遥操进程

   ```
   python teleop.py
   ```

   

3. （可选）启动记录进程

   ```
   python record.py
   ```

   

