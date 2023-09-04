# RM23AutoAim
## Ubuntu22.04 + ROS2 Humble
ROS2 package
```
|-- infantry_description //步兵urdf package，提供坐标系转换
|   |-- launch
|   |-- meshes
|   `-- urdf
|-- mindvision_camera  //mindvision相机驱动包
|-- rm_auto_aim        //自动瞄准算法
|   |-- armor_detector //检测器
|   |-- armor_tracker  //跟踪器，EKF整车全局观测器
|   |-- auto_aim_bringup //启动文件
|   |-- auto_aim_interfaces //自定义autoaim interface
|   `-- docs
|-- rm_serial_driver //与控制下位机通信package
`-- rm_vision //整个项目的bringup package
    |-- docs
    `-- rm_vision_bringup
```
具体模块内部会有详细README.md
