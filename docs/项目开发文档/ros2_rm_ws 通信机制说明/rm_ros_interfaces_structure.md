# rm_ros_interfaces 包目录结构大纲图

## 根目录
```
rm_ros_interfaces/
├── include/
│   └── rm_ros_interfaces/
│       └── rm_ros_interfaces/
│           ├── msg/
│           │   ├── detail/
│           │   │   ├── *_struct.h
│           │   │   ├── *_struct.hpp
│           │   │   ├── *_traits.hpp
│           │   │   └── *_rosidl_typesupport_fastrtps_cpp.hpp
│           │   ├── *.h
│           │   └── *.hpp
│           └── srv/
│               ├── detail/
│               │   ├── *_struct.h
│               │   ├── *_struct.hpp
│               │   ├── *_traits.hpp
│               │   └── *_rosidl_typesupport_fastrtps_cpp.hpp
│               ├── activate_gripper.h/.hpp
│               ├── close_modbus.h/.hpp
│               ├── deactivate_gripper.h/.hpp
│               ├── initialize_gripper.h/.hpp
│               ├── get_dh.h/.hpp
│               └── modbus_control.h/.hpp
├── local/
│   └── lib/
│       └── python3.10/
│           └── dist-packages/
│               └── rm_ros_interfaces/
│                   ├── msg/
│                   ├── srv/
│                   └── *.py
└── share/
    └── rm_ros_interfaces/
        ├── cmake/
        │   ├── rm_ros_interfacesConfig.cmake
        │   ├── rm_ros_interfacesConfig-version.cmake
        │   └── 各种导出文件
        ├── environment/
        ├── hook/
        ├── local_setup.sh
        ├── package.sh
        └── package.xml
```

## 详细说明

### 1. include 目录
包含为C++生成的头文件：
- **msg目录**：包含消息类型的头文件
- **srv目录**：包含服务类型的头文件
- **detail子目录**：包含实现细节的头文件

### 2. local/lib/python3.10/dist-packages 目录
包含为Python生成的模块文件：
- Python绑定代码
- 消息和服务的Python类实现

### 3. share 目录
包含包的元数据和配置信息：
- **cmake目录**：CMake配置文件，用于其他包依赖此包时的配置
- **environment目录**：环境配置脚本
- **hook目录**：构建钩子脚本
- package.xml：包的元数据描述文件