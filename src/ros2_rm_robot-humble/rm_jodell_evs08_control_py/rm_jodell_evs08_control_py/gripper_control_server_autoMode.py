
import rclpy
from rclpy.node import Node
# 使用rm_ros_interfaces中的服务定义
from rm_ros_interfaces.srv import InitializeGripper, ActivateGripper, DeactivateGripper, CloseModbus

# 导入Realman Robot官方API
try:
    from Robotic_Arm.rm_robot_interface import *
    REALMAN_API_AVAILABLE = True
except ImportError:
    REALMAN_API_AVAILABLE = False
    print("Warning: Realman Robot API not available. Using simulation mode.")

import time

class JodellEvs08ControlServer(Node):

    def __init__(self):
        super().__init__('jodell_evs08_control_server')
        
        # 声明参数并设置默认值
        self.declare_parameter('robot_ip', '192.168.1.18') # Lab433 Realman IP

        self.declare_parameter('robot_port', 8080)
        self.declare_parameter('register_address', 1000)
        self.declare_parameter('comm_port', 1)
        self.declare_parameter('device_address', 9)
        
        # 获取参数值
        self.robot_ip           = self.get_parameter('robot_ip').value
        self.robot_port         = self.get_parameter('robot_port').value
        self.register_address   = self.get_parameter('register_address').value
        self.comm_port          = self.get_parameter('comm_port').value
        self.device_address     = self.get_parameter('device_address').value
        
        # ✨ 创建服务，使用相对名称以便支持命名空间
        self.init_service       = self.create_service(
            InitializeGripper, 'initialize_gripper', self.initialize_gripper_callback)
        self.activate_service   = self.create_service(
            ActivateGripper, 'activate_gripper', self.activate_gripper_callback)
        self.deactivate_service = self.create_service(
            DeactivateGripper, 'deactivate_gripper', self.deactivate_gripper_callback)
        self.close_service      = self.create_service(
            CloseModbus, 'close_modbus', self.close_modbus_callback)
        
        # 保存连接状态
        self.robot_handle   = None
        self.arm            = None
        self.is_initialized = False
        
        self.get_logger().info('Jodell EVS08 Control Server is ready')
        self.get_logger().info(f'''Default parameters: IP={self.robot_ip},
                               Port={self.robot_port}, 
                               Register={self.register_address}, 
                               CommPort={self.comm_port}, 
                               Device={self.device_address}''')
        if not REALMAN_API_AVAILABLE:
            self.get_logger().warn('Realman Robot API not available. Running in simulation mode.')


    def initialize_gripper_callback(self, request, response):
        """初始化吸盘 - 连接机械臂并设置Modbus模式"""
        # 使用请求中的参数值，如果未提供则使用默认值
        robot_ip = request.ip if request.ip else self.robot_ip
        robot_port = request.port if request.port else self.robot_port
        register_address = request.register_address if request.register_address else self.register_address
        comm_port = request.comm_port if request.comm_port else self.comm_port
        device_address = request.device_address if request.device_address else self.device_address
        
        self.get_logger().info(f'Initializing gripper with IP: {robot_ip}, Port: {robot_port}')
        
        try:
            # 保存参数
            self.robot_ip = robot_ip
            self.robot_port = robot_port
            self.register_address = register_address
            self.comm_port = comm_port
            self.device_address = device_address
            
            if REALMAN_API_AVAILABLE:
                # 实例化RoboticArm类
                self.arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
                
                # 创建机械臂连接
                self.get_logger().info(f'Connecting to robot at {self.robot_ip}:{self.robot_port}')
                self.robot_handle = self.arm.rm_create_robot_arm(self.robot_ip, self.robot_port)
                self.get_logger().info(f'Connected to robot with ID: {self.robot_handle.id}')
                
                # 打印arm对象的所有属性和方法（调试用）
                # 🐛 self.get_logger().debug(f'RoboticArm attributes and methods: {dir(self.arm)}')
                
                # ✨ 设置工具电压为24V
                print( self.arm.rm_set_tool_voltage(3) )
                
                # 设置末端为Modbus RTU模式
                self.get_logger().info(f'Setting Modbus RTU mode on port {self.comm_port}')
                result = self.arm.rm_set_modbus_mode(self.comm_port, 115200, 2)  # 使用115200波特率，2秒超时
                if result != 0:
                    raise Exception(f"Failed to set Modbus mode, error code: {result}")
                
                # 使能吸盘
                self.get_logger().info('Enabling gripper')
                # 吸盘使能通常通过Modbus命令完成，在activate阶段执行
            else:
                # 模拟连接机械臂
                self.get_logger().info(f'Connecting to robot at {self.robot_ip}:{self.robot_port}')
                time.sleep(0.5)  # 模拟连接时间
                
                # 模拟设置工具电压
                self.get_logger().info('Setting tool voltage to 24V')
                time.sleep(0.2)
                
                # 模拟设置末端为Modbus RTU模式
                self.get_logger().info(f'Setting Modbus RTU mode on port {self.comm_port}')
                time.sleep(0.2)
                
                # 模拟使能吸盘
                self.get_logger().info('Enabling gripper')
                time.sleep(0.2)
            
            self.is_initialized = True
            response.success = True
            response.message = "Gripper initialized successfully"
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to initialize gripper: {str(e)}"
            
        self.get_logger().info(response.message)
        return response


    def activate_gripper_callback(self, request, response):
        """激活吸盘 - 启动真空吸盘"""
        self.get_logger().info('Activating gripper (enabling vacuum)')
        
        if not self.is_initialized:
            response.success = False
            response.message = "Gripper not initialized. Please initialize first."
            self.get_logger().warn(response.message)
            return response
            
        try:
            if REALMAN_API_AVAILABLE:
                # 先清除并设置 rACT=0 再设置 rACT=1 的控制指令
                # 构造写入参数结构体
                write_params = rm_peripheral_read_write_params_t(
                    self.comm_port,       # 通信端口号
                    self.register_address,  # 数据起始（寄存器地址）
                    self.device_address,  # 外部设备地址
                    1                     # 数据数量（对于单个寄存器写入固定为1）
                )
                
                # 设置 rACT
                # rAct 代表吸盘的使能状态其中0代表下使能，1代表上使能状态
                # 先写入0清除状态
                result = self.arm.rm_write_single_register(write_params, 0)
                if result != 0:
                    raise Exception(f"Failed to clear gripper register, error code: {result}")
                result = self.arm.rm_write_single_register(write_params, 1)
                if result != 0:
                    raise Exception(f"Failed to set gripper register, error code: {result}")                
                print("使能吸盘成功")
                
                # 等待一段时间
                time.sleep(0.5)
                
                # 吸盘启动，双通道
                # 写入29，激活通道1
                result = self.arm.rm_write_single_register(write_params, 29)
                if result != 0:
                    raise Exception(f"Failed to write Modbus register, error code: {result}")
                
                self.get_logger().info(f'Sending register write command: address={self.register_address}, data=29')
                # 写入 109，激活通道2
                result = self.arm.rm_write_single_register(write_params, 109)
                if result != 0:
                    raise Exception(f"Failed to write Modbus register, error code: {result}")
                
                self.get_logger().info(f'Sending register write command: address={self.register_address}, data=29')
            else:
                # 模拟发送write_single_register命令，data=0 (先清除)
                self.get_logger().info(f'Sending register write command: address={self.register_address}, data=0')
                time.sleep(0.1)
                
                # 模拟发送write_single_register命令，data=29 (真空吸盘启动)
                self.get_logger().info(f'Sending register write command: address={self.register_address}, data=29')
                time.sleep(0.1)
            
            response.success = True
            response.message = "Gripper activated (vacuum enabled)"
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to activate gripper: {str(e)}"
            
        self.get_logger().info(response.message)
        return response

    def deactivate_gripper_callback(self, request, response):
        """停用吸盘 - 停止真空并释放物体"""
        self.get_logger().info('Deactivating gripper (releasing vacuum)')
        
        if not self.is_initialized:
            response.success = False
            response.message = "Gripper not initialized. Please initialize first."
            self.get_logger().warn(response.message)
            return response
            
        try:
            if REALMAN_API_AVAILABLE:
                # 构造写入参数结构体
                write_params = rm_peripheral_read_write_params_t(
                    self.comm_port,       # 端口号
                    self.register_address,  # 寄存器地址
                    self.device_address,  # 设备地址
                    1                     # 数据数量（对于单个寄存器写入固定为1）
                )
                
                # 发送write_single_register命令，data=37 (真空吸盘松开)
                result = self.arm.rm_write_single_register(write_params, 37)
                if result != 0:
                    raise Exception(f"Failed to write Modbus register, error code: {result}")
                
                self.get_logger().info(f'Sending register write command: address={self.register_address}, data=37')
            else:
                # 模拟发送write_single_register命令，data=37 (真空吸盘松开)
                self.get_logger().info(f'Sending register write command: address={self.register_address}, data=37')
                time.sleep(0.1)
            
            response.success = True
            response.message = "Gripper deactivated (vacuum released)"
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to deactivate gripper: {str(e)}"
            
        self.get_logger().info(response.message)
        return response

    def close_modbus_callback(self, request, response):
        """关闭Modbus模式"""
        self.get_logger().info('Closing Modbus mode')
        
        if not self.is_initialized:
            response.success = False
            response.message = "Gripper not initialized."
            self.get_logger().warn(response.message)
            return response
            
        try:
            if REALMAN_API_AVAILABLE:
                # 关闭Modbus RTU模式
                self.get_logger().info('Closing Modbus RTU mode')
                result = self.arm.rm_close_modbus_mode(self.comm_port)
                if result != 0:
                    raise Exception(f"Failed to close Modbus mode, error code: {result}")
                
                # 删除机械臂连接
                self.arm.rm_delete_robot_arm()
            else:
                # 模拟关闭Modbus RTU模式
                self.get_logger().info('Closing Modbus RTU mode')
                time.sleep(0.2)
            
            self.is_initialized = False
            self.robot_handle = None
            self.arm = None
            self.robot_ip = None
            self.robot_port = None
            self.register_address = None
            self.comm_port = None
            self.device_address = None
            
            response.success = True
            response.message = "Modbus mode closed"
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to close Modbus mode: {str(e)}"
            
        self.get_logger().info(response.message)
        return response

    def _write_modbus_register(self, address, data):
        """
        通过Modbus写入单个寄存器
        
        :param address: 寄存器地址
        :param data: 要写入的数据
        :return: 是否成功
        """
        try:
            if REALMAN_API_AVAILABLE:
                self.get_logger().info(f'Writing to Modbus register {address} with data {data}')
                
                # 使用官方API写单个寄存器
                # 构造写入参数结构体
                write_params = rm_peripheral_read_write_params_t(
                    self.comm_port,       # 端口号
                    address,              # 寄存器地址
                    self.device_address,  # 设备地址
                    1                     # 数据数量（对于单个寄存器写入固定为1）
                )
                
                # 写入单个寄存器
                result = self.arm.rm_write_single_register(write_params, data)
                if result != 0:  # 返回值是状态码
                    raise Exception(f"Failed to write Modbus register, error code: {result}")
                
                return True
            else:
                # 在实际实现中，这里会通过TCP/IP发送Modbus RTU命令到机械臂
                # 模拟通信过程
                self.get_logger().info(f'Writing to Modbus register {address} with data {data}')
                time.sleep(0.1)  # 模拟通信延迟
                
                # 模拟成功响应
                return True
        except Exception as e:
            self.get_logger().error(f'Failed to write Modbus register: {str(e)}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = JodellEvs08ControlServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()