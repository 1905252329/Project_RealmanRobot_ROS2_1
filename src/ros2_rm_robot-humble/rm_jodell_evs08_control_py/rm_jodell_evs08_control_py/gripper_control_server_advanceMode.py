# 高级控制模式（自定义通道参数）
import rclpy
from rclpy.node import Node
# 使用rm_ros_interfaces中的服务定义
from rm_ros_interfaces.srv import InitializeGripper, ActivateGripper, DeactivateGripper, CloseModbus, GetParasGripper

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
        # self.declare_parameter('robot_ip', '169.254.247.18') # Lab429 Realman IP
        self.declare_parameter('robot_ip', '192.168.1.18') # Lab433 Realman IP
        self.declare_parameter('robot_port', 8080)
        self.declare_parameter('register_address', 1000)
        self.declare_parameter('comm_port', 1)
        self.declare_parameter('device_address', 9)
        self.declare_parameter('register_num',1)  # 读写的寄存器数量，默认1个寄存器

        # 获取参数值
        self.robot_ip           = self.get_parameter('robot_ip').value
        self.robot_port         = self.get_parameter('robot_port').value
        self.register_address   = self.get_parameter('register_address').value
        self.comm_port          = self.get_parameter('comm_port').value
        self.device_address     = self.get_parameter('device_address').value
        self.register_num       = self.get_parameter('register_num').value
        
        # ✨ 创建服务，使用相对名称以便支持命名空间
        self.init_service       = self.create_service(
            InitializeGripper, 'initialize_gripper', self.initialize_gripper_callback)
        self.activate_service   = self.create_service(
            ActivateGripper, 'activate_gripper', self.activate_gripper_callback)
        self.deactivate_service = self.create_service(
            DeactivateGripper, 'deactivate_gripper', self.deactivate_gripper_callback)
        self.close_service      = self.create_service(
            CloseModbus, 'close_modbus', self.close_modbus_callback)
        self.getParas_service = self.create_service(
            GetParasGripper, 'getParas_gripper', self.getParas_gripper_callback)

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
        self.get_logger().info('\n') 
        """初始化吸盘 - 连接机械臂并设置Modbus模式"""
        # 使用请求中的参数值，如果未提供则使用默认值
        robot_ip         = request.ip if request.ip else self.robot_ip
        robot_port       = request.port if request.port else self.robot_port
        register_address = request.register_address if request.register_address else self.register_address
        comm_port        = request.comm_port if request.comm_port else self.comm_port
        device_address   = request.device_address if request.device_address else self.device_address
        register_num     = request.register_num if request.register_num else self.register_num

        self.get_logger().info(f'Initializing gripper with IP: {robot_ip}, Port: {robot_port}')
        
        try:
            # 保存参数
            self.robot_ip         = robot_ip
            self.robot_port       = robot_port
            self.register_address = register_address
            self.comm_port        = comm_port
            self.device_address   = device_address
            self.register_num     = register_num

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
            
            self.is_initialized = True
            response.success = True
            response.message = "Gripper initialized successfully"
            # print()
            

        except Exception as e:
            response.success = False
            response.message = f"Failed to initialize gripper: {str(e)}"
            
        self.get_logger().info(response.message)
        return response
        

    def activate_gripper_callback(self, request, response):
        self.get_logger().info('\n') 
        """激活吸盘 - 启动真空吸盘"""
        self.get_logger().info('Activating gripper (enabling vacuum)')
        
        if not self.is_initialized:
            response.success = False
            response.message = "Gripper not initialized. Please initialize first."
            self.get_logger().warn(response.message)
            return response
            
        try:
            # 先清除并设置 rACT=0 再设置 rACT=1 的控制指令
            if not self.write_modbus_register(0x03E8, 1, [0]):
                raise Exception('Failed to clear rACT (write 0)')
            if not self.write_modbus_register(0x03E8, 1, [1]):
                raise Exception('Failed to set rACT (write 1)')
            print("使能吸盘成功")

            # 等待一段时间
            time.sleep(0.5)

            # === 设置通道 1 参数 ==
            # 注意这里第二次写入是2个寄存器（与 auto 版本行为一致）
            data_0 = [8192, 20520] # 参数：0x2000(68% 最大真空度)，参数2：0x5028(20% 最小真空度)                
            data_1 = [10240, 20520] # 参数：0x2800(60% 最大真空度)，参数2：0x5028(20% 最小真空度)
            data_2 = [16384, 20520] # 参数：0x4000(36% 最大真空度)，参数2：0x5028(20% 最小真空度)

            # if not self.write_modbus_register(1001, 2, data_2):
            #     raise Exception('Failed to write channel1 params')

            self.write_modbus_register(0x03E9, 1, 0x2800) # 最大真空度（高字节），预留无意义（低字节）
            self.write_modbus_register(0x03EA, 1, 0x500A) # 最小真空度（高字节），抓取超时（低字节）
            print("吸盘通道 1 参数设置成功")

            # === 设置通道 2 参数 ===
            data_0 = [7712, 80] # 参数：0x1E20(68% 最大真空度)，参数2：0x0050(20% 最小真空度)                
            data_1 = [7720, 80] # 参数：0x1E28(60% 最大真空度)，参数2：0x0050(20% 最小真空度)
            data_2 = [7744, 80] # 参数：0x1E40(36% 最大真空度)，参数2：0x0050(20% 最小真空度)

            # if not self.write_modbus_register(1003, 2, data_2):
            #     raise Exception('Failed to write channel2 params')

            self.write_modbus_register(0x03EB, 1, 0x1E28) # 抓取超时（高字节），最大真空度（低字节）
            self.write_modbus_register(0x03EC, 1, 0x0050) # 预留无意义（高字节），最小真空度（低字节）  
            print("吸盘通道 2 参数设置成功")

            # 等待一段时间
            time.sleep(0.5)

            # 吸盘双通道启动，使用与 auto 版本不一致的控制码（15/127 表示双通道启动）
            if not self.write_modbus_register(1000, 1, [15]):
                raise Exception('Failed to start channel1 (write 15)')
            self.get_logger().info(f'Sending register write command: address=1000, data = 15')

            if not self.write_modbus_register(1000, 1, [127]):
                raise Exception('Failed to start channel2 (write 127)')
            self.get_logger().info(f'Sending register write command: address=1000, data = 127')
            time.sleep(0.5)
            
            # 读取通道1和通道2的实际真空度/压力寄存器（高字节）并打印十进制数值
            addr_ch1 = 0x07D2
            addr_ch2 = 0x07D5
            vac1 = self.read_modbus_register(addr_ch1)
            vac2 = self.read_modbus_register(addr_ch2)

            if vac1 is not None:
                self.get_logger().info(f'Channel1 vacuum (high byte) = {vac1}')
            else:
                self.get_logger().warn(f'Failed to read Channel1 vacuum from register 0x{addr_ch1:04X}')

            if vac2 is not None:
                self.get_logger().info(f'Channel2 vacuum (high byte) = {vac2}')
            else:
                self.get_logger().warn(f'Failed to read Channel2 vacuum from register 0x{addr_ch2:04X}') 

            
            response.success = True
            response.message = "Gripper activated (vacuum enabled)"
            # print()
                     
        except Exception as e:
            response.success = False
            response.message = f"Failed to activate gripper: {str(e)}"
            
        self.get_logger().info(response.message)
        return response
         

    def deactivate_gripper_callback(self, request, response):
        self.get_logger().info('\n') # 换行
        """停用吸盘 - 停止真空并释放物体"""
        self.get_logger().info('Deactivating gripper (releasing vacuum)')
        
        if not self.is_initialized:
            response.success = False
            response.message = "Gripper not initialized. Please initialize first."
            self.get_logger().warn(response.message)
            return response
            
        try:
            if not self.write_modbus_register(1000, 1, [55]):
                raise Exception('Failed to deactivate gripper (write 55)')
            self.get_logger().info(f'Sending register write command: address=1000, data = 55')
            
            response.success = True
            response.message = "Gripper deactivated (vacuum released)"
            print()

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
            # 关闭Modbus RTU模式
            self.get_logger().info('Closing Modbus RTU mode')
            result = self.arm.rm_close_modbus_mode(self.comm_port)
            if result != 0:
                raise Exception(f"Failed to close Modbus mode, error code: {result}")
            
            # 删除机械臂连接
            self.arm.rm_delete_robot_arm()
            
            self.is_initialized   = False
            self.robot_handle     = None
            self.arm              = None
            self.robot_ip         = None
            self.robot_port       = None
            self.register_address = None
            self.comm_port        = None
            self.device_address   = None
            
            response.success = True
            response.message = "Modbus mode closed"
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to close Modbus mode: {str(e)}"
            
        self.get_logger().info(response.message)
        return response

    def getParas_gripper_callback(self, request, response):
        self.get_logger().info('\n') # 换行
        """获取吸盘参数"""
        self.get_logger().info('Get gripper parameters')
        
        if not self.is_initialized:
            response.success = False
            response.message = "Gripper not initialized. Please initialize first."
            self.get_logger().warn(response.message)
            return response
            
        try:
            # 读取通道1和通道2的最大/实际真空度/压力寄存器（高字节）并打印十进制数值
            addr_ch1 = 0x07D2
            addr_ch2 = 0x07D5
            vac1 = self.read_modbus_register(addr_ch1)
            vac2 = self.read_modbus_register(addr_ch2)

            if vac1 is not None:
                self.get_logger().info(f'Channel_1 vacuum (high byte) = {vac1}')
                response.channel1_vacuum = vac1 
            else:
                self.get_logger().warn(f'Failed to read Channel_1 vacuum from register 0x{addr_ch1:04X}')
                response.channel1_vacuum = -1  # 使用-1表示读取失败

            if vac2 is not None:
                self.get_logger().info(f'Channel_2 vacuum (high byte) = {vac2}')
                response.channel2_vacuum = vac2
            else:
                self.get_logger().warn(f'Failed to read Channel_2 vacuum from register 0x{addr_ch2:04X}') 
                response.channel2_vacuum = -1  # 使用-1表示读取失败

            response.success = True
            response.message = "Get gripper parameters successfully"
            print()

        except Exception as e:
            response.success = False
            response.message = f"Failed to get gripper parameters: {str(e)}"
            # 发生异常时设置无效值
            response.channel1_vacuum = -1
            response.channel2_vacuum = -1
            
        self.get_logger().info(response.message)
        return response

    def write_modbus_register(self, register_address, register_num, data):
        """
        通过Modbus写入单个寄存器
        
        :param address: 寄存器地址
        :param data: 要写入的数据
        :return: 是否成功
        """
        try:
            self.get_logger().info(f'Writing to Modbus register {register_address} with data {data} (num={register_num})')

            # 验证 data 长度与 register_num 一致
            if isinstance(data, list):
                data_len = len(data)
            else:
                # 允许直接传入单个整数
                data = [data]
                data_len = 1

            if data_len != register_num:
                self.get_logger().warn(f'Data length ({data_len}) != register_num ({register_num}), adjusting register_num to {data_len}')
                register_num = data_len

            # 构造写入参数结构体
            write_params = rm_peripheral_read_write_params_t(
                self.comm_port,       # 端口号
                register_address,     # 寄存器地址
                self.device_address,  # 设备地址
                register_num          # 数据数量
            )

            # 根据寄存器数量选择调用单寄存器或多寄存器接口
            if register_num == 1:
                result = self.arm.rm_write_single_register(write_params, data[0])
            else:
                result = self.arm.rm_write_registers(write_params, data)

            if result != 0:  # 返回值是状态码
                raise Exception(f"Failed to write Modbus register, error code: {result}")

            self.get_logger().info(f'Writing to Modbus register result is {result}')
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to write Modbus register: {str(e)}')
            return False

    def read_modbus_register(self, register_address):
        """
        通过Modbus读取寄存器

        :param register_address: 寄存器起始地址
        :param register_num: 要读取的寄存器数量
        :return: 列表形式的寄存器值或 None
        """
        try:
            self.get_logger().info(f'Reading Modbus register {register_address} ')
            if not REALMAN_API_AVAILABLE:
                # 模拟返回
                return 0
            
            # read_params 的 num 字段文档说明：读取保持寄存器时无需设置 num（每次只能读 1 个）
            read_params = rm_peripheral_read_write_params_t(
                self.comm_port,
                register_address,
                self.device_address,
                1
            )

            # 调用 API：返回 (status:int, value:int)
            status, value = self.arm.rm_read_input_registers(read_params)
            self.get_logger().info(f'Read Modbus register result: status={status}, value={value}')

            if isinstance(status, int) and status == 0:
                # value 为寄存器原始整数（可能为有符号或无符号，依据设备协议）
                # 若需要把 16-bit 高字节解释为无符号，可做 & 0xFFFF
                return int(value) & 0xFFFF
            else:
                self.get_logger().error(f'rm_read_holding_registers failed, code={status}')
                return None


        except Exception as e:
            self.get_logger().error(f'Failed to read Modbus register: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = JodellEvs08ControlServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()