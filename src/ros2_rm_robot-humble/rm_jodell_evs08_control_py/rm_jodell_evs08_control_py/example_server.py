from robotic_arm import *
import socket
import time
 
ip = "192.168.1.18"
port = 8080
robot = Arm(RM65,ip,port)
 
#控制器寄存器地址
Control_Register_Address = int('03E8',16)
 
#通讯端口
c_port = 1
serve_address = (ip,port)
#设备地址
device_address = 9
 
def Socket_Connect():
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client.connect(serve_address)
        print("机械臂Socket连接成功")
        return True
    except socket.error as e:
        print("机械臂Socket连接失败")
        return False
 
if __name__ =="__main__":
 
    Socket_Connect()
 
    #设置工具端电压
    robot.Set_Tool_Voltage(3)
    print("设置工具端电源输出24V成功")
    time.sleep(1)
 
    #配置末端为Modbus RTU模式
    robot.Set_Modbus_Mode(1,115200,2)
    print("配置末端Modbus RTU成功")
 
    #请求激活，使能电动吸盘
    #清除并设置rACT = 0
    robot.Write_Single_Register(c_port, Control_Register_Address, 0, device_address) 
    
    # 设置rACT = 1
    time.sleep(0.5)
    robot.Write_Single_Register(c_port, Control_Register_Address, 1, device_address)  
    print("使能吸盘成功")
 
    #启动吸盘
    robot.Write_Single_Register(c_port, Control_Register_Address, int('001D',16), device_address) 
    print("启动吸盘成功")
    time.sleep(1)
 
    robot.Write_Single_Register(c_port, Control_Register_Address, 109, device_address)
    time.sleep(10)
 
    #以破真空方式停止吸盘
    robot.Write_Single_Register(c_port, Control_Register_Address, int('0025',16), device_address)
 
    #关闭末端Modbus RTU模式
    robot.Close_Modbus_Mode(1,False)
    print("末端Modbus RTU关闭成功")
 
    print("机械臂Socket关闭成功")