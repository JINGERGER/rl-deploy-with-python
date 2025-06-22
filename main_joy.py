import os
import sys
import threading
import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
import limxsdk.datatypes as datatypes
import controllers as controllers
from functools import partial

# ROS2相关导入
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyPublisherNode(Node):
    """ROS2节点用于发布Joy消息"""
    def __init__(self):
        super().__init__('limx_joy_publisher')
        self.publisher = self.create_publisher(Joy, 'joy', 10)
        self.get_logger().info("Joy publisher node initialized")

    def publish_joy(self, sensor_joy):
        """将SensorJoy数据转换为ROS2 Joy消息并发布"""
        joy_msg = Joy()
        
        # 设置消息头时间戳
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.header.frame_id = "joy_frame"
        
        # 复制轴数据
        joy_msg.axes = [float(axis) for axis in sensor_joy.axes]
        
        # 复制按钮数据
        joy_msg.buttons = [int(button) for button in sensor_joy.buttons]
        
        self.publisher.publish(joy_msg)
        self.get_logger().debug("Published Joy message")

def sensor_joy_callback(sensor_joy: datatypes.SensorJoy, joy_node=None):
    """处理SensorJoy数据的回调函数"""
    print("\n------\nSensorJoy received:" +
          "\n  stamp: " + str(sensor_joy.stamp) +
          "\n  axes: " + str(sensor_joy.axes) +
          "\n  buttons: " + str(sensor_joy.buttons))
    
    # 如果提供了ROS2节点，则发布Joy消息
    if joy_node:
        joy_node.publish_joy(sensor_joy)

def ros2_spin_thread(node):
    """ROS2自旋线程函数"""
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # 获取环境变量中的机器人类型
    robot_type = os.getenv("ROBOT_TYPE")
    
    if not robot_type:
        print("Error: Please set the ROBOT_TYPE using 'export ROBOT_TYPE=<robot_type>'.")
        sys.exit(1)

    # 初始化ROS2
    rclpy.init()
    joy_node = JoyPublisherNode()
    
    # 创建并启动ROS2自旋线程
    spin_thread = threading.Thread(target=ros2_spin_thread, args=(joy_node,), daemon=True)
    spin_thread.start()
    print("ROS2 Joy publisher node spinning in background thread")

    # 创建机器人实例
    robot = Robot(RobotType.PointFoot)
    robot_ip = "127.0.0.1"
    
    if len(sys.argv) > 1:
        robot_ip = sys.argv[1]

    if not robot.init(robot_ip):
        sys.exit()
    
    print("Subscribing to SensorJoy...")

    # 创建部分函数，将joy_node绑定到回调中
    joy_callback_with_node = partial(sensor_joy_callback, joy_node=joy_node)
    
    start_controller = robot_ip == "127.0.0.1"

    # 根据机器人类型创建控制器
    if robot_type.startswith("PF"):
        controller = controllers.PointfootController(
            f'{os.path.dirname(os.path.abspath(__file__))}/controllers/model', 
            robot, robot_type, start_controller)
        controller.run()
    elif robot_type.startswith("WF"):
        controller = controllers.WheelfootController(
            f'{os.path.dirname(os.path.abspath(__file__))}/controllers/model', 
            robot, robot_type, start_controller)
        # 订阅手柄数据，使用带有ROS2节点的回调函数
        robot.subscribeSensorJoy(joy_callback_with_node)
        controller.run()
    elif robot_type.startswith("SF"):
        controller = controllers.SolefootController(
            f'{os.path.dirname(os.path.abspath(__file__))}/controllers/model', 
            robot, robot_type, start_controller)
        controller.run()
    else:
        print(f"Error: unknown robot type '{robot_type}'")
    
    # 主线程结束时，ROS2线程会自动退出（因为是守护线程）