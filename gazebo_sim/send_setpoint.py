#!/usr/bin/env python3
"""
持续发送 setpoint 到仲裁系统（AUTOPILOT源）
使用正确的时间戳，通过仲裁器统一管理
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import sys

class SetpointPublisher(Node):
    def __init__(self, uav_name, altitude, x=0.0, y=0.0):
        super().__init__('setpoint_publisher')
        self.uav_name = uav_name
        self.altitude = altitude
        self.target_x = x
        self.target_y = y
        
        # 发布到仲裁系统（AUTOPILOT源）
        self.publisher = self.create_publisher(
            String,
            '/uav/source/autopilot/cmd',
            10
        )
        
        # 20Hz 发送
        self.timer = self.create_timer(0.05, self.publish_command)
        
        self.seq = 0
        self.get_logger().info(f'[{uav_name}] AUTOPILOT 开始发送位置命令')
        self.get_logger().info(f'  目标: x={x:.2f}, y={y:.2f}, z={altitude:.2f}')
        self.get_logger().info(f'  频率: 20Hz, 优先级: 120')
    
    def publish_command(self):
        """发布统一格式的位置命令"""
        current_time = time.time()
        self.seq += 1
        
        # 构建统一命令格式
        command = {
            "header": {
                "timestamp": current_time,
                "source_id": "autopilot",
                "seq": self.seq,
                "target_uav_id": self.uav_name  # 目标无人机ID
            },
            "meta": {
                "priority": 100,  # AUTOPILOT 优先级（根据你的修改）
                "mode": "position",
                "expires_at": current_time + 0.5
            },
            "body": {
                "setpoint": {
                    "x": self.target_x,
                    "y": self.target_y,
                    "z": self.altitude,
                    "yaw": 0.0
                }
            }
        }
        
        msg = String()
        msg.data = json.dumps(command)
        self.publisher.publish(msg)
        
        self.seq += 1
        if self.seq % 100 == 0:
            self.get_logger().info(f'[{self.uav_name}] 已发送 {self.seq} 个命令')

def main(args=None):
    if len(sys.argv) < 2:
        print("用法: ./send_setpoint.py <uav_name> [altitude] [x] [y]")
        print("示例: ./send_setpoint.py uav1 2.0")
        print("示例: ./send_setpoint.py uav2 2.0 5.0 3.0")
        sys.exit(1)
    
    uav_name = sys.argv[1]
    altitude = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    x = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    y = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    
    # 初始化ROS2（使用独立的context）
    import signal
    
    # 捕获信号
    def signal_handler(sig, frame):
        print(f'\n[{uav_name}] 收到停止信号，退出...')
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    rclpy.init(args=args)
    node = SetpointPublisher(uav_name, altitude, x, y)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f'\n[{uav_name}] 停止 setpoint 发送')
    except Exception as e:
        print(f'\n[{uav_name}] 异常退出: {e}')
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()

