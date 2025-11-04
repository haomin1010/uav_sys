#!/usr/bin/env python3
"""
监控脚本：实时显示系统状态
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime


class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        
        # 订阅状态话题
        self.sub_arbiter_status = self.create_subscription(
            String,
            '/uav/arbiter/status',
            self.on_arbiter_status,
            10
        )
        
        self.sub_sync_status = self.create_subscription(
            String,
            '/uav/sync/status',
            self.on_sync_status,
            10
        )
        
        self.sub_authoritative = self.create_subscription(
            String,
            '/uav/authoritative_cmd',
            self.on_authoritative_cmd,
            10
        )
        
        self.get_logger().info('系统监控器已启动')
        self.get_logger().info('=' * 60)
    
    def on_arbiter_status(self, msg: String):
        """显示仲裁器状态"""
        try:
            status = json.loads(msg.data)
            current_source = status.get('current_source', 'None')
            
            print(f"\n{'='*60}")
            print(f"[{datetime.now().strftime('%H:%M:%S')}] 仲裁器状态")
            print(f"{'='*60}")
            print(f"当前生效源: {current_source if current_source else '无'}")
            print(f"\n决策源状态:")
            
            for source_id, info in status.get('sources', {}).items():
                enabled = "✓" if info.get('enabled') else "✗"
                has_cmd = "✓" if info.get('has_cmd') else "✗"
                valid = "✓" if info.get('cmd_valid') else "✗"
                age = info.get('last_hb_age', 999)
                
                symbol = "→" if source_id == current_source else " "
                
                print(f"  {symbol} {source_id:8s} | "
                      f"优先级:{info.get('priority', 0):3d} | "
                      f"启用:{enabled} | "
                      f"有命令:{has_cmd} | "
                      f"有效:{valid} | "
                      f"心跳:{age:.2f}s前")
            
        except Exception as e:
            self.get_logger().error(f'解析仲裁器状态失败: {e}')
    
    def on_sync_status(self, msg: String):
        """显示同步状态"""
        try:
            status = json.loads(msg.data)
            synced = status.get('overall_synced', False)
            synced_str = "✓ 同步" if synced else "✗ 失步"
            
            print(f"\n同步状态: {synced_str}")
            print(f"平台状态:")
            
            for platform, info in status.get('platforms', {}).items():
                alive = "✓" if info.get('alive') else "✗"
                age = info.get('state_age', 999)
                has_pos = "✓" if info.get('has_position') else "✗"
                
                print(f"  - {platform:8s}: 在线:{alive} | "
                      f"状态更新:{age:.2f}s前 | "
                      f"位置数据:{has_pos}")
            
        except Exception as e:
            self.get_logger().error(f'解析同步状态失败: {e}')
    
    def on_authoritative_cmd(self, msg: String):
        """显示权威命令"""
        try:
            from uav_decision_arbiter.command_msg import CommandMsg
            cmd = CommandMsg.from_json(msg.data)
            
            print(f"\n>> 权威命令 [来源: {cmd.header.source_id}, seq: {cmd.header.seq}]")
            print(f"   模式: {cmd.meta.mode}, 优先级: {cmd.meta.priority}")
            
            if cmd.body.velocity:
                vel = cmd.body.velocity
                print(f"   速度: vx={vel.vx:.2f}, vy={vel.vy:.2f}, "
                      f"vz={vel.vz:.2f}, yaw_rate={vel.yaw_rate:.2f}")
            elif cmd.body.setpoint:
                pos = cmd.body.setpoint
                print(f"   位置: x={pos.x:.2f}, y={pos.y:.2f}, "
                      f"z={pos.z:.2f}, yaw={pos.yaw:.2f}")
            elif cmd.body.trajectory:
                print(f"   轨迹: {len(cmd.body.trajectory)}个点")
            
        except Exception as e:
            self.get_logger().error(f'解析权威命令失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n监控器已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

