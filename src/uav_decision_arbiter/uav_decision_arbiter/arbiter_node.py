#!/usr/bin/env python3
"""
决策仲裁器节点
功能：接收三路决策源命令，根据优先级和时效性选择当前生效的命令
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from typing import Dict, Optional, Tuple
from .command_msg import CommandMsg, SourceID, PRIORITY_MAP


class ArbiterNode(Node):
    """决策仲裁器节点"""
    
    def __init__(self):
        super().__init__('arbiter_node')
        
        # 参数配置
        self.declare_parameter('hysteresis_ms', 200)  # 防抖窗口（毫秒）
        self.declare_parameter('heartbeat_timeout', 2.0)  # 心跳超时（秒）
        self.declare_parameter('uav_ids', ['uav1', 'uav2', 'uav3'])  # 管理的无人机ID列表
        self.declare_parameter('centralized_mode', True)  # 集中仲裁模式
        
        self.hysteresis_ms = self.get_parameter('hysteresis_ms').value
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').value
        self.uav_ids = self.get_parameter('uav_ids').value
        self.centralized_mode = self.get_parameter('centralized_mode').value
        
        # 集中仲裁模式：为每架无人机维护独立的决策状态
        # 数据结构: uav_states[uav_id][source_id] = {priority, last_cmd, last_hb, enabled}
        self.uav_states = {}
        for uav_id in self.uav_ids:
            self.uav_states[uav_id] = {
                "sources": {
                    SourceID.HUMAN.value: {
                        "priority": PRIORITY_MAP[SourceID.HUMAN],
                        "last_cmd": None,
                        "last_hb": 0.0,
                        "enabled": True
                    },
                    SourceID.CENTRAL.value: {
                        "priority": PRIORITY_MAP[SourceID.CENTRAL],
                        "last_cmd": None,
                        "last_hb": 0.0,
                        "enabled": True
                    },
                    SourceID.RL.value: {
                        "priority": PRIORITY_MAP[SourceID.RL],
                        "last_cmd": None,
                        "last_hb": 0.0,
                        "enabled": True
                    }
                },
                "current_source": None,  # 当前生效的决策源
                "last_switch_time": 0.0   # 上次切换时间
            }
        
        # 订阅各决策源的命令话题
        self.sub_human = self.create_subscription(
            String,
            '/uav/source/human/cmd',
            lambda msg: self.on_command(msg, SourceID.HUMAN.value),
            10
        )
        
        self.sub_central = self.create_subscription(
            String,
            '/uav/source/central/cmd',
            lambda msg: self.on_command(msg, SourceID.CENTRAL.value),
            10
        )
        
        self.sub_rl = self.create_subscription(
            String,
            '/uav/source/rl/cmd',
            lambda msg: self.on_command(msg, SourceID.RL.value),
            10
        )
        
        # 发布仲裁后的权威命令
        self.pub_authoritative = self.create_publisher(
            String,
            '/uav/authoritative_cmd',
            10
        )
        
        # 发布当前状态信息（用于监控）
        self.pub_status = self.create_publisher(
            String,
            '/uav/arbiter/status',
            10
        )
        
        # 定时器：定期评估和发布状态
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        
        self.get_logger().info('决策仲裁器节点已启动')
        self.get_logger().info(f'模式: {"集中仲裁" if self.centralized_mode else "单机"}')
        self.get_logger().info(f'管理无人机: {self.uav_ids}')
        self.get_logger().info(f'优先级设置: 人类={PRIORITY_MAP[SourceID.HUMAN]}, '
                             f'中央算力={PRIORITY_MAP[SourceID.CENTRAL]}, '
                             f'RL={PRIORITY_MAP[SourceID.RL]}')
    
    def now(self) -> float:
        """获取当前时间戳（秒）"""
        return time.time()
    
    def on_command(self, msg: String, source_id: str):
        """接收命令回调"""
        try:
            # 解析命令消息
            cmd = CommandMsg.from_json(msg.data)
            
            # 验证source_id匹配
            if cmd.header.source_id != source_id:
                self.get_logger().warning(
                    f'源ID不匹配: 话题={source_id}, 消息={cmd.header.source_id}'
                )
                return
            
            current_time = self.now()
            target_uav = cmd.header.target_uav_id
            
            # 集中仲裁模式：根据target_uav_id处理
            if target_uav == "all":
                # 广播命令：更新所有无人机
                for uav_id in self.uav_ids:
                    self.uav_states[uav_id]["sources"][source_id]["last_cmd"] = cmd
                    self.uav_states[uav_id]["sources"][source_id]["last_hb"] = current_time
                    self.evaluate(uav_id)
                    
                self.get_logger().debug(
                    f'收到广播命令 source={source_id}, seq={cmd.header.seq}, '
                    f'mode={cmd.meta.mode}, 应用到所有无人机'
                )
            else:
                # 定向命令：只更新指定无人机
                if target_uav not in self.uav_states:
                    self.get_logger().warning(
                        f'未知的无人机ID: {target_uav}, 忽略命令'
                    )
                    return
                
                self.uav_states[target_uav]["sources"][source_id]["last_cmd"] = cmd
                self.uav_states[target_uav]["sources"][source_id]["last_hb"] = current_time
                
                self.get_logger().debug(
                    f'收到定向命令 target={target_uav}, source={source_id}, '
                    f'seq={cmd.header.seq}, mode={cmd.meta.mode}'
                )
                
                # 立即评估该无人机
                self.evaluate(target_uav)
            
        except Exception as e:
            self.get_logger().error(f'解析命令失败: {e}')
    
    def evaluate(self, uav_id: str):
        """评估并选择指定无人机的当前生效决策源"""
        current_time = self.now()
        uav_state = self.uav_states[uav_id]
        sources = uav_state["sources"]
        
        # 收集所有有效的候选命令
        candidates = []
        
        for source_id, info in sources.items():
            if not info["enabled"]:
                continue
            
            cmd = info["last_cmd"]
            
            # 检查是否有命令且未过期
            if cmd is None:
                continue
            
            if not cmd.is_valid(current_time):
                # 命令已过期
                if cmd.is_expired(current_time):
                    self.get_logger().debug(f'[{uav_id}] {source_id} 的命令已过期')
                continue
            
            # 检查心跳是否超时
            if current_time - info["last_hb"] > self.heartbeat_timeout:
                self.get_logger().warning(f'[{uav_id}] {source_id} 心跳超时')
                continue
            
            # 添加到候选列表：(优先级, 时间戳, source_id, cmd)
            candidates.append((
                info["priority"],
                cmd.header.timestamp,
                source_id,
                cmd
            ))
        
        # 如果没有有效命令，发布安全悬停命令
        if not candidates:
            if uav_state["current_source"] is not None:
                self.get_logger().warning(f'[{uav_id}] 没有有效命令源，切换到安全模式')
                uav_state["current_source"] = None
                self.publish_safe_hold(uav_id)
            return
        
        # 按优先级降序，然后按时间戳降序排序
        candidates.sort(key=lambda x: (-x[0], -x[1]))
        
        # 选择最高优先级的候选
        chosen_priority, chosen_ts, chosen_source, chosen_cmd = candidates[0]
        
        # 判断是否需要切换
        if self.should_switch(uav_id, chosen_source, chosen_priority):
            old_source = uav_state["current_source"]
            uav_state["current_source"] = chosen_source
            uav_state["last_switch_time"] = current_time
            
            self.get_logger().info(
                f'[{uav_id}] 决策源切换: {old_source} -> {chosen_source} '
                f'(优先级={chosen_priority})'
            )
            
            # 发布新的权威命令
            self.publish_authoritative(uav_id, chosen_cmd)
        else:
            # 即使不切换，也可能需要更新命令（同一源的新命令）
            if uav_state["current_source"] == chosen_source:
                self.publish_authoritative(uav_id, chosen_cmd)
    
    def should_switch(self, uav_id: str, new_source: str, new_priority: int) -> bool:
        """判断指定无人机是否应该切换决策源"""
        uav_state = self.uav_states[uav_id]
        current_source = uav_state["current_source"]
        
        # 如果当前没有激活源，直接切换
        if current_source is None:
            return True
        
        # 如果是同一个源，不算切换
        if current_source == new_source:
            return False
        
        current_priority = uav_state["sources"][current_source]["priority"]
        
        # 如果新源优先级更高，立即切换
        if new_priority > current_priority:
            return True
        
        # 如果新源优先级更低，不切换
        if new_priority < current_priority:
            return False
        
        # 优先级相同，考虑防抖窗口
        time_since_last_switch = (self.now() - uav_state["last_switch_time"]) * 1000  # 转为毫秒
        if time_since_last_switch < self.hysteresis_ms:
            return False
        
        return True
    
    def publish_authoritative(self, uav_id: str, cmd: CommandMsg):
        """发布权威命令（确保包含目标无人机ID）"""
        # 确保命令包含正确的target_uav_id
        cmd.header.target_uav_id = uav_id
        
        msg = String()
        msg.data = cmd.to_json()
        self.pub_authoritative.publish(msg)
        
        self.get_logger().debug(
            f'发布权威命令: target={uav_id}, source={cmd.header.source_id}, '
            f'mode={cmd.meta.mode}, seq={cmd.header.seq}'
        )
    
    def publish_safe_hold(self, uav_id: str):
        """发布安全悬停命令"""
        # 创建一个悬停命令（速度为0）
        from .command_msg import Header, Meta, Body, Velocity, CommandMode
        
        current_time = self.now()
        
        cmd = CommandMsg(
            header=Header(
                timestamp=current_time,
                source_id="arbiter",
                seq=0,
                target_uav_id=uav_id
            ),
            meta=Meta(
                priority=255,  # 最高优先级
                mode=CommandMode.VELOCITY.value,
                expires_at=current_time + 1.0
            ),
            body=Body(
                velocity=Velocity(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0)
            )
        )
        
        msg = String()
        msg.data = cmd.to_json()
        self.pub_authoritative.publish(msg)
        
        self.get_logger().info(f'[{uav_id}] 发布安全悬停命令')
    
    def timer_callback(self):
        """定时回调：发布状态信息"""
        current_time = self.now()
        
        status = {
            "timestamp": current_time,
            "mode": "centralized" if self.centralized_mode else "single",
            "uav_count": len(self.uav_ids),
            "uavs": {}
        }
        
        # 为每架无人机生成状态
        for uav_id, uav_state in self.uav_states.items():
            uav_status = {
                "current_source": uav_state["current_source"],
                "sources": {}
            }
            
            for source_id, info in uav_state["sources"].items():
                cmd = info["last_cmd"]
                uav_status["sources"][source_id] = {
                    "priority": info["priority"],
                    "enabled": info["enabled"],
                    "last_hb_age": current_time - info["last_hb"],
                    "has_cmd": cmd is not None,
                    "cmd_valid": cmd.is_valid(current_time) if cmd else False
                }
            
            status["uavs"][uav_id] = uav_status
        
        msg = String()
        msg.data = json.dumps(status, indent=2)
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArbiterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

