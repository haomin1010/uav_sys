"""
RL平台可视化界面
使用pygame显示多架无人机的实时状态
"""
import pygame
import numpy as np
from typing import Dict, List, Optional
import math


class UAVVisualizer:
    """无人机可视化器"""
    
    def __init__(self, uav_ids: List[str], window_size: tuple = (1200, 800)):
        pygame.init()
        
        self.uav_ids = uav_ids
        self.num_uavs = len(uav_ids)
        
        # 窗口设置
        self.window_size = window_size
        self.screen = pygame.display.set_mode(window_size)
        pygame.display.set_caption('UAV RL Platform - 多机决策可视化')
        
        # 字体
        self.font = pygame.font.Font(None, 24)
        self.font_small = pygame.font.Font(None, 18)
        
        # 颜色
        self.colors = {
            'background': (240, 240, 245),
            'grid': (200, 200, 210),
            'uav': [(255, 100, 100), (100, 255, 100), (100, 100, 255), 
                    (255, 255, 100), (255, 100, 255), (100, 255, 255)],
            'target': (255, 150, 0),
            'trajectory': (150, 150, 200),
            'text': (50, 50, 50),
            'info_bg': (255, 255, 255),
            'active_source': (100, 255, 100),
            'inactive_source': (200, 200, 200)
        }
        
        # 可视化参数
        self.scale = 15.0  # 像素/米
        self.center_offset = np.array([window_size[0] // 3, window_size[1] // 2])
        
        # 轨迹历史
        self.trajectories: Dict[str, List[np.ndarray]] = {
            uav_id: [] for uav_id in uav_ids
        }
        self.max_trajectory_length = 100
        
        # 信息面板
        self.info_panel_width = 350
        
        # 时钟
        self.clock = pygame.time.Clock()
        
        # 状态信息
        self.current_source = None  # 当前控制源
        self.formation_info = None  # 编队信息
    
    def world_to_screen(self, world_pos: np.ndarray, view: str = 'top') -> tuple:
        """世界坐标转屏幕坐标"""
        if view == 'top':
            # 俯视图：x-y平面
            screen_x = world_pos[0] * self.scale + self.center_offset[0]
            screen_y = -world_pos[1] * self.scale + self.center_offset[1]  # y轴翻转
        else:
            # 侧视图：x-z平面
            screen_x = world_pos[0] * self.scale + self.center_offset[0]
            screen_y = -world_pos[2] * self.scale + self.center_offset[1]
        
        return (int(screen_x), int(screen_y))
    
    def draw_grid(self):
        """绘制网格"""
        grid_color = self.colors['grid']
        
        # 绘制水平和垂直网格线
        for i in range(-20, 21, 5):
            # 水平线
            start = self.world_to_screen(np.array([i, -20, 0]))
            end = self.world_to_screen(np.array([i, 20, 0]))
            pygame.draw.line(self.screen, grid_color, start, end, 1)
            
            # 垂直线
            start = self.world_to_screen(np.array([-20, i, 0]))
            end = self.world_to_screen(np.array([20, i, 0]))
            pygame.draw.line(self.screen, grid_color, start, end, 1)
        
        # 绘制坐标轴
        origin = self.world_to_screen(np.array([0, 0, 0]))
        x_axis = self.world_to_screen(np.array([5, 0, 0]))
        y_axis = self.world_to_screen(np.array([0, 5, 0]))
        
        pygame.draw.line(self.screen, (255, 0, 0), origin, x_axis, 2)  # X轴红色
        pygame.draw.line(self.screen, (0, 255, 0), origin, y_axis, 2)  # Y轴绿色
        
        # 标注
        text = self.font_small.render('X(East)', True, (255, 0, 0))
        self.screen.blit(text, x_axis)
        text = self.font_small.render('Y(North)', True, (0, 255, 0))
        self.screen.blit(text, y_axis)
    
    def draw_uav(self, position: np.ndarray, uav_id: str, idx: int, 
                 velocity: Optional[np.ndarray] = None):
        """绘制单架无人机"""
        screen_pos = self.world_to_screen(position)
        color = self.colors['uav'][idx % len(self.colors['uav'])]
        
        # 无人机本体（圆形）
        pygame.draw.circle(self.screen, color, screen_pos, 12, 0)
        pygame.draw.circle(self.screen, (0, 0, 0), screen_pos, 12, 2)
        
        # ID标签
        text = self.font_small.render(uav_id, True, (255, 255, 255))
        text_rect = text.get_rect(center=screen_pos)
        self.screen.blit(text, text_rect)
        
        # 速度向量
        if velocity is not None and np.linalg.norm(velocity) > 0.1:
            vel_scale = 50
            end_pos = (
                int(screen_pos[0] + velocity[0] * vel_scale),
                int(screen_pos[1] - velocity[1] * vel_scale)
            )
            pygame.draw.line(self.screen, color, screen_pos, end_pos, 2)
            # 箭头
            pygame.draw.circle(self.screen, color, end_pos, 4, 0)
    
    def draw_target(self, position: np.ndarray, uav_id: str):
        """绘制目标点"""
        screen_pos = self.world_to_screen(position)
        color = self.colors['target']
        
        # 目标标记（X形）
        size = 8
        pygame.draw.line(self.screen, color,
                        (screen_pos[0] - size, screen_pos[1] - size),
                        (screen_pos[0] + size, screen_pos[1] + size), 2)
        pygame.draw.line(self.screen, color,
                        (screen_pos[0] + size, screen_pos[1] - size),
                        (screen_pos[0] - size, screen_pos[1] + size), 2)
        
        # 圆圈
        pygame.draw.circle(self.screen, color, screen_pos, 10, 2)
    
    def draw_trajectory(self, uav_id: str):
        """绘制轨迹"""
        if uav_id not in self.trajectories or len(self.trajectories[uav_id]) < 2:
            return
        
        color = self.colors['trajectory']
        points = [self.world_to_screen(pos) for pos in self.trajectories[uav_id]]
        
        if len(points) > 1:
            pygame.draw.lines(self.screen, color, False, points, 1)
    
    def draw_info_panel(self, uav_states: Dict, current_source: str = None,
                       formation_info: Dict = None, rl_step: int = 0):
        """绘制信息面板"""
        panel_x = self.window_size[0] - self.info_panel_width
        
        # 背景
        pygame.draw.rect(self.screen, self.colors['info_bg'],
                        (panel_x, 0, self.info_panel_width, self.window_size[1]))
        pygame.draw.line(self.screen, self.colors['grid'],
                        (panel_x, 0), (panel_x, self.window_size[1]), 2)
        
        y_offset = 20
        
        # 标题
        title = self.font.render('RL Decision Platform', True, self.colors['text'])
        self.screen.blit(title, (panel_x + 20, y_offset))
        y_offset += 40
        
        # 当前控制源
        source_text = f'Control: {current_source if current_source else "None"}'
        color = self.colors['active_source'] if current_source == 'rl' else self.colors['inactive_source']
        text = self.font_small.render(source_text, True, color)
        self.screen.blit(text, (panel_x + 20, y_offset))
        y_offset += 30
        
        # RL步数
        step_text = f'RL Step: {rl_step}'
        text = self.font_small.render(step_text, True, self.colors['text'])
        self.screen.blit(text, (panel_x + 20, y_offset))
        y_offset += 40
        
        # 分隔线
        pygame.draw.line(self.screen, self.colors['grid'],
                        (panel_x + 10, y_offset), 
                        (self.window_size[0] - 10, y_offset), 1)
        y_offset += 20
        
        # 无人机状态
        for idx, (uav_id, state) in enumerate(uav_states.items()):
            color = self.colors['uav'][idx % len(self.colors['uav'])]
            
            # UAV ID
            text = self.font.render(uav_id.upper(), True, color)
            self.screen.blit(text, (panel_x + 20, y_offset))
            y_offset += 25
            
            # 位置
            pos = state['position']
            pos_text = f'  Pos: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})'
            text = self.font_small.render(pos_text, True, self.colors['text'])
            self.screen.blit(text, (panel_x + 20, y_offset))
            y_offset += 20
            
            # 速度
            vel = state['velocity']
            speed = np.linalg.norm(vel)
            vel_text = f'  Spd: {speed:.2f} m/s'
            text = self.font_small.render(vel_text, True, self.colors['text'])
            self.screen.blit(text, (panel_x + 20, y_offset))
            y_offset += 20
            
            # 目标距离
            if 'target_distance' in state:
                dist_text = f'  Dst: {state["target_distance"]:.2f} m'
                text = self.font_small.render(dist_text, True, self.colors['text'])
                self.screen.blit(text, (panel_x + 20, y_offset))
            
            y_offset += 30
        
        # 编队信息
        if formation_info:
            y_offset += 10
            pygame.draw.line(self.screen, self.colors['grid'],
                            (panel_x + 10, y_offset), 
                            (self.window_size[0] - 10, y_offset), 1)
            y_offset += 20
            
            text = self.font.render('Formation Info', True, self.colors['text'])
            self.screen.blit(text, (panel_x + 20, y_offset))
            y_offset += 30
            
            if 'leader_uav' in formation_info:
                leader_text = f'Leader: {formation_info["leader_uav"]}'
                text = self.font_small.render(leader_text, True, self.colors['text'])
                self.screen.blit(text, (panel_x + 20, y_offset))
                y_offset += 25
        
        # 控制说明
        y_offset = self.window_size[1] - 100
        pygame.draw.line(self.screen, self.colors['grid'],
                        (panel_x + 10, y_offset), 
                        (self.window_size[0] - 10, y_offset), 1)
        y_offset += 10
        
        controls = [
            'Controls:',
            'SPACE - Pause/Resume',
            'R - Reset',
            'Q - Quit'
        ]
        
        for ctrl in controls:
            text = self.font_small.render(ctrl, True, self.colors['text'])
            self.screen.blit(text, (panel_x + 20, y_offset))
            y_offset += 20
    
    def update_trajectory(self, uav_id: str, position: np.ndarray):
        """更新轨迹"""
        if uav_id in self.trajectories:
            self.trajectories[uav_id].append(position.copy())
            
            # 限制轨迹长度
            if len(self.trajectories[uav_id]) > self.max_trajectory_length:
                self.trajectories[uav_id].pop(0)
    
    def render(self, uav_states: Dict, targets: Dict = None, 
              current_source: str = None, formation_info: Dict = None,
              rl_step: int = 0):
        """渲染一帧"""
        # 清屏
        self.screen.fill(self.colors['background'])
        
        # 绘制网格
        self.draw_grid()
        
        # 绘制目标点
        if targets:
            for uav_id, target_pos in targets.items():
                self.draw_target(target_pos, uav_id)
        
        # 绘制轨迹
        for uav_id in self.uav_ids:
            self.draw_trajectory(uav_id)
        
        # 绘制无人机
        for idx, uav_id in enumerate(self.uav_ids):
            if uav_id in uav_states:
                state = uav_states[uav_id]
                position = np.array(state['position'])
                velocity = np.array(state['velocity'])
                
                self.draw_uav(position, uav_id, idx, velocity)
                self.update_trajectory(uav_id, position)
        
        # 绘制信息面板
        self.draw_info_panel(uav_states, current_source, formation_info, rl_step)
        
        # 更新显示
        pygame.display.flip()
        self.clock.tick(30)  # 30 FPS
    
    def handle_events(self) -> Dict[str, bool]:
        """处理事件"""
        events = {
            'quit': False,
            'pause': False,
            'reset': False
        }
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                events['quit'] = True
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    events['quit'] = True
                elif event.key == pygame.K_SPACE:
                    events['pause'] = True
                elif event.key == pygame.K_r:
                    events['reset'] = True
        
        return events
    
    def clear_trajectories(self):
        """清空轨迹"""
        for uav_id in self.uav_ids:
            self.trajectories[uav_id] = []
    
    def close(self):
        """关闭可视化"""
        pygame.quit()

