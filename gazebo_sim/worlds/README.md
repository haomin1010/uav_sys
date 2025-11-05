# Gazebo世界文件

## 可用的世界

### multi_uav.world（默认）
- **灰色地面** - 方便看清无人机
- 网格纹理 - 帮助判断距离
- 优化的光照设置
- 适合多无人机仿真

## 自定义世界

你可以根据需要修改 `multi_uav.world`：

### 修改地面颜色

编辑第54-59行的材质：

```xml
<material>
  <script>
    <uri>file://media/materials/scripts/gazebo.material</uri>
    <name>Gazebo/Grey</name>  <!-- 可选: Gazebo/White, Gazebo/DarkGrey -->
  </script>
</material>
```

可用颜色：
- `Gazebo/Grey` - 中灰色（默认）
- `Gazebo/DarkGrey` - 深灰色
- `Gazebo/White` - 白色
- `Gazebo/Black` - 黑色
- `Gazebo/Red` - 红色
- `Gazebo/Green` - 绿色
- `Gazebo/Blue` - 蓝色

### 添加障碍物

在 `</world>` 标签前添加：

```xml
<model name="box_obstacle">
  <pose>5 5 0.5 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>
      <material>
        <ambient>1 0 0 1</ambient>
        <diffuse>1 0 0 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### 修改摄像机视角

编辑第75-79行：

```xml
<camera name='user_camera'>
  <pose>10 -10 8 0 0.4 2.35</pose>  <!-- X Y Z Roll Pitch Yaw -->
  <view_controller>orbit</view_controller>
</camera>
```

常用视角：
- 俯视：`<pose>0 0 20 0 1.57 0</pose>`
- 侧视：`<pose>15 0 5 0 0.3 3.14</pose>`
- 斜视（默认）：`<pose>10 -10 8 0 0.4 2.35</pose>`

## 切换回空白世界

如果想使用原来的空白世界，编辑 `start_gazebo_sim.sh`：

```bash
# 方法1: 重命名自定义世界文件
mv multi_uav.world multi_uav.world.bak

# 方法2: 直接编辑脚本，注释掉自定义世界路径
```

## 创建新的世界文件

1. 复制 `multi_uav.world` 为新名字
2. 修改内容
3. 在启动脚本中指定新的world文件路径



