# FRC 6766 2025赛季机器人代码

## 项目概述
这是 FRC 6766 战队 2025 赛季的机器人代码仓库。我们的机器人使用 Java 语言开发，基于 WPILib 框架，并采用了 CTRE Phoenix 6 全向驱动系统。

### 主要特性
- 高性能全向驱动系统（Swerve Drive）
  - 使用 TalonFX 电机和 CANcoder 编码器
  - 支持场地坐标系控制
  - 自动角度保持功能
- 自动路径规划和跟踪
  - 基于 PathPlanner 的自动路径生成
  - 支持动态路径规划
- 智能目标追踪
  - 自动寻找最近目标点
  - 综合评分机制（距离+角度）

## 硬件配置
### 底盘系统
- 驱动电机：TalonFX × 4
- 转向电机：TalonFX × 4
- 编码器：CANcoder × 4
- IMU：Pigeon2 (ID: 10)

### 机构系统
- 取球机构（Intake）
  - 驱动电机：TalonFX (ID: 20)
  - 位置电机：TalonFX (ID: 21)
- 升降机构（Elevator）
  - 驱动电机1：TalonFX (ID: 30)
  - 驱动电机2：TalonFX (ID: 31)
- 投射机构（Mouth）
  - 驱动电机：TalonFX (ID: 40)
  - 位置电机1：TalonFX (ID: 41)
  - 位置电机2：TalonFX (ID: 42)

## 控制系统
### 手柄映射（Xbox Controller）
- 左摇杆：底盘平移控制
- 右摇杆：底盘旋转控制
- A键：底盘制动
- B键：轮子指向控制
- 左缓冲键：重置场地坐标系
- Back + X/Y：系统辨识（SysId）程序

### 自动驾驶功能
- 场地坐标系控制
- 自动角度保持
- 目标点自动追踪
- 路径规划与跟踪

## 代码结构
```
src/main/java/frc/robot/
├── commands/          # 指令类
│   ├── TeleopSwerve.java     # 遥控驾驶指令
│   └── AutoTargetCommand.java # 自动追踪指令
├── subsystems/        # 子系统类
│   └── Swerve.java           # 底盘子系统
├── Constants.java     # 常量定义
├── Robot.java         # 机器人主类
└── RobotContainer.java # 机器人配置类
```

## 开发指南
1. 所有常量配置都在 `Constants.java` 中定义
2. 新功能开发请创建独立的分支
3. 提交代码前请确保：
   - 代码已格式化
   - 通过基本功能测试
   - 添加必要的注释

## 调试工具
- Phoenix Tuner X：电机和传感器配置
- PathPlanner：自动路径规划
- SmartDashboard：实时数据显示
- SysId：系统辨识和PID调整

## 许可证
本项目基于 WPILib BSD 许可证开源。

## 团队信息
- 团队：FRC 6766
- 赛季：2025
