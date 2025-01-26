package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/**
 * 自动寻找并前往最近目标点的命令
 * 会从预设的目标点列表中找出最优的目标点，并自动规划路径前往该点
 * 评分标准同时考虑位置距离和方位差异
 */
public class AutoTargetCommand extends Command {
    private final Swerve m_swerve;
    private final List<Pose2d> m_targetPoses;
    private Command m_pathfindCommand = null;

    // 路径约束配置
    private static final PathConstraints constraints = new PathConstraints(
            3.0, // 最大速度 (m/s)
            3.0, // 最大加速度 (m/s²)
            360.0, // 最大角速度 (deg/s)
            720.0 // 最大角加速度 (deg/s²)
    );

    private static final double MAX_DISTANCE_SCORE = 50.0; // 最高距离分数
    private static final double MAX_ROTATION_SCORE = 50.0; // 最高旋转分数
    private static final double MAX_ACCEPTABLE_DISTANCE = 5.0; // 最大可接受距离（米）
    private static final double MAX_ACCEPTABLE_ROTATION = Math.PI; // 最大可接受旋转差异（弧度）

    /**
     * 构造函数
     * 
     * @param swerve      底盘子系统
     * @param targetPoses 目标位姿列表
     */
    public AutoTargetCommand(Swerve swerve, List<Pose2d> targetPoses) {
        m_swerve = swerve;
        m_targetPoses = targetPoses;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // 获取当前位姿
        Pose2d currentPose = m_swerve.getState().Pose;

        // 找出最优的目标点
        Pose2d bestTarget = findBestTarget(currentPose);

        // 创建寻路命令
        m_pathfindCommand = AutoBuilder.pathfindToPose(
                bestTarget,
                constraints,
                0.0); // 目标点速度为0

        // 开始执行寻路命令
        m_pathfindCommand.initialize();
    }

    @Override
    public void execute() {
        if (m_pathfindCommand != null) {
            m_pathfindCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (m_pathfindCommand != null) {
            m_pathfindCommand.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return m_pathfindCommand != null && m_pathfindCommand.isFinished();
    }

    /**
     * 找出最优的目标点
     * 评分标准同时考虑位置距离和方位差异
     * 
     * @param currentPose 当前位姿
     * @return 得分最高的目标点
     */
    private Pose2d findBestTarget(Pose2d currentPose) {
        Pose2d bestTarget = m_targetPoses.get(0);
        double bestScore = -1;
        Translation2d currentTranslation = currentPose.getTranslation();
        double currentRotation = currentPose.getRotation().getRadians();

        for (Pose2d targetPose : m_targetPoses) {
            // 快速距离检查
            double distance = currentTranslation.getDistance(targetPose.getTranslation());
            if (distance > MAX_ACCEPTABLE_DISTANCE) {
                continue; // 跳过距离过远的目标点
            }

            // 快速角度检查
            double rotationDiff = Math.abs(currentRotation - targetPose.getRotation().getRadians());
            if (rotationDiff > MAX_ACCEPTABLE_ROTATION) {
                continue; // 跳过角度差异过大的目标点
            }

            // 计算综合得分
            double score = calculateScore(distance, rotationDiff);
            if (score > bestScore) {
                bestScore = score;
                bestTarget = targetPose;
            }
        }

        return bestTarget;
    }

    /**
     * 计算目标点的得分
     * 使用线性插值代替指数计算，减少计算量
     * 
     * @param distance    距离（米）
     * @param rotationDiff 旋转差异（弧度）
     * @return 目标点的综合得分
     */
    private double calculateScore(double distance, double rotationDiff) {
        // 线性计算距离分数
        double distanceScore = MAX_DISTANCE_SCORE * (1 - Math.min(distance / MAX_ACCEPTABLE_DISTANCE, 1.0));

        // 线性计算旋转分数
        double rotationScore = MAX_ROTATION_SCORE * (1 - Math.min(rotationDiff / MAX_ACCEPTABLE_ROTATION, 1.0));

        return distanceScore + rotationScore;
    }
} 