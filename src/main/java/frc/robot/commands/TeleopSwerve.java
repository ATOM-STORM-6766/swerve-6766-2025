package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

/**
 * 遥控驱动命令类
 * 实现了两种控制模式的自动切换：
 * 1. 场地中心控制（FieldCentric）：当有旋转输入时使用，可以自由控制机器人的旋转
 * 2. 场地中心朝向控制（FieldCentricFacingAngle）：当无旋转输入时使用，自动保持机器人的朝向
 */
public class TeleopSwerve extends Command {
    private final Swerve m_swerveDrivetrain;
    private final CommandXboxController m_controller;

    // 最大速度和角速度
    private final double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // 场地中心控制请求：用于自由控制机器人的移动和旋转
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1) // 设置移动死区为最大速度的10%
            .withRotationalDeadband(MaxAngularRate * 0.1) // 设置旋转死区为最大角速度的10%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // 使用开环电压控制驱动电机
            .withSteerRequestType(SteerRequestType.MotionMagicExpo); // 使用MotionMagic控制转向电机

    // 场地中心朝向控制请求：用于保持机器人朝向
    private final SwerveRequest.FieldCentricFacingAngle facingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1) // 设置移动死区为最大速度的10%
            .withRotationalDeadband(MaxAngularRate * 0.1) // 设置旋转死区为最大角速度的10%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // 使用开环电压控制驱动电机
            .withSteerRequestType(SteerRequestType.MotionMagicExpo); // 使用MotionMagic控制转向电机

    // 用于角度保持的变量
    private Rotation2d lastAngle; // 记录最后的目标角度
    private final Timer correctionTimer = new Timer(); // 用于延迟更新目标角度的计时器
    private static final double CORRECTION_TIME_THRESHOLD = 0.2; // 角度更新延迟阈值（200ms）

    private boolean isRotating = true;

    /**
     * 构造函数
     * 
     * @param swerveDrivetrain 底盘子系统
     * @param controller       手柄控制器
     */
    public TeleopSwerve(Swerve swerveDrivetrain, CommandXboxController controller) {
        m_swerveDrivetrain = swerveDrivetrain;
        m_controller = controller;
        facingAngle.HeadingController.setPID(
                SwerveConstants.HeadingController.getP(),
                SwerveConstants.HeadingController.getI(),
                SwerveConstants.HeadingController.getD());
        addRequirements(m_swerveDrivetrain);
    }

    @Override
    public void initialize() {
        // 初始化时重置计时器和目标角度
        correctionTimer.stop();
        correctionTimer.reset();
        lastAngle = m_swerveDrivetrain.getState().Pose.getRotation();
    }

    @Override
    public void execute() {
        // 获取手柄输入并应用死区
        double xSpeed = -MathUtil.applyDeadband(m_controller.getLeftY(), 0.1);
        double ySpeed = -MathUtil.applyDeadband(m_controller.getLeftX(), 0.1);
        double rotationSpeed = -MathUtil.applyDeadband(m_controller.getRightX(), 0.1);

        if (Math.abs(rotationSpeed) > 0)
            isRotating = true;
        else if (isRotating)
        // correctionTimer.restart();

        // if (correctionTimer.hasElapsed(CORRECTION_TIME_THRESHOLD))
        {
            correctionTimer.stop();
            correctionTimer.reset();
            lastAngle = m_swerveDrivetrain.getState().Pose.getRotation();
            isRotating = false;
        }

        isRotating = Math.abs(rotationSpeed) > 0;

        // 根据是否有旋转输入选择控制模式
        m_swerveDrivetrain.setControl(
                isRotating ?
                // 有旋转输入：使用场地中心控制
                        fieldCentric.withVelocityX(xSpeed * MaxSpeed)
                                .withVelocityY(ySpeed * MaxSpeed)
                                .withRotationalRate(rotationSpeed * MaxAngularRate)
                        :
                        // 无旋转输入：使用场地中心朝向控制
                        facingAngle.withVelocityX(xSpeed * MaxSpeed)
                                .withVelocityY(ySpeed * MaxSpeed)
                                .withTargetDirection(lastAngle));
        ;
    }

    // @Override
    // public void end(boolean interrupted) {
    // // 命令结束时（无论是正常结束还是被打断）：
    // // 使用制动模式停止底盘
    // m_swerveDrivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    // }
}
