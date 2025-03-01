package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

/**
 * 遥控驱动命令类
 * 实现了两种控制模式的自动切换：
 * 1. 场地中心控制（FieldCentric）：当有旋转输入时使用，可以自由控制机器人的旋转
 * 2. 场地中心朝向控制（FieldCentricFacingAngle）：当无旋转输入时使用，自动保持机器人的朝向
 */
public class SwerveWithHead extends Command {
    private final Swerve m_swerveDrivetrain;
    private final XboxController m_controller;
    // 最大速度和角速度

    // 场地中心控制请求：用于自由控制机器人的移动和旋转
    private final SwerveRequest.RobotCentricFacingAngle facingAngle = new SwerveRequest.RobotCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // 使用开环电压控制驱动电机
            .withSteerRequestType(SteerRequestType.MotionMagicExpo) // 使用MotionMagic控制转向电机
            .withHeadingPID(SwerveConstants.HeadingController.getP(), SwerveConstants.HeadingController.getI(),
                    SwerveConstants.HeadingController.getD());

    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // 使用开环电压控制驱动电机
            .withSteerRequestType(SteerRequestType.MotionMagicExpo); // 使用MotionMagic控制转向电机

    /**
     * 构造函数
     * 
     * @param swerveDrivetrain 底盘子系统
     * @param controller       手柄控制器
     */
    public SwerveWithHead(Swerve swerveDrivetrain, XboxController controller) {
        m_swerveDrivetrain = swerveDrivetrain;
        m_controller = controller;
        addRequirements(m_swerveDrivetrain);
    }

    @Override
    public void execute() {
        Rotation2d angle = Rotation2d.fromDegrees(m_controller.getPOV());

        m_swerveDrivetrain.setControl(
                robotCentric.withVelocityX(0.2 * angle.getCos())
                        .withVelocityY(-0.2 * angle.getSin()));

    }
}
