package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.MouthConstants;
import frc.robot.subsystems.vision.FieldTargets;
import frc.robot.util.FourBarLinkage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DigitalInput;

public class Mouth extends SubsystemBase {

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable driveStateTable = inst.getTable("Vision");
    private final StructPublisher<Translation2d> drivePose = driveStateTable
            .getStructTopic("Mouth", Translation2d.struct)
            .publish();

    // 硬件设备
    private final TalonFX m_positionMotor;
    private final TalonFX m_driveMotor;
    private final TalonFX m_hornMotor;
    private final DutyCycleEncoder m_encoder;
    private final DigitalInput m_limitSwitch = new DigitalInput(2);

    // 控制请求
    private final MotionMagicVelocityVoltage m_voltageOutRequest = new MotionMagicVelocityVoltage(0)
            .withEnableFOC(true);
    private final MotionMagicExpoVoltage m_positionRequest = new MotionMagicExpoVoltage(0).withEnableFOC(true);
    public final Trigger m_limitSwitchTrigger = new Trigger(m_limitSwitch::get);

    MedianFilter filter = new MedianFilter(5);

    // 状态变量
    private FieldTargets target;
    private boolean m_isInitialized = false;
    private int m_stableReadingsCount = 0;
    private double m_lastEncoderReading = 0.0;
    private FourBarLinkage linkage = new FourBarLinkage();

    /**
     * 构造函数
     */
    public Mouth(FieldTargets target) {
        this.target = target;
        // 初始化电机
        m_positionMotor = new TalonFX(MouthConstants.kMouthPositionMotorId);
        m_driveMotor = new TalonFX(MouthConstants.kMouthDriverMotorId);
        m_hornMotor = new TalonFX(MouthConstants.kMouthHornMotorId);

        // 配置位置电机
        m_positionMotor.getConfigurator().apply(MouthConstants.positionConfigs);
        m_driveMotor.getConfigurator().apply(MouthConstants.driveConfigs);
        m_hornMotor.getConfigurator().apply(MouthConstants.hornConfigs);

        // 设置取球电机为制动模式
        m_driveMotor.setNeutralMode(NeutralModeValue.Brake);

        // 初始化编码器
        m_encoder = new DutyCycleEncoder(MouthConstants.kEncoderChannel, 1, MouthConstants.kEncoderOffset);

        // 将电机位置设置为编码器位置
        syncMotorPosition();
        m_hornMotor.setPosition(1.48);
        m_hornMotor.set(-0.015);
    }

    /**
     * 同步电机位置到编码器位置
     */
    private void syncMotorPosition() {
        // 检查编码器初始化状态
        if (!m_isInitialized) {
            if (m_encoder.isConnected()) {
                double currentReading = m_encoder.get();
                // 检查读数是否稳定1
                if (Math.abs(currentReading - m_lastEncoderReading) < 0.005) {
                    m_stableReadingsCount++;
                    if (m_stableReadingsCount >= 10) { // 需要连续10次稳定读数
                        m_isInitialized = true;
                        if (currentReading > 0.5)
                            currentReading += -1;
                        m_positionMotor.setPosition(currentReading);
                        m_positionMotor.setControl(m_positionRequest.withPosition(0));
                    }
                } else {
                    m_stableReadingsCount = 0; // 如果读数不稳定，重置计数
                }
                m_lastEncoderReading = currentReading;
            }
        }
    }

    @Override
    public void periodic() {
        // 检查编码器和电机位置的差异
        double encoderPos = m_encoder.get();
        if (encoderPos > 0.5)
            encoderPos += -1;
        double motorPos = m_positionMotor.getPosition().getValueAsDouble();

        if (!m_isInitialized)
            syncMotorPosition();

        // 添加遥测数据
        SmartDashboard.putNumber("Mouth/EncoderPosition", encoderPos);
        SmartDashboard.putNumber("Mouth/MotorPosition", motorPos);
        SmartDashboard.putBoolean("Mouth/IsInitialized", m_isInitialized);
        SmartDashboard.putNumber("Mouth/StableReadingsCount", m_stableReadingsCount);
        SmartDashboard.putNumber("Mouth/StatorCurrent", m_positionMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Mouth/TorqueCurrent", m_positionMotor.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Mouth/LimitSwitch", !m_limitSwitch.get());
    }

    /**
     * 设置取球电机速度
     * 
     * @param speed 速度（-1.0 到 1.0）
     */
    public Command setSpeed(double speed) {
        return run(() -> m_driveMotor.setControl(m_voltageOutRequest.withVelocity(speed)));
    }

    /**
     * 设置机构位置
     * 
     * @param position 目标位置（转数）
     */
    public void setPosition(double position) {
        // 如果未初始化完成，拒绝控制请求

        if (!m_isInitialized) {
            System.out.println("Mouth not initialized");
            return;
        }

        // position = filter.calculate(position);
        // 限制在有效范围内
        m_positionMotor.setControl(m_positionRequest.withPosition(position));
    }

    public void setLeft(Supplier<Pose2d> curr) {
        Pose2d t = target.reef.pose.toPose2d().transformBy(FieldTargets.leftPip).relativeTo(curr.get());
        drivePose.set(t.getTranslation());
        Rotation2d position = t.getTranslation().getAngle();
        double x = 143.28 - linkage.calculate(position.getRadians() + Math.PI / 2) * 180 / Math.PI;
        setPosition(x / 360);
    }

    public void setRight(Supplier<Pose2d> curr) {
        Rotation2d position = target.reef.pose.toPose2d().transformBy(FieldTargets.rightPip).relativeTo(curr.get())
                .getTranslation()
                .getAngle();
        double x = 143.28 - linkage.calculate(position.getRadians() + Math.PI / 2) * 180 / Math.PI;
        setPosition(x / 360);
    }

    /**
     * 获取当前位置
     * 
     * @return 当前位置（转数）
     */
    public double getPosition() {
        return m_encoder.get();
    }

    /**
     * 急停命令
     */
    public Command stopDrive() {
        return runOnce(() -> {
            double currentSpeed = m_driveMotor.getVelocity().getValueAsDouble();
            m_driveMotor.setControl(m_voltageOutRequest.withVelocity(-currentSpeed * 0.65));
            System.out.println(-currentSpeed * 0.65);
        })
                .andThen(Commands.waitSeconds(0.05))
                .andThen(setSpeed(0)); // 最后设置为0
    }

    /**
     * 回中所有电机
     */
    public void stop() {
        setPosition(0);
    }
}