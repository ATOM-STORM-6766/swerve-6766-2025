package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {
    // 硬件设备
    private final TalonFX m_driveMotor;
    private final TalonFX m_positionMotor;
    private final DutyCycleEncoder m_encoder;

    // 控制请求
    private final VoltageOut m_voltageOutRequest = new VoltageOut(0);
    private final MotionMagicExpoVoltage m_positionRequest = new MotionMagicExpoVoltage(0).withEnableFOC(true);

    // 状态变量
    private double m_targetPosition = 0.0;

    /**
     * 构造函数
     */
    public Intake() {
        // 初始化电机
        m_driveMotor = new TalonFX(IntakeConstants.kIntakeDriverMotorId);
        m_positionMotor = new TalonFX(IntakeConstants.kIntakePositionMotorId);

        // 配置驱动电机
        m_driveMotor.getConfigurator().apply(IntakeConstants.driveConfigs);

        // 配置位置电机
        m_positionMotor.getConfigurator().apply(IntakeConstants.positionConfigs);

        // 初始化编码器
        m_encoder = new DutyCycleEncoder(IntakeConstants.kEncoderChannel, 1, -0.236083984375);// -IntakeConstants.kEncoderOffset);//
                                                                                              // +0.1127504062166605
        m_encoder.setInverted(true);

        // 将电机位置设置为编码器位置
        syncMotorPosition();
    }

    /**
     * 同步电机位置到编码器位置
     */
    private void syncMotorPosition() {
        double encoderPos = m_encoder.get() > 0.5 ? m_encoder.get() - 1 : m_encoder.get();
        m_positionMotor.setPosition(encoderPos);
        m_targetPosition = encoderPos;
    }

    @Override
    public void periodic() {
        // 检查编码器和电机位置的差异
        double encoderPos = m_encoder.get();
        double motorPos = m_positionMotor.getPosition().getValueAsDouble();

        // 添加遥测数据
        SmartDashboard.putNumber("Intake/EncoderPosition", encoderPos);
        SmartDashboard.putNumber("Intake/MotorPosition", motorPos);
        SmartDashboard.putNumber("Intake/TargetPosition", m_targetPosition);
        SmartDashboard.putNumber("Intake/PositionError", m_targetPosition - encoderPos);
        SmartDashboard.putBoolean("Intake/AtPosition", isAtPosition());
        SmartDashboard.putNumber("Intake/StatorCurrent", m_positionMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Velocity", m_positionMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake/TorqueCurrent", m_positionMotor.getTorqueCurrent().getValueAsDouble());
    }

    /**
     * 设置取球电机速度
     * 
     * @param speed 速度（-1.0 到 1.0）
     */
    public void setIntakeSpeed(double speed) {
        m_driveMotor.setControl(m_voltageOutRequest.withOutput(12 * speed));
    }

    /**
     * 设置机构位置
     * 
     * @param position 目标位置（转数）
     */
    public void setPosition(double position) {
        // 限制在有效范围内 m_voltageOutRequest.withOutput(12 * position));//
        m_positionMotor.setControl(m_positionRequest.withPosition(position));
    }

    public Command toL1() {
        return runOnce(() -> setPosition(0.095));
    }

    public Command toGround() {
        return runOnce(() -> setPosition(-0.1));
    }

    public Command toVertical() {
        return runOnce(() -> setPosition(0.23));
    }

    public Command toAlga(){
        return runOnce(()->setPosition(0.03));
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
     * 检查是否到达目标位置
     * 
     * @return 是否到达
     */
    public boolean isAtPosition() {
        return Math.abs(getPosition() - m_targetPosition) < 0.01;
    }

    /**
     * 停止所有电机
     */
    public void stop() {
        m_driveMotor.setControl(m_voltageOutRequest.withOutput(0.0));
        m_positionMotor.setControl(m_positionRequest.withPosition(0));
    }
}