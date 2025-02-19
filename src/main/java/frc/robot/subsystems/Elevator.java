package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemBase {
    // 硬件设备
    private final TalonFX m_motor1;
    private final TalonFX m_motor2;

    // 控制请求
    private final MotionMagicExpoVoltage m_positionRequest = new MotionMagicExpoVoltage(0).withEnableFOC(true);

    // 状态变量
    private double m_targetPosition = 0.0;

    /**
     * 构造函数
     */
    public Elevator() {
        // 初始化电机
        m_motor1 = new TalonFX(ElevatorConstants.kElevatorDriverMotor1Id);
        m_motor2 = new TalonFX(ElevatorConstants.kElevatorDriverMotor2Id);

        // 配置主电机
        m_motor1.getConfigurator().apply(ElevatorConstants.motorConfigs);

        // 配置从电机（跟随主电机）
        m_motor2.getConfigurator().apply(ElevatorConstants.motorConfigs);
        m_motor2.setControl(new Follower(ElevatorConstants.kElevatorDriverMotor1Id, false)); // true表示反向跟随

        m_motor1.setPosition(0);
    }

    @Override
    public void periodic() {
        // 添加遥测数据
        double currentPos = getPosition();
        SmartDashboard.putNumber("Elevator/Position", currentPos);
        SmartDashboard.putNumber("Elevator/TargetPosition", m_targetPosition);
        SmartDashboard.putNumber("Elevator/PositionError", m_targetPosition - currentPos);
        SmartDashboard.putBoolean("Elevator/AtPosition", isAtPosition());
        SmartDashboard.putNumber("Elevator/Motor1Current", m_motor1.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Motor2Current", m_motor2.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Velocity", m_motor1.getVelocity().getValueAsDouble());
    }

    /**
     * 设置电梯高度
     * 
     * @param position 目标位置（转数）
     */
    public void setPosition(double position) {
        // 限制在有效范围内
        m_motor1.setControl(m_positionRequest.withPosition(position));
    }

    public void setOpenPosition(double position) {
        // 限制在有效范围内
        m_motor1.setControl(new VoltageOut(12 * position));
    }

    /*
     * 8.8
     * 5.174
     * 2.913
     * 1.536133
     * 
     * in 1.656
     */

    public Command toL1() {
        return runOnce(() -> setPosition(1.18));
    }

    public Command toIn() {
        return runOnce(() -> setPosition(1.2));
    }

    public Command toL2() {
        return runOnce(() -> setPosition(2.167));
    }

    public Command toL3() {
        return runOnce(() -> setPosition(3.856));
    }

    public Command toL4() {
        return runOnce(() -> setPosition(6.725));
    }

    /**
     * 获取当前位置
     * 
     * @return 当前位置（转数）
     */
    public double getPosition() {
        return m_motor1.getPosition().getValueAsDouble();
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
     * 停止电梯
     */
    public Command stop() {
        return runOnce(() -> setPosition(0));
    }
}