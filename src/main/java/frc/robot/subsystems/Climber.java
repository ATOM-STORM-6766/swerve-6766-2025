package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

    // 硬件设备
    private final TalonFX m_motor1;
    private final TalonFX m_motor2;

    // 控制请求
    private final MotionMagicExpoVoltage m_positionRequest = new MotionMagicExpoVoltage(0).withEnableFOC(true);

    public Climber() {
        // 初始化电机
        m_motor1 = new TalonFX(ClimberConstants.kElevatorDriverMotor1Id);
        m_motor2 = new TalonFX(ClimberConstants.kElevatorDriverMotor2Id);

        // 配置主电机
        m_motor1.getConfigurator().apply(ClimberConstants.motorConfigs);

        // 配置从电机（跟随主电机）
        m_motor2.getConfigurator().apply(ClimberConstants.motorConfigs);
        m_motor2.setControl(new Follower(ClimberConstants.kElevatorDriverMotor1Id, true)); // true表示反向跟随

        m_motor1.setPosition(0);
    }

}
