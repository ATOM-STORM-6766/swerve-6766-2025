package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;

import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.units.measure.*;

public class Constants {
    // 由 Tuner X Swerve Project Generator 生成
    // https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
    public class SwerveConstants {

        public static final PhoenixPIDController HeadingController = new PhoenixPIDController(0.1, 0.0, 0.001);
        // 两组增益都需要根据您的具体机器人进行调整

        // 转向电机使用任何 SwerveModule.SteerRequestType 控制请求，
        // 输出类型由 SwerveModuleConstants.SteerMotorClosedLoopOutput 指定
        private static final Slot0Configs steerGains = new Slot0Configs()
                .withKP(100).withKI(0).withKD(0.5)
                .withKS(0.1).withKV(2.66).withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // 当使用闭环控制时，驱动电机使用由 SwerveModuleConstants.DriveMotorClosedLoopOutput 指定的控制输出类型
        private static final Slot0Configs driveGains = new Slot0Configs()
                .withKP(0.1).withKI(0).withKD(0)
                .withKS(0).withKV(0.124);

        // 用于转向电机的闭环输出类型；
        // 这会影响转向电机的 PID/FF 增益
        private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // 用于驱动电机的闭环输出类型；
        // 这会影响驱动电机的 PID/FF 增益
        private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // 驱动电机使用的电机类型
        private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        // 转向电机使用的电机类型
        private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

        // 用于转向电机的远程传感器反馈类型；
        // 当没有 Pro 许可证时，FusedCANcoder/SyncCANcoder 自动降级为 RemoteCANcoder
        private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

        // 车轮开始打滑时的定子电流；
        // 这需要根据您的具体机器人进行调整
        private static final Current kSlipCurrent = Amps.of(120.0);

        // 驱动和转向电机以及方位编码器的初始配置；这些不能为空。
        // 某些配置将被覆盖；请查看 `with*InitialConfigs()` API 文档。
        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
                .withAudio(new AudioConfigs()
                        .withBeepOnBoot(false)
                        .withBeepOnConfig(false));
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
                .withAudio(new AudioConfigs()
                        .withBeepOnBoot(false)
                        .withBeepOnConfig(false))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        // Swerve 方位不需要太大的扭矩输出，所以我们可以设置一个相对较低的
                        // 定子电流限制，以帮助避免断电而不影响性能。
                        .withStatorCurrentLimit(Amps.of(60))
                        .withStatorCurrentLimitEnable(true));

        private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
        // Pigeon 2 的配置；如果不需要应用 Pigeon 2 配置，则保持为 null
        private static final Pigeon2Configuration pigeonConfigs = null;

        // 设备所在的 CAN 总线；
        // 所有 swerve 设备必须共享同一个 CAN 总线
        public static final CANBus kCANBus = new CANBus("CANivore", "./logs/example.hoot");

        // 在 12V 输出电压下的理论最大速度（m/s）；
        // 这需要根据您的具体机器人进行调整
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.21);

        // 方位每旋转 1 圈会导致驱动电机转动 kCoupleRatio 圈；
        // 这可能需要根据您的具体机器人进行调整
        private static final double kCoupleRatio = 50.0 / 14; // 值为L3的第一级减速比 14：50 的倒数

        private static final double kDriveGearRatio = 6.122448979591837;
        private static final double kSteerGearRatio = 21.428571428571427;
        private static final Distance kWheelRadius = Inches.of(2);

        private static final boolean kInvertLeftSide = true;
        private static final boolean kInvertRightSide = true;

        private static final int kPigeonId = 10;

        // 这些仅用于仿真
        private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
        private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
        // 克服摩擦所需的模拟电压
        private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
        private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(kCANBus.getName())
                .withPigeon2Id(kPigeonId)
                .withPigeon2Configs(pigeonConfigs);

        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withCouplingGearRatio(kCoupleRatio)
                .withWheelRadius(kWheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent)
                .withSpeedAt12Volts(kSpeedAt12Volts)
                .withDriveMotorType(kDriveMotorType)
                .withSteerMotorType(kSteerMotorType)
                .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage);

        // Front Left
        private static final int kFrontLeftDriveMotorId = 2;
        private static final int kFrontLeftSteerMotorId = 3;
        private static final int kFrontLeftEncoderId = 11;
        private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.015380859375);
        private static final boolean kFrontLeftSteerMotorInverted = true;
        private static final boolean kFrontLeftEncoderInverted = false;

        private static final Distance kFrontLeftXPos = Inches.of(11.1585);
        private static final Distance kFrontLeftYPos = Inches.of(11.1585);

        // Back Left
        private static final int kBackLeftDriveMotorId = 4;
        private static final int kBackLeftSteerMotorId = 5;
        private static final int kBackLeftEncoderId = 12;
        private static final Angle kBackLeftEncoderOffset = Rotations.of(0.315185546875);
        private static final boolean kBackLeftSteerMotorInverted = true;
        private static final boolean kBackLeftEncoderInverted = false;

        private static final Distance kBackLeftXPos = Inches.of(-11.1585);
        private static final Distance kBackLeftYPos = Inches.of(11.1585);

        // Back Right
        private static final int kBackRightDriveMotorId = 6;
        private static final int kBackRightSteerMotorId = 7;
        private static final int kBackRightEncoderId = 13;
        private static final Angle kBackRightEncoderOffset = Rotations.of(-0.158447265625);
        private static final boolean kBackRightSteerMotorInverted = true;
        private static final boolean kBackRightEncoderInverted = false;

        private static final Distance kBackRightXPos = Inches.of(-11.1585);
        private static final Distance kBackRightYPos = Inches.of(-11.1585);

        // Front Right
        private static final int kFrontRightDriveMotorId = 8;
        private static final int kFrontRightSteerMotorId = 9;
        private static final int kFrontRightEncoderId = 14;
        private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.109130859375);
        private static final boolean kFrontRightSteerMotorInverted = true;
        private static final boolean kFrontRightEncoderInverted = false;

        private static final Distance kFrontRightXPos = Inches.of(11.1585);
        private static final Distance kFrontRightYPos = Inches.of(-11.1585);

        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft = ConstantCreator
                .createModuleConstants(
                        kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId,
                        kFrontLeftEncoderOffset,
                        kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide,
                        kFrontLeftSteerMotorInverted,
                        kFrontLeftEncoderInverted);
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight = ConstantCreator
                .createModuleConstants(
                        kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
                        kFrontRightEncoderOffset,
                        kFrontRightXPos, kFrontRightYPos, kInvertRightSide,
                        kFrontRightSteerMotorInverted,
                        kFrontRightEncoderInverted);
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft = ConstantCreator
                .createModuleConstants(
                        kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId,
                        kBackLeftEncoderOffset,
                        kBackLeftXPos, kBackLeftYPos, kInvertLeftSide,
                        kBackLeftSteerMotorInverted,
                        kBackLeftEncoderInverted);
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight = ConstantCreator
                .createModuleConstants(
                        kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId,
                        kBackRightEncoderOffset,
                        kBackRightXPos, kBackRightYPos, kInvertRightSide,
                        kBackRightSteerMotorInverted,
                        kBackRightEncoderInverted);
    }

    public class IntakeConstants {
        // 设备ID
        public static final int kIntakeDriverMotorId = 20;
        public static final int kIntakePositionMotorId = 21;
        public static final int kEncoderChannel = 0;

        // 机构参数
        public static final double kGearRatio = 140.0; // 总减速比
        public static final double kMaxAngle = 0.25; // 最大角度（0.25转 = 90度）

        // 驱动电机配置
        public static final TalonFXConfiguration driveConfigs = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(40)
                        .withStatorCurrentLimitEnable(true));

        // 位置电机配置
        public static final TalonFXConfiguration positionConfigs = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(30)
                        .withStatorCurrentLimitEnable(true))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(0.15)
                        .withMotionMagicAcceleration(0.3))
                .withSlot0(new Slot0Configs()
                        .withKP(1.5)
                        .withKI(0.0)
                        .withKD(0.5)
                        .withKS(0.25))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(kMaxAngle)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(0.0))
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(kGearRatio));

        // Through Bore Encoder配置
        public static final double kEncoderMinDutyCycle = 1.0 / 4096.0;
        public static final double kEncoderMaxDutyCycle = 4095.0 / 4096.0;
    }

    public class ElevatorConstants {
        public static final int kElevatorDriverMotor1Id = 30; // 驱动电机1
        public static final int kElevatorDriverMotor2Id = 31; // 驱动电机2
    }

    public class MouthConstants {
        public static final int kMouthDriverMotorId = 40; // 位置电机
        public static final int kMouthPositionMotorId1 = 41; // 驱动电机
        public static final int kMouthPositionMotorId2 = 42; // 驱动电机
    }
}
