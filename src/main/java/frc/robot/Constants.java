package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;

import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.FieldTargets;

public class Constants {

    public static final Alliance alliance = Alliance.Red;

    public static final FieldTargets targets = new FieldTargets(Constants.alliance);

    // 由 Tuner X Swerve Project Generator 生成
    // https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
    public class SwerveConstants {

        public static final PhoenixPIDController HeadingController;
        static {
            HeadingController = new PhoenixPIDController(3, 0, 0.01);
            HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        }
        // 两组增益都需要根据您的具体机器人进行调整

        // 转向电机使用任何 SwerveModule.SteerRequestType 控制请求，
        // 输出类型由 SwerveModuleConstants.SteerMotorClosedLoopOutput 指定
        private static final Slot0Configs steerGains = new Slot0Configs()
                .withKP(5).withKI(0).withKD(0.01)
                .withKS(0.25727).withKV(1.6302).withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // 当使用闭环控制时，驱动电机使用由 SwerveModuleConstants.DriveMotorClosedLoopOutput 指定的控制输出类型
        private static final Slot0Configs driveGains = new Slot0Configs()
                .withKP(0.17494).withKI(0).withKD(0)
                .withKS(0.1237).withKV(0.12354).withKA(0.014822);

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
        private static final Current kSlipCurrent = Amps.of(70.0);

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
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(3.0);

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

        // Through Bore Encoder配置
        public static final double kEncoderOffset = 0.8649453425872388;

        // 驱动电机配置
        public static final TalonFXConfiguration driveConfigs = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(40)
                        .withStatorCurrentLimitEnable(true));

        // 位置电机PID增益
        public static final Slot0Configs positionGains = new Slot0Configs()
                .withKP(100) // 位置环增益
                .withKI(0.0)
                .withKD(0)
                .withKS(0) // 静摩擦补偿
                .withKV(0) // 速度前馈
                .withKA(0) // 加速度前馈
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKG(0.2998046875);

        // 位置电机配置
        public static final TalonFXConfiguration positionConfigs = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(20)
                        .withStatorCurrentLimitEnable(true))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicExpo_kV(0.012)
                        .withMotionMagicExpo_kA(0.8))
                .withTorqueCurrent(new TorqueCurrentConfigs()
                        .withPeakForwardTorqueCurrent(40) // 最大正向力矩电流
                        .withPeakReverseTorqueCurrent(-40)) // 最大反向力矩电流
                .withSlot0(positionGains)
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(kMaxAngle)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(-1.1))
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(kGearRatio));

    }

    public class ElevatorConstants {
        // 设备ID
        public static final int kElevatorDriverMotor1Id = 30; // 驱动电机1
        public static final int kElevatorDriverMotor2Id = 31; // 驱动电机2

        // 机构参数
        public static final double kGearRatio = 48.0 / 20 * 48 / 22 * 3 / 1; // 总减速比
        public static final double kMaxHeight = 6.75; // 最大高度（转数）

        // 电机配置
        public static final TalonFXConfiguration motorConfigs = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(35)
                        .withStatorCurrentLimitEnable(true))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicExpo_kV(0.006)//
                        .withMotionMagicExpo_kA(0.3))//
                .withSlot0(new Slot0Configs()
                        .withKP(23.5)// (6) // 位置环增益
                        .withKI(0)// (0)
                        .withKD(0)// (0)
                        .withKS(0.3)// (0.12) // 静摩擦补偿
                        .withKV(0)// (8) // 速度前馈
                        .withKA(0)// (0.5) // 加速度前馈
                        .withKG(0.1))// (0.18))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(kMaxHeight)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(0.01))
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(kGearRatio));
    }

    public class MouthConstants {

        public static InterpolatingDoubleTreeMap Mtom = new InterpolatingDoubleTreeMap();
        static {
            double[] temp = {
                    -22.457,
                    -22.376,
                    -22.291,
                    -22.201,
                    -22.108,
                    -22.01,
                    -21.909,
                    -21.803,
                    -21.694,
                    -21.581,
                    -21.464,
                    -21.344,
                    -21.22,
                    -21.092,
                    -20.961,
                    -20.826,
                    -20.687,
                    -20.546,
                    -20.401,
                    -20.252,
                    -20.1,
                    -19.945,
                    -19.787,
                    -19.625,
                    -19.46,
                    -19.292,
                    -19.121,
                    -18.947,
                    -18.77,
                    -18.59,
                    -18.407,
                    -18.221,
                    -18.032,
                    -17.84,
                    -17.646,
                    -17.448,
                    -17.248,
                    -17.045,
                    -16.84,
                    -16.631,
                    -16.42,
                    -16.207,
                    -15.99,
                    -15.772,
                    -15.55,
                    -15.327,
                    -15.1,
                    -14.872,
                    -14.641,
                    -14.407,
                    -14.171,
                    -13.933,
                    -13.693,
                    -13.45,
                    -13.205,
                    -12.958,
                    -12.708,
                    -12.457,
                    -12.203,
                    -11.947,
                    -11.689,
                    -11.43,
                    -11.168,
                    -10.904,
                    -10.638,
                    -10.37,
                    -10.1,
                    -9.828,
                    -9.555,
                    -9.28,
                    -9.002,
                    -8.724,
                    -8.443,
                    -8.161,
                    -7.877,
                    -7.591,
                    -7.304,
                    -7.015,
                    -6.724,
                    -6.432,
                    -6.139,
                    -5.844,
                    -5.547,
                    -5.249,
                    -4.95,
                    -4.649,
                    -4.347,
                    -4.044,
                    -3.74,
                    -3.434,
                    -3.127,
                    -2.819,
                    -2.51,
                    -2.2,
                    -1.888,
                    -1.576,
                    -1.263,
                    -0.948,
                    -0.633,
                    -0.317,
                    0,
                    0.318,
                    0.636,
                    0.956,
                    1.275,
                    1.596,
                    1.917,
                    2.239,
                    2.516,
                    2.884,
                    3.207,
                    3.531,
                    3.855,
                    4.179,
                    4.504,
                    4.829,
                    5.154,
                    5.479,
                    5.804,
                    6.129,
                    6.455,
                    6.78,
                    7.105,
                    7.43,
                    7.754,
                    8.078,
                    8.402,
                    8.726,
                    9.049,
                    9.372,
                    9.694,
                    10.015,
                    10.336,
                    10.656,
                    10.975,
                    11.293,
                    11.611,
                    11.927,
                    12.242,
                    12.556,
                    12.869,
                    13.18,
                    13.49,
                    13.799,
                    14.106,
                    14.411,
                    14.714,
                    15.016,
                    15.316,
                    15.614,
                    15.91,
                    16.204,
                    16.495,
                    16.784,
                    17.07,
                    17.354,
                    17.636,
                    17.914,
                    18.19,
                    18.463,
                    18.732,
                    18.998,
                    19.261,
                    19.521,
                    19.776,
                    20.028,
                    20.277,
                    20.521,
                    20.761,
                    20.996,
                    21.228,
                    21.454,
                    21.676,
                    21.893,
                    22.105,
                    22.312,
                    22.513,
                    22.709,
                    22.899,
                    23.083,
                    23.261,
                    23.433,
                    23.598,
                    23.757,
                    23.908,
                    24.053,
                    24.19,
                    24.32,
                    24.442,
                    24.556,
                    24.662,
                    24.76,
                    24.849,
                    24.929,
                    25,
                    25.061,
                    25.114,
                    25.156,
                    25.188,
                    25.21,
                    25.221
            };
            for (int i = 0; i < temp.length; i++) {
                Mtom.put(temp[i], ((double) i - 100) / 360);
            }
        }

        // 设备ID
        public static final int kMouthPositionMotorId = 40; // 位置电机
        public static final int kMouthDriverMotorId = 41; // 驱动电机
        public static final int kMouthHornMotorId = 42; // 角电机
        public static final int kEncoderChannel = 1; // Through Bore编码器通道

        // 机构参数
        public static final double kGearRatio = 20.0; // 总减速比
        public static final double kMaxAngle = 0.27; // 最大角度（0.5转 = 180度）

        // Through Bore Encoder配置
        public static final double kEncoderOffset = -0.01010699555929988 + 0.0102992424508561165;

        // 位置电机PID增益
        public static final Slot0Configs positionGains = new Slot0Configs()
                .withKP(100.0) // 位置环增益
                .withKI(0.0)
                .withKD(0.1)
                .withKS(0.5); // 静摩擦补偿

        // 位置电机配置
        public static final TalonFXConfiguration positionConfigs = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(30)
                        .withStatorCurrentLimitEnable(true))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicExpo_kV(1.2)
                        .withMotionMagicExpo_kA(2.4))
                .withTorqueCurrent(new TorqueCurrentConfigs()
                        .withPeakForwardTorqueCurrent(40) // 最大正向力矩电流
                        .withPeakReverseTorqueCurrent(-40)) // 最大反向力矩电流
                .withSlot0(positionGains)
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(kMaxAngle)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(-kMaxAngle))
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(kGearRatio));

        // 驱动电机
        public static final TalonFXConfiguration driveConfigs = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(80)
                        .withStatorCurrentLimitEnable(true))
                .withSlot0(new Slot0Configs()
                        .withKP(0.1) // 位置环增益
                        .withKI(0.0)
                        .withKD(0.001)
                        .withKS(0.1796875)
                        .withKV(0.11500000208616257))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(100)
                        .withMotionMagicAcceleration(1000));

        public static final TalonFXConfiguration hornConfigs = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(10)
                        .withStatorCurrentLimitEnable(true))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(1.48)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(0));
    }

    public class ClimberConstants {
        // 设备ID
        public static final int kElevatorDriverMotor1Id = 50; // 驱动电机1
        public static final int kElevatorDriverMotor2Id = 51; // 驱动电机2

        // 机构参数
        public static final double kGearRatio = 14.65; // 总减速比
        public static final double kMaxHeight = 0.25; // 最大高度（转数）

        // 电机配置
        public static final TalonFXConfiguration motorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(35)
                        .withStatorCurrentLimitEnable(true))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicExpo_kV(0.006)//
                        .withMotionMagicExpo_kA(0.3))//
                .withSlot0(new Slot0Configs()
                        .withKP(23.5)// (6) // 位置环增益
                        .withKI(0)// (0)
                        .withKD(0)// (0)
                        .withKS(0.3)// (0.12) // 静摩擦补偿
                        .withKV(0)// (8) // 速度前馈
                        .withKA(0)// (0.5) // 加速度前馈
                        .withKG(0.1)// (0.18))
                        .withGravityType(GravityTypeValue.Arm_Cosine))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(false)
                        .withForwardSoftLimitThreshold(kMaxHeight)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(0.01))
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(kGearRatio));
    }
}
