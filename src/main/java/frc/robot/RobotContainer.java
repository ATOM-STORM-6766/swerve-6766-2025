// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.*;
import static frc.robot.subsystems.vision.VisionConstants.*;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Mouth;

public class RobotContainer {

    /* 设置平台所需的转向驱动控制绑定 */
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry();

    private final CommandXboxController joystick1 = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    // 添加子系统

    public final Swerve drivetrain = new Swerve(
            SwerveConstants.DrivetrainConstants,
            SwerveConstants.FrontLeft, SwerveConstants.FrontRight, SwerveConstants.BackLeft,
            SwerveConstants.BackRight);
    public final Intake intake = new Intake();
    public final Elevator elevator = new Elevator();
    public final Mouth mouth = new Mouth();
    public final Vision vision;

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        drivetrain.registerTelemetry(logger::telemeterize);

        if (RobotBase.isReal()) {
            vision = new Vision(
                    drivetrain::addVisionMeasurement,
                    new VisionIOPhotonVision(camera0Name, robotToCamera0));
            // new VisionIOPhotonVision(camera1Name, robotToCamera1));
        } else {
            vision = new Vision(
                    drivetrain::addVisionMeasurement,
                    new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, () -> drivetrain.getState().Pose));
            // new VisionIOPhotonVision(camera1Name, robotToCamera1));
        }

        autoChooser = AutoBuilder.buildAutoChooser("None");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureAuto();
        configureBindings();
    }

    private void configureBindings() {
        // 注意：根据WPILib约定，X轴定义为前进方向，
        // Y轴定义为向左方向
        drivetrain.setDefaultCommand(new TeleopSwerve(drivetrain, joystick1));

        joystick1.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick1.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(
                        new Rotation2d(-joystick1.getLeftY(), -joystick1.getLeftX()))));

        // 按住back/start和X/Y键时运行SysId程序
        // 注意：每个程序在单次记录中应该只运行一次
        joystick1.back().and(joystick1.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick1.back().and(joystick1.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick1.start().and(joystick1.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick1.start().and(joystick1.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // 按下左缓冲键时重置场地坐标系朝向
        joystick1.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Intake控制
        // 右扳机控制进料速度（0-100%）
        joystick2.rightTrigger().whileTrue(
                intake.run(() -> intake.setIntakeSpeed(joystick2.getRightTriggerAxis() *
                        0.6)))
                .whileFalse(
                        intake.runOnce(() -> intake.setIntakeSpeed(0)));

        joystick2.leftTrigger().whileTrue(
                intake.run(() -> intake.setIntakeSpeed(-joystick2.getLeftTriggerAxis() *
                        0.3)))
                .whileFalse(
                        intake.runOnce(() -> intake.setIntakeSpeed(0)));

        // 左扳机控制进料机构位置（0-90度）
        joystick2.axisLessThan(1, -0.9).onTrue(intake.toVertical());
        joystick2.axisGreaterThan(1, 0.9).onTrue(intake.toGround());
        joystick2.axisLessThan(0, -0.9).onTrue(intake.toAlga());
        joystick2.axisGreaterThan(0, 0.9).onTrue(intake.toL1());

        // 电梯控制
        joystick2.povUp().onTrue(elevator.toL1());
        joystick2.x().onTrue(elevator.toIn());
        joystick2.povRight().onTrue(elevator.toL2());
        joystick2.povDown().onTrue(elevator.toL3());
        joystick2.povLeft().onTrue(elevator.toL4());
        joystick2.a().onTrue(elevator.stop());

        // 嘴控制
        joystick2.y().onTrue(mouth.setSpeed(30))// Commands.print("T"))//
                .onFalse(mouth.setSpeed(0)); // 吐 //Commands.print("F"));//

        mouth.m_limitSwitchTrigger.toggleOnFalse(mouth.stopDrive());

        joystick2.leftBumper().whileTrue(mouth.run(() -> mouth.setPosition(-0.25)))
                .onFalse(mouth.run(() -> mouth.stop())); // 收起

        joystick2.rightBumper().whileTrue(mouth.run(() -> mouth.setPosition(0.2)))
                .onFalse(mouth.run(() -> mouth.stop()));// 展开
    }

    private void configureAuto() {
        NamedCommands.registerCommand("Temp", new PrintCommand("Temp"));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}