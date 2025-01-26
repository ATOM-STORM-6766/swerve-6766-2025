// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

    /* 设置平台所需的转向驱动控制绑定 */
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry();

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final Swerve drivetrain = new Swerve(
            SwerveConstants.DrivetrainConstants,
            SwerveConstants.FrontLeft, SwerveConstants.FrontRight, SwerveConstants.BackLeft, SwerveConstants.BackRight);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        drivetrain.registerTelemetry(logger::telemeterize);

        autoChooser = AutoBuilder.buildAutoChooser("None");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureAuto();
        configureBindings();
    }

    private void configureBindings() {
        // 注意：根据WPILib约定，X轴定义为前进方向，
        // Y轴定义为向左方向
        drivetrain.setDefaultCommand(new TeleopSwerve(drivetrain, joystick));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // 按住back/start和X/Y键时运行SysId程序
        // 注意：每个程序在单次记录中应该只运行一次
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // 按下左缓冲键时重置场地坐标系朝向
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

    private void configureAuto() {
        NamedCommands.registerCommand("Temp", new PrintCommand("Temp"));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
