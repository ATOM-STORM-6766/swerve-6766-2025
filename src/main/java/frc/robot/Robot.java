// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final Timer m_disableTimer = new Timer();

    // private final TestContainer m_robotContainer;
    private final RobotContainer m_robotContainer;

    public Robot() {

        CameraServer.startAutomaticCapture();
        // m_robotContainer = new TestContainer();
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        PathfindingCommand.warmupCommand().schedule();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        m_disableTimer.start();
    }

    @Override
    public void disabledPeriodic() {
        // if (m_disableTimer.hasElapsed(5)) {
        // m_robotContainer.drivetrain.setNeutralMode(NeutralModeValue.Coast);
        // m_disableTimer.stop();
        // m_disableTimer.reset();
        // }
    }

    @Override
    public void disabledExit() {
        // m_robotContainer.drivetrain.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
