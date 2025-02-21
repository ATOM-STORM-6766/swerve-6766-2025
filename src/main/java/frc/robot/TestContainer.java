package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Light;

public class TestContainer {
    public Light light = Light.getInstance();
    public Elevator elevator = new Elevator();
    public Climber climber = new Climber();
    public CommandXboxController joystick1 = new CommandXboxController(0);

    public TestContainer() {
        joystick1.a().onTrue(light.runOnce(() -> light.autoMoving()))
                .onFalse(light.runOnce(() -> light.autoMovingStop()));
        joystick1.b().onTrue(light.runOnce(() -> light.emptyMouth())).onFalse(light.runOnce(() -> light.fullMouth()));
        joystick1.x().onTrue(light.runOnce(() -> light.moveElevator()))
                .onFalse(light.runOnce(() -> light.moveElevatorStop()));
        joystick1.y().onTrue(light.runOnce(() -> light.setRainbow())).onFalse(light.runOnce(() -> light.interrupted()));
        joystick1.povDown().onTrue(elevator.stop());
        joystick1.povUp().onTrue(elevator.runOnce(() -> elevator.setPosition(2.73315)));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
