package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.*;
import frc.robot.util.Subsystem;

public class SuperstructureFactory {
    private static Elevator elevator = Subsystem.elevator;
    private static Claw claw = Subsystem.claw;
    // private static Arm arm = Subsystem.arm;

    public static Command intakeCoral() {
        Command ret = elevator.moveElevatorCommand(ElevatorConstants.CORAL_INTAKE).alongWith(null);
        return null;
    }
}
