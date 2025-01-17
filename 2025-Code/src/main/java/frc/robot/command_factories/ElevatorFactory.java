package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.util.Subsystem;

public class ElevatorFactory {
    private static Elevator elevator = Subsystem.elevator;

    public static Command moveL1() {
        return elevator.moveElevatorCommand(ElevatorConstants.L1);
    }

    public static Command moveL2() {
        return elevator.moveElevatorCommand(ElevatorConstants.L2);
    }

    public static Command moveL3() {
        return elevator.moveElevatorCommand(ElevatorConstants.L3);
    }

    public static Command moveL4() {
        return elevator.moveElevatorCommand(ElevatorConstants.L4);
    }

    public static Command moveToIntake() {
        return elevator.moveElevatorCommand(ElevatorConstants.CORAL_INTAKE);
    }
}
