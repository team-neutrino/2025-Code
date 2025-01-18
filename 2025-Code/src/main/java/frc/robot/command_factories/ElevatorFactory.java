package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.ElevatorConstants.*;
import frc.robot.util.Subsystem;

public class ElevatorFactory {

    public static Command moveL1() {
        return Subsystem.elevator.moveElevatorCommand(L1);
    }

    public static Command moveL2() {
        return Subsystem.elevator.moveElevatorCommand(L2);
    }

    public static Command moveL3() {
        return Subsystem.elevator.moveElevatorCommand(L3);
    }

    public static Command moveL4() {
        return Subsystem.elevator.moveElevatorCommand(L4);
    }

    public static Command moveToIntake() {
        return Subsystem.elevator.moveElevatorCommand(CORAL_INTAKE);
    }

    public static Command movetoRemoveAlgaeL2() {
        return Subsystem.elevator.moveElevatorCommand(REMOVE_ALGAE_L2);
    }

    public static Command movetoRemoveAlgaeL3() {
        return Subsystem.elevator.moveElevatorCommand(REMOVE_ALGAE_L3);
    }
}
