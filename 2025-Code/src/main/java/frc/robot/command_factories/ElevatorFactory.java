package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.util.Subsystem;

public class ElevatorFactory {
    public static Command moveL1() {
        Elevator elevator = Subsystem.elevator;
        return elevator.moveElevatorCommand(ElevatorConstants.L1);
    }

    public static Command moveL2() {
        Elevator elevator = Subsystem.elevator;
        return elevator.moveElevatorCommand(ElevatorConstants.L2);
    }

    public static Command moveL3() {
        Elevator elevator = Subsystem.elevator;
        return elevator.moveElevatorCommand(ElevatorConstants.L3);
    }

    public static Command moveL4() {
        Elevator elevator = Subsystem.elevator;
        return elevator.moveElevatorCommand(ElevatorConstants.L4);
    }
}
