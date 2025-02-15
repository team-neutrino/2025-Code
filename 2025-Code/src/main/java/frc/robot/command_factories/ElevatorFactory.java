package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.util.Subsystem.elevator;

public class ElevatorFactory {
    private static int lastPov = -1;

    public static Command moveL1() {
        return elevator.moveElevatorCommand(L1);
    }

    public static Command moveL2() {
        return elevator.moveElevatorCommand(L2);
    }

    public static Command moveL3() {
        return elevator.moveElevatorCommand(L3);
    }

    public static Command moveL4() {
        return elevator.moveElevatorCommand(L4);
    }

    // public static Command modifyL4(CommandXboxController controller) {
    // return new RunCommand(() -> {
    // int pov = controller.getHID().getPOV();
    // if (lastPov == -1 && pov != -1) {
    // L4 += pov == 0 ? 3 : pov == 180 ? -3 : 0;
    // }
    // lastPov = pov;
    // });
    // }

    public static Command moveToIntake() {
        return elevator.moveElevatorCommand(CORAL_INTAKE);
    }

    public static Command movetoRemoveAlgaeL2() {
        return elevator.moveElevatorCommand(REMOVE_ALGAE_L2);
    }

    public static Command movetoRemoveAlgaeL3() {
        return elevator.moveElevatorCommand(REMOVE_ALGAE_L3);
    }

    public static Command dunkL3() {
        return elevator.moveElevatorCommand(L3 - 4);
    }

    public static Command dunkL2() {
        return elevator.moveElevatorCommand(L2 - 4);
    }
}
