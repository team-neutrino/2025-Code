package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.util.Subsystem.elevator;

import java.util.function.DoubleSupplier;

public class ElevatorFactory {

    public static Command moveToGiven(DoubleSupplier dubsLambda) {
        return elevator.moveElevatorCommandLambda(dubsLambda);
    }

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

    public static Command zeroElevator() {
        return elevator.moveElevatorCommand(0);
    }

    public static Command moveToIntake() {
        return elevator.moveElevatorCommand(CORAL_INTAKE);
    }

    public static Command moveToRemoveAlgaeL2() {
        return elevator.moveElevatorCommand(REMOVE_ALGAE_L2);
    }

    public static Command moveToRemoveAlgaeL3() {
        return elevator.moveElevatorCommand(REMOVE_ALGAE_L3);
    }

    public static Command moveToScoreBarge() {
        return elevator.moveElevatorCommand(SCORE_ALGAE_BARGE);
    }

    public static Command moveToScoreProcessor() {
        return elevator.moveElevatorCommand(SCORE_ALGAE_PROCESSOR);
    }

}
