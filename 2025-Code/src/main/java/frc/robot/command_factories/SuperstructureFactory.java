package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static frc.robot.util.Subsystem.*;

import java.util.function.BooleanSupplier;

public class SuperstructureFactory {
    public static Command intakeCoral() {
        Command elevatorCom = ElevatorFactory.moveToIntake();
        Command armCom = ArmFactory.armToIntake();
        Command coralCom = CoralFactory.runIntake();
        return elevatorCom.alongWith(armCom, coralCom).until(() -> coral.debouncedHasCoral());
    }

    public static Command outtake() {
        Command elevatorCom = ElevatorFactory.moveToIntake();
        Command armCom = ArmFactory.armToIntake();
        Command coralCom = CoralFactory.runOuttake();
        return elevatorCom.alongWith(armCom, coralCom);
    }

    public static Command descoreAlgaeL2() {
        Command elevatorCom = ElevatorFactory.movetoRemoveAlgaeL2();
        Command armCom = ArmFactory.armToDescoreL2();
        return elevatorCom.alongWith(armCom);
    }

    public static Command descoreAlgaeL3() {
        Command elevatorCom = ElevatorFactory.movetoRemoveAlgaeL3();
        Command armCom = ArmFactory.armToDescoreL3();
        return elevatorCom.alongWith(armCom);
    }

    public static Command scoreUnderhand(CommandXboxController controller) {
        Command elevatorCom = ElevatorFactory.moveL2();
        Command armScoreCom = ArmFactory.moveToUnderhand();
        Command coralDefaultCom = coral.coralDefaultCommand();
        Command coralScoreCom = CoralFactory.runOuttake();
        BooleanSupplier readyToScore = () -> (arm.readyToScore() && elevator.readyToScore()
                && controller.getHID().getRightBumperButton());
        BooleanSupplier comEnd = () -> !coral.debouncedHasCoral();

        return ((elevatorCom.alongWith(armScoreCom, coralDefaultCom))
                .until(readyToScore)).andThen(coralScoreCom.until(comEnd));
    }

    public static Command scoreL1(CommandXboxController controller) {
        Command elevatorCom = ElevatorFactory.moveL1();
        Command armScoreCom = ArmFactory.moveToL1();
        Command coralDefaultCom = coral.coralDefaultCommand();
        Command coralScoreCom = CoralFactory.runSlowOuttake();
        BooleanSupplier readyToScore = () -> (arm.readyToScore() && elevator.readyToScore()
                && controller.getHID().getRightBumperButton());
        BooleanSupplier comEnd = () -> !coral.debouncedHasCoral();

        return ((elevatorCom.alongWith(armScoreCom, coralDefaultCom))
                .until(readyToScore)).andThen(coralScoreCom.until(comEnd));
    }

    public static Command scoreL2(CommandXboxController controller) {
        Command elevatorCom = ElevatorFactory.moveL2();
        Command armScoreCom = ArmFactory.moveToL2();
        Command coralDefaultCom = coral.coralDefaultCommand();
        Command coralScoreCom = CoralFactory.runOuttake();
        BooleanSupplier readyToScore = () -> (arm.readyToScore() && elevator.readyToScore()
                && controller.getHID().getRightBumperButton());
        BooleanSupplier comEnd = () -> !coral.debouncedHasCoral();

        return ((elevatorCom.alongWith(armScoreCom, coralDefaultCom))
                .until(readyToScore)).andThen(coralScoreCom.until(comEnd));
    }

    public static Command scoreL3(CommandXboxController controller) {
        Command elevatorCom = ElevatorFactory.moveL3();
        Command armScoreCom = ArmFactory.moveToL3();
        Command coralDefaultCom = coral.coralDefaultCommand();
        Command coralScoreCom = CoralFactory.runOuttake();
        BooleanSupplier readyToScore = () -> (arm.readyToScore() && elevator.readyToScore()
                && controller.getHID().getRightBumperButton());
        BooleanSupplier comEnd = () -> !coral.debouncedHasCoral();

        return ((elevatorCom.alongWith(armScoreCom, coralDefaultCom))
                .until(readyToScore)).andThen(
                        coralScoreCom.until(comEnd));
    }

    public static Command scoreL4(CommandXboxController controller) {
        Command elevatorCom = ElevatorFactory.moveL4();
        Command armScoreCom = ArmFactory.moveToL4();
        Command coralDefaultCom = coral.coralDefaultCommand();
        Command coralScoreCom = CoralFactory.runOuttake();
        Command armEvacCom = ArmFactory.evacuateScoreL4();
        BooleanSupplier readyToScore = () -> (arm.readyToScore() && elevator.readyToScore()
                && controller.getHID().getRightBumperButton());
        BooleanSupplier comEnd = () -> !coral.debouncedHasCoral();

        return ((elevatorCom.alongWith(armScoreCom, coralDefaultCom))
                .until(readyToScore)).andThen(
                        (armEvacCom.alongWith(coralScoreCom)).until(comEnd));
    }

    // AUTON COMMANDS

    public static Command scoreCoralL1AutonCommand() {
        Command elevatorCom = ElevatorFactory.moveL1();
        Command armScoreCom = ArmFactory.moveToL1();
        Command coralDefaultCom = coral.coralDefaultCommand();
        Command coralScoreCom = CoralFactory.runOuttake();
        BooleanSupplier readyToScore = () -> (arm.readyToScore() && elevator.readyToScore());
        BooleanSupplier comEnd = () -> !coral.debouncedHasCoral();

        return ((elevatorCom.alongWith(armScoreCom, coralDefaultCom))
                .until(readyToScore)).andThen(coralScoreCom).until(comEnd);
    }

    public static Command scoreCoralL2AutonCommand() {
        Command elevatorCom = ElevatorFactory.moveL2();
        Command armScoreCom = ArmFactory.moveToL2();
        Command coralDefaultCom = coral.coralDefaultCommand();
        Command coralScoreCom = CoralFactory.runOuttake();
        BooleanSupplier readyToScore = () -> (arm.readyToScore() && elevator.readyToScore());
        BooleanSupplier comEnd = () -> !coral.debouncedHasCoral();

        return ((elevatorCom.alongWith(armScoreCom, coralDefaultCom))
                .until(readyToScore)).andThen(coralScoreCom).until(comEnd);
    }

    public static Command scoreCoralL3AutonCommand() {
        Command elevatorCom = ElevatorFactory.moveL1();
        Command armScoreCom = ArmFactory.moveToL1();
        Command coralDefaultCom = coral.coralDefaultCommand();
        Command coralScoreCom = CoralFactory.runOuttake();
        BooleanSupplier readyToScore = () -> (arm.readyToScore() && elevator.readyToScore());
        BooleanSupplier comEnd = () -> !coral.debouncedHasCoral();

        return ((elevatorCom.alongWith(armScoreCom, coralDefaultCom))
                .until(readyToScore)).andThen(coralScoreCom).until(comEnd);
    }

    public static Command scoreCoralL4AutonCommand() {
        Command elevatorCom = ElevatorFactory.moveL4();
        Command armScoreCom = ArmFactory.moveToL4();
        Command coralDefaultCom = coral.coralDefaultCommand();
        Command coralScoreCom = CoralFactory.runOuttake();
        Command armEvacCom = ArmFactory.evacuateScoreL4();
        BooleanSupplier readyToScore = () -> (arm.readyToScore() && elevator.readyToScore());
        BooleanSupplier comEnd = () -> !coral.debouncedHasCoral();

        return ((elevatorCom.alongWith(armScoreCom, coralDefaultCom))
                .until(readyToScore)).andThen(
                        (armEvacCom.alongWith(coralScoreCom)).until(comEnd));
    }

    public static Command moveToScoreL4Command() {
        return new ParallelCommandGroup(ElevatorFactory.moveL4(), ArmFactory.moveToL4());
    }

    public static Command moveToScoreL3Command() {
        return new ParallelCommandGroup(ElevatorFactory.moveL3(), ArmFactory.moveToL3());
    }

    public static Command moveToScoreL2Command() {
        return new ParallelCommandGroup(ElevatorFactory.moveL2(), ArmFactory.moveToL2());
    }

    public static Command moveToScoreL1Command() {
        return new ParallelCommandGroup(ElevatorFactory.moveL1(), ArmFactory.moveToL1());
    }

    public static Command moveToIntake() {
        return new ParallelCommandGroup(ElevatorFactory.moveToIntake(), ArmFactory.armToIntake());
    }

    public static Command intakeCoralAutonCommand() {
        return new ParallelCommandGroup(ElevatorFactory.moveToIntake(), ArmFactory.armToIntake(),
                CoralFactory.runIntake()).until(() -> coral.debouncedHasCoral());
    }
}
