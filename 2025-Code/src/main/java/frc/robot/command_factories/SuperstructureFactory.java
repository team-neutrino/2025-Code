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
        Command clawCom = ClawFactory.runIntake();
        return elevatorCom.alongWith(armCom, clawCom).until(() -> claw.hasGamePiece());
    }

    public static Command outtake() {
        Command elevatorCom = ElevatorFactory.moveToIntake();
        Command armCom = ArmFactory.armToIntake();
        Command clawCom = ClawFactory.runOuttake();
        return elevatorCom.alongWith(armCom, clawCom);
    }

    public static Command descoreAlgaeL2() {
        Command elevatorCom = ElevatorFactory.movetoRemoveAlgaeL2();
        Command armCom = ArmFactory.armToDescoreL2();
        Command clawCom = ClawFactory.runIntake();
        return elevatorCom.alongWith(armCom, clawCom).until(() -> claw.hasGamePiece());
    }

    public static Command descoreAlgaeL3() {
        Command elevatorCom = ElevatorFactory.movetoRemoveAlgaeL3();
        Command armCom = ArmFactory.armToDescoreL3();
        Command clawCom = ClawFactory.runIntake();
        return elevatorCom.alongWith(armCom, clawCom).until(() -> claw.hasGamePiece());
    }

    public static Command scoreUnderhand(CommandXboxController controller) {
        Command elevatorCom = ElevatorFactory.moveL2();
        Command armScoreCom = ArmFactory.moveToUnderhand();
        Command clawDefaultCom = claw.clawDefaultCommand();
        Command clawScoreCom = ClawFactory.runOuttake();
        BooleanSupplier readyToScore = () -> (arm.armReady() && elevator.elevatorReady()
                && controller.getHID().getRightBumperButton());
        BooleanSupplier comEnd = () -> !claw.hasGamePiece();

        return ((elevatorCom.alongWith(armScoreCom, clawDefaultCom))
                .until(readyToScore)).andThen(clawScoreCom).until(comEnd);
    }

    public static Command scoreL1(CommandXboxController controller) {
        Command elevatorCom = ElevatorFactory.moveL1();
        Command armScoreCom = ArmFactory.moveToL1();
        Command clawDefaultCom = claw.clawDefaultCommand();
        Command clawScoreCom = ClawFactory.runOuttake();
        BooleanSupplier readyToScore = () -> (arm.armReady() && elevator.elevatorReady()
                && controller.getHID().getRightBumperButton());
        BooleanSupplier comEnd = () -> !claw.hasGamePiece();

        return ((elevatorCom.alongWith(armScoreCom, clawDefaultCom))
                .until(readyToScore)).andThen(clawScoreCom).until(comEnd);
    }

    public static Command scoreL2(CommandXboxController controller) {
        Command elevatorCom = ElevatorFactory.moveL2();
        Command armScoreCom = ArmFactory.moveToL2();
        Command clawDefaultCom = claw.clawDefaultCommand();
        Command clawScoreCom = ClawFactory.runOuttake();
        BooleanSupplier readyToScore = () -> (arm.armReady() && elevator.elevatorReady()
                && controller.getHID().getRightBumperButton());
        BooleanSupplier comEnd = () -> !claw.hasGamePiece();

        return ((elevatorCom.alongWith(armScoreCom, clawDefaultCom))
                .until(readyToScore)).andThen(clawScoreCom).until(comEnd);
    }

    public static Command scoreL3(CommandXboxController controller) {
        Command elevatorCom = ElevatorFactory.moveL3();
        Command armScoreCom = ArmFactory.moveToL3();
        Command clawDefaultCom = claw.clawDefaultCommand();
        Command clawScoreCom = ClawFactory.runOuttake();
        BooleanSupplier readyToScore = () -> (arm.armReady() && elevator.elevatorReady()
                && controller.getHID().getRightBumperButton());
        BooleanSupplier comEnd = () -> !claw.hasGamePiece();

        return ((elevatorCom.alongWith(armScoreCom, clawDefaultCom))
                .until(readyToScore)).andThen(clawScoreCom).until(comEnd);
    }

    public static Command scoreL4(CommandXboxController controller) {
        Command elevatorCom = ElevatorFactory.moveL4();
        Command armScoreCom = ArmFactory.moveToL4();
        Command clawDefaultCom = claw.clawDefaultCommand();
        Command clawScoreCom = ClawFactory.runOuttake();
        Command armEvacCom = ArmFactory.evacuateScoreL4();
        BooleanSupplier readyToScore = () -> (arm.armReady() && elevator.elevatorReady()
                && controller.getHID().getRightBumperButton());
        BooleanSupplier comEnd = () -> !claw.hasGamePiece();

        return ((elevatorCom.alongWith(armScoreCom, clawDefaultCom))
                .until(readyToScore)).andThen(
                        (armEvacCom.alongWith(clawScoreCom)).until(comEnd));
    }

    // AUTON COMMANDS

    public static Command scoreCoralL1AutonCommand() {
        Command elevatorCom = ElevatorFactory.moveL1();
        Command armScoreCom = ArmFactory.moveToL1();
        Command clawDefaultCom = claw.clawDefaultCommand();
        Command clawScoreCom = ClawFactory.runOuttake();
        BooleanSupplier readyToScore = () -> (arm.armReady() && elevator.elevatorReady());
        BooleanSupplier comEnd = () -> !claw.hasGamePiece();

        return ((elevatorCom.alongWith(armScoreCom, clawDefaultCom))
                .until(readyToScore)).andThen(clawScoreCom).until(comEnd);
    }

    public static Command scoreCoralL2AutonCommand() {
        Command elevatorCom = ElevatorFactory.moveL1();
        Command armScoreCom = ArmFactory.moveToL1();
        Command clawDefaultCom = claw.clawDefaultCommand();
        Command clawScoreCom = ClawFactory.runOuttake();
        BooleanSupplier readyToScore = () -> (arm.armReady() && elevator.elevatorReady());
        BooleanSupplier comEnd = () -> !claw.hasGamePiece();

        return ((elevatorCom.alongWith(armScoreCom, clawDefaultCom))
                .until(readyToScore)).andThen(clawScoreCom).until(comEnd);
    }

    public static Command scoreCoralL3AutonCommand() {
        Command elevatorCom = ElevatorFactory.moveL1();
        Command armScoreCom = ArmFactory.moveToL1();
        Command clawDefaultCom = claw.clawDefaultCommand();
        Command clawScoreCom = ClawFactory.runOuttake();
        BooleanSupplier readyToScore = () -> (arm.armReady() && elevator.elevatorReady());
        BooleanSupplier comEnd = () -> !claw.hasGamePiece();

        return ((elevatorCom.alongWith(armScoreCom, clawDefaultCom))
                .until(readyToScore)).andThen(clawScoreCom).until(comEnd);
    }

    public static Command scoreCoralL4AutonCommand() {
        Command elevatorCom = ElevatorFactory.moveL4();
        Command armScoreCom = ArmFactory.moveToL4();
        Command clawDefaultCom = claw.clawDefaultCommand();
        Command clawScoreCom = ClawFactory.runOuttake();
        Command armEvacCom = ArmFactory.evacuateScoreL4();
        BooleanSupplier readyToScore = () -> (arm.armReady() && elevator.elevatorReady());
        BooleanSupplier comEnd = () -> !claw.hasGamePiece();

        return ((elevatorCom.alongWith(armScoreCom, clawDefaultCom))
                .until(readyToScore)).andThen(
                        (armEvacCom.alongWith(clawScoreCom)).until(comEnd));
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
                ClawFactory.runIntake()).until(() -> claw.hasGamePiece());
    }
}
