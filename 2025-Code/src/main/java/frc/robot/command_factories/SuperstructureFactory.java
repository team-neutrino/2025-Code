package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveToPoint.Mode;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.*;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.Constants;
import frc.robot.util.Subsystem;

import static frc.robot.util.Subsystem.*;

import java.util.function.BooleanSupplier;

public class SuperstructureFactory {

    public static boolean antiDriveTeamCondition() {
        if (Subsystem.swerve.getCurrentCommand() == null) {
            return false;
        }
        return Subsystem.swerve
                .getCurrentCommand().getName().equals(Constants.DriveToPoint.DRIVE_TO_POINT_BASIC)
                        ? Subsystem.swerve.isAtPoint()
                                && (Subsystem.limelight.getTvReef1() || Subsystem.limelight.getTvReef2())
                        : true;
    }

    public static Command dynamicCoralIntake() {
        Command ret = new RunCommand(() -> {
            Command swerveCom = swerve.getCurrentCommand();
            DriveToPointCommand casted = (swerveCom != null
                    && swerveCom.getName().equals(DriveToPoint.DRIVE_TO_POINT_BASIC))
                            ? (DriveToPointCommand) swerveCom
                            : null;
            // if we're running driveToPoint and the distance from target is below a certain
            // threshold, change the arm and elevator position based on that distance
            if (casted != null
                    && Math.abs(casted.distStraightPlayerStation()) <= DriveToPoint.DYNAMIC_INTAKE_THRESHOLD) {
                arm.setTarget(ArmConstants.CORAL_STATION_POSITION);
                // right now the P is just the conversion factor for meters to inches,
                // effectively making every inch we're off from target position one extra inch
                // in elevator height.
                elevator.setTargetHeight(ElevatorConstants.CORAL_INTAKE
                        - (casted.distStraightPlayerStation() * ElevatorConstants.DYNAMIC_ADJUST_P));
            } else {
                arm.setTarget(ArmConstants.CORAL_STATION_POSITION);
                elevator.setTargetHeight(ElevatorConstants.CORAL_INTAKE);
            }
        }, arm, elevator);
        return ret.alongWith(CoralFactory.runIntake()).until(() -> coral.debouncedHasCoral());
    }

    public static Command intakeCoral() {
        Command elevatorCom = ElevatorFactory.moveToIntake();
        Command armCom = ArmFactory.armToIntake();
        Command coralCom = CoralFactory.runIntake();
        return elevatorCom.alongWith(armCom, coralCom).until(() -> coral.hasCoral());
    }

    public static Command outtake() {
        Command elevatorCom = ElevatorFactory.moveToIntake();
        Command armCom = ArmFactory.armToIntake();
        Command coralCom = CoralFactory.runOuttake();
        return elevatorCom.alongWith(armCom, coralCom);
    }

    public static Command descoreAlgaeL2() {
        Command elevatorCom = ElevatorFactory.moveToRemoveAlgaeL2();
        Command armCom = ArmFactory.armToDescoreL2();
        Command algaeCom = AlgaeFactory.runIntake();
        return elevatorCom.alongWith(armCom, algaeCom);
    }

    public static Command descoreAlgaeL3() {
        Command elevatorCom = ElevatorFactory.moveToRemoveAlgaeL3();
        Command armCom = ArmFactory.armToDescoreL3();
        Command algaeCom = AlgaeFactory.runIntake();
        return elevatorCom.alongWith(armCom, algaeCom);
    }

    public static Command scoreBargeCommand(CommandXboxController controller) {
        Command elevatorCom = ElevatorFactory.moveToScoreBarge();
        Command armScoreCom = ArmFactory.armToScoreBarge();
        Command algaeScoreCom = AlgaeFactory.runOuttake();
        BooleanSupplier readyToScore = () -> (arm.readyToScore() && elevator.readyToScore()
                && controller.getHID().getRightBumperButton());
        BooleanSupplier comEnd = () -> !algae.debouncedHasAlgae();

        return ((elevatorCom.alongWith(armScoreCom))
                .until(readyToScore)).andThen(algaeScoreCom.until(comEnd));
    }

    public static Command scoreProcessorCommand(CommandXboxController controller) {
        Command elevatorCom = ElevatorFactory.moveToScoreProcessor();
        Command armScoreCom = ArmFactory.armToScoreProcessor();
        Command algaeDefaultCom = algae.algaeDefaultCommand();
        Command algaeScoreCom = AlgaeFactory.runOuttake();
        BooleanSupplier readyToScore = () -> (arm.readyToScore() && elevator.readyToScore()
                && controller.getHID().getRightBumperButton());
        BooleanSupplier comEnd = () -> !algae.debouncedHasAlgae();

        return ((elevatorCom.alongWith(armScoreCom, algaeDefaultCom))
                .until(readyToScore)).andThen(algaeScoreCom.until(comEnd));
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

        return ((elevatorCom.alongWith(coralDefaultCom))
                .until(readyToScore)).andThen(armScoreCom).andThen(coralScoreCom.until(comEnd));
    }

    public static Command scoreL2(CommandXboxController controller) {
        Command elevatorCom = ElevatorFactory.moveL2();
        Command armScoreCom = ArmFactory.moveToL2();
        Command coralDefaultCom = coral.coralDefaultCommand();
        Command coralScoreCom = CoralFactory.runOuttake();
        BooleanSupplier readyToScore = (() -> arm.readyToScore() && elevator.readyToScore()
                && controller.getHID().getRightBumperButton() && antiDriveTeamCondition());
        BooleanSupplier comEnd = () -> !coral.debouncedHasCoral();

        return ((elevatorCom.alongWith(coralDefaultCom))
                .until(() -> elevator.readyToScore())).andThen(armScoreCom).until(readyToScore)
                .andThen(coralScoreCom.until(comEnd));
    }

    public static Command scoreL3(CommandXboxController controller) {
        Command elevatorCom = ElevatorFactory.moveL3();
        Command armScoreCom = ArmFactory.moveToL3();
        Command coralDefaultCom = coral.coralDefaultCommand();
        Command coralScoreCom = CoralFactory.runOuttake();
        BooleanSupplier readyToScore = () -> (arm.readyToScore() && elevator.readyToScore()
                && controller.getHID().getRightBumperButton() && antiDriveTeamCondition());
        BooleanSupplier comEnd = () -> !coral.debouncedHasCoral();

        return ((elevatorCom.alongWith(coralDefaultCom))
                .until(() -> elevator.readyToScore())).andThen(armScoreCom).until(readyToScore)
                .andThen(coralScoreCom.until(comEnd));
    }

    public static Command scoreL4(CommandXboxController controller) {
        Command elevatorCom = ElevatorFactory.moveL4();
        Command armScoreCom = ArmFactory.moveToL4();
        Command coralDefaultCom = coral.coralDefaultCommand();
        Command coralScoreCom = CoralFactory.runOuttake();
        Command armEvacCom = ArmFactory.evacuateScoreL4();
        Command evacWait = new WaitCommand(0.2);
        BooleanSupplier readyToScore = () -> (arm.readyToScore() && elevator.readyToScore()
                && controller.getHID().getRightBumperButton() && antiDriveTeamCondition());
        BooleanSupplier comEnd = () -> !coral.debouncedHasCoral();

        return ((elevatorCom.alongWith(coralDefaultCom))
                .until(() -> elevator.readyToScore())).andThen(armScoreCom).until(readyToScore)
                .andThen((coralScoreCom.alongWith(evacWait.andThen(armEvacCom))).until(comEnd));
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

    public static Command scoreNetAutomated(CommandXboxController controller) {
        Command drive = new DriveToPointCommand(controller, Mode.NET);
        Command elevatorCom = ElevatorFactory.moveToScoreBarge();
        Command elevatorDefaultCom = elevator.elevatorDefaultCommand();
        Command armCom = ArmFactory.armToScoreBarge();
        Command algaeDefault = algae.algaeDefaultCommand();
        Command algaeScoreCom = AlgaeFactory.runOuttake();
        BooleanSupplier isAtPoint = () -> swerve.isAtPointDebounced();
        BooleanSupplier readyToScore = () -> (elevator.readyToScore() && arm.readyToScore()
                && swerve.isAtPointDebounced());
        BooleanSupplier comEnd = () -> !algae.debouncedHasAlgae();

        return ((drive.alongWith(elevatorDefaultCom.until(isAtPoint).andThen(elevatorCom), armCom,
                algaeDefault.until(readyToScore).andThen(algaeScoreCom))).until(comEnd));
    }

    public static Command moveToScoreL4Command() {
        return new ParallelCommandGroup(ElevatorFactory.moveL4(), ArmFactory.moveToL4());
    }

    public static Command spicyMoveToScoreL4Command() {
        return new ParallelCommandGroup(ElevatorFactory.moveL4(), ArmFactory.moveToL4())
                .onlyWhile(() -> DriveToPointController.getManhattanDistance() < .2);
    }

    public static Command moveToScoreL3Command() {
        return new ParallelCommandGroup(ElevatorFactory.moveL3(), ArmFactory.moveToL3());
    }

    public static Command spicyMoveToScoreL3Command() {
        return new ParallelCommandGroup(ElevatorFactory.moveL3(), ArmFactory.moveToL3())
                .onlyWhile(() -> DriveToPointController.getManhattanDistance() < .2);
    }

    public static Command moveToScoreL2Command() {
        return new ParallelCommandGroup(ElevatorFactory.moveL2(), ArmFactory.moveToL2());
    }

    public static Command spicyMoveToScoreL2Command() {
        return new ParallelCommandGroup(ElevatorFactory.moveL2(), ArmFactory.moveToL2())
                .onlyWhile(() -> DriveToPointController.getManhattanDistance() < .2);
    }

    public static Command moveToScoreL1Command() {
        return new ParallelCommandGroup(ElevatorFactory.moveL1(), ArmFactory.moveToL1());
    }

    public static Command moveToIntake() {
        return new ParallelCommandGroup(ElevatorFactory.moveToIntake(), ArmFactory.armToIntake());
    }

    public static Command moveToDescoreAlgaeL2() {
        return new ParallelCommandGroup(ElevatorFactory.moveToRemoveAlgaeL2(), ArmFactory.armToDescoreL2())
                .until(() -> arm.readyToScore());
    }

    public static Command moveToDescoreAlgaeL3() {
        return new ParallelCommandGroup(ElevatorFactory.moveToRemoveAlgaeL3(), ArmFactory.armToDescoreL3())
                .until(() -> arm.readyToScore());
    }

}
