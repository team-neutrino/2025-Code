package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveToPoint.Mode;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.Constants.*;
import frc.robot.util.DriveToPointController;

import static frc.robot.util.Subsystem.*;

import java.util.function.BooleanSupplier;

public class SuperstructureFactory {


    }
                        : true;
                                && (Subsystem.limelight.getTvReef1() || Subsystem.limelight.getTvReef2())
                        ? Subsystem.swerve.isAtPoint()
                .getCurrentCommand().getName().equals(Constants.DriveToPoint.DRIVE_TO_POINT_BASIC)
        return Subsystem.swerve
        }
            return false;
        if (Subsystem.swerve.getCurrentCommand() == null) {
    public static boolean antiDriveTeamCondition() {
    /**
     * WILL ONLY WORK FOR BLUE ALLIANCE CURRENTLY
     * <p>
     * the "badSolution" parameter is becuase I don't want to repeat the expression,
     * {@code (DriveToPointCommand) swerve.getCurrentCommand()} every time I need to
     * do
     * the math needed to generate the dynamic positions. It is indeed a bad
     * solution - the commented out code at the start of the method would be a
     * better solution if it worked but this code would only run once on robot
     * startup thus meaning that it would always be null and would not magically
     * change to being the appropriate instance of DriveToPointCommand when the
     * button is pressed.
     * <p>
     * the expression in "armCom" can be changed to be an actual lambda if it turns
     * out adjusting both the arm and elevator is necessary for accurate dynamic
     * adjustment.
     * <p>
     * The logic for running should run as follows: the command only runs if the
     * DriveToPoint is also running and we are close enough to the target point for
     * the generated elevator/arm values to be reasonable. The command has the
     * typical intake end condition of having a coral.
     */
    public static Command dynamicCoralIntake(DriveToPointCommand badSolution) {
        // DriveToPointCommand currentSwerveCom = swerve.getCurrentCommand().getName()
        // .equals(DriveToPoint.DRIVE_ASSIST_COMMAND) ? (DriveToPointCommand)
        // swerve.getCurrentCommand() : null;

        Command elevCom = ElevatorFactory
                .moveToGiven(() -> (ElevatorConstants.CORAL_INTAKE - (badSolution.distStraightPlayerStation()
                        * ElevatorConstants.DYNAMIC_ADJUST_P)));
        Command armCom = ArmFactory.moveToGiven(() -> ArmConstants.CORAL_STATION_POSITION);

        BooleanSupplier runCondition = () -> swerve.getCurrentCommand() != null && swerve.getCurrentCommand().getName()
                .equals(DriveToPoint.DRIVE_TO_POINT_BASIC)
                && Math.abs(badSolution.distStraightPlayerStation()) <= DriveToPoint.DYNAMIC_INTAKE_THRESHOLD;
        BooleanSupplier endCondition = () -> coral.debouncedHasCoral();

        return elevCom.alongWith(armCom, CoralFactory.runIntake()).onlyWhile(runCondition).until(endCondition);
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
                && controller.getHID().getRightBumperButton());
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
                && controller.getHID().getRightBumperButton());
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
        Command evacWait = new WaitCommand(0.1);
        BooleanSupplier readyToScore = () -> (arm.readyToScore() && elevator.readyToScore()
                && controller.getHID().getRightBumperButton());
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
        Command evacWait = new WaitCommand(0.1);
        BooleanSupplier readyToScore = () -> (arm.readyToScore() && elevator.readyToScore());
        BooleanSupplier comEnd = () -> !coral.debouncedHasCoral();

        return ((elevatorCom.alongWith(coralDefaultCom))
                .until(() -> elevator.readyToScore())).andThen(armScoreCom).until(readyToScore)
                .andThen((coralScoreCom.alongWith(evacWait.andThen(armEvacCom))).until(comEnd));
    }

    public static Command scoreNetAutomated(CommandXboxController controller) {
        Command drive = new DriveToPointCommand(controller, DriveToPoint.Mode.NET);
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

    public static Command moveToDescoreAlgaeL2() {
        return new ParallelCommandGroup(ElevatorFactory.moveToRemoveAlgaeL2(), ArmFactory.armToDescoreL2())
                .until(() -> arm.readyToScore());
    }

    public static Command moveToDescoreAlgaeL3() {
        return new ParallelCommandGroup(ElevatorFactory.moveToRemoveAlgaeL3(), ArmFactory.armToDescoreL3())
                .until(() -> arm.readyToScore());
    }

}
