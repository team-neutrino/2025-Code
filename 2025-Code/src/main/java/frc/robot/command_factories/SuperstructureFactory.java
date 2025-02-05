package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveAssistCom;

import static frc.robot.util.Subsystem.*;

public class SuperstructureFactory {
    public static Command intakeCoral(CommandXboxController driverController) {
        Command swerveAdjustToPlayerStationCom;
        Command elevatorCom = ElevatorFactory.moveToIntake();
        Command armCom = ArmFactory.armToIntake();
        Command clawCom = ClawFactory.runIntake().alongWith(WristFactory.wristToIntake());
        if (!claw.hasGamePiece()) {
            swerveAdjustToPlayerStationCom = new DriveAssistCom(driverController);
            return elevatorCom.alongWith(armCom, clawCom, swerveAdjustToPlayerStationCom)
                    .until(() -> claw.hasGamePiece());
        }
        return elevatorCom.alongWith(armCom, clawCom).until(() -> claw.hasGamePiece());
    }

    public static Command outtake() {
        Command elevatorCom = ElevatorFactory.moveToIntake();
        Command armCom = ArmFactory.armToIntake();
        Command clawCom = ClawFactory.runOuttake().alongWith(WristFactory.wristToIntake());
        return elevatorCom.alongWith(armCom, clawCom);
    }

    public static Command moveToIntake() {
        return new ParallelCommandGroup(ElevatorFactory.moveToIntake(), ArmFactory.armToIntake());
    }

    public static Command descoreAlgaeL2() {
        Command elevatorCom = ElevatorFactory.movetoRemoveAlgaeL2();
        Command armCom = ArmFactory.armToDescoreL2();
        Command clawCom = ClawFactory.runIntake().alongWith(WristFactory.wristToIntake());
        return elevatorCom.alongWith(armCom, clawCom).until(() -> claw.hasGamePiece());
    }

    public static Command descoreAlgaeL3() {
        Command elevatorCom = ElevatorFactory.movetoRemoveAlgaeL3();
        Command armCom = ArmFactory.armToDescoreL3();
        Command clawCom = ClawFactory.runIntake().alongWith(WristFactory.wristToIntake());
        return elevatorCom.alongWith(armCom, clawCom).until(() -> claw.hasGamePiece());
    }

    public static Command scoreCoralL1Command() {
        return new SequentialCommandGroup(new ParallelCommandGroup(
                ElevatorFactory.moveL1(),
                ArmFactory.moveToL1()), ClawFactory.runOuttake());
    }

    public static Command scoreCoralL2Command() {
        return new SequentialCommandGroup(new ParallelCommandGroup(
                ElevatorFactory.moveL2(),
                ArmFactory.moveToL2()), ClawFactory.runOuttake());
    }

    public static Command scoreCoralL3Command() {
        return new SequentialCommandGroup(new ParallelCommandGroup(
                ElevatorFactory.moveL3(),
                ArmFactory.moveToL3()), ClawFactory.runOuttake());
    }

    public static Command scoreCoralL4Command() {
        return new SequentialCommandGroup(new ParallelCommandGroup(
                ElevatorFactory.moveL4(),
                ArmFactory.moveToL4()), WristFactory.wristToScoring(), ClawFactory.runOuttake());
    }

    public static Command dunkL4Command() {
        return new ParallelCommandGroup(ArmFactory.dunkL4(), WristFactory.wristToScoring(), ElevatorFactory.moveL4());
    }

    public static Command dunkL3Command() {
        return new ParallelCommandGroup(ArmFactory.dunkL3(), WristFactory.wristToScoring(), ElevatorFactory.dunkL3());
    }

    public static Command dunkL2Command() {
        return new ParallelCommandGroup(ArmFactory.dunkL2(), WristFactory.wristToScoring(), ElevatorFactory.dunkL2());
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
}
