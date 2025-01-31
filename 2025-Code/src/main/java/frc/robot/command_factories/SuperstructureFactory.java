package frc.robot.command_factories;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve.SwerveRequestStash;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.util.Subsystem.*;

public class SuperstructureFactory {
    public static Command intakeCoral() {
        Command elevatorCom = ElevatorFactory.moveToIntake();
        Command armCom = ArmFactory.armToIntake();
        Command clawCom = ClawFactory.runIntake().alongWith(WristFactory.wristToIntake());
        return elevatorCom.alongWith(armCom, clawCom).until(() -> claw.hasGamePiece());
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
                ArmFactory.moveToL4()), ClawFactory.runOuttake());
    }

    public static Command moveToScoreL4Command() {
        return new ParallelCommandGroup(ElevatorFactory.moveL4(), ArmFactory.moveToL4());
    }

    public static Command moveToScoreL3Command() {
        return new ParallelCommandGroup(ElevatorFactory.moveL3(), ArmFactory.moveToL3());
    }

    /**
     * Give this method the DRIVER controller.
     * 
     * @param driverController The DRIVER controller
     */
    public static Command autoAlign(CommandXboxController driverController) {
        return swerve.applyRequest(() -> SwerveRequestStash.autoAlignBaseline(driverController)
                // .withTargetDirection(new Rotation2d(-Math.PI / 3)));
                .withTargetDirection(Rotation2d.fromDegrees(swerve.getYawDegrees() -
                        limelight.getTx())));
    }
}
