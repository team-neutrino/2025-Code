package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.*;

import static frc.robot.util.Subsystem.*;

public class SuperstructureFactory {
    public static Command intakeCoral() {
        Command elevatorCom = ElevatorFactory.moveToIntake();
        Command armCom = ArmFactory.armToIntake();
        Command clawCom = ClawFactory.runIntake().alongWith(WristFactory.wristToIntake());
        return elevatorCom.alongWith(armCom, clawCom).until(() -> claw.hasGamePiece());
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
        return new ParallelCommandGroup(
                ElevatorFactory.moveL1(),
                ArmFactory.armToScore(ArmConstants.L1_POSITION));
    }

    public static Command scoreCoralL2Command() {
        return new ParallelCommandGroup(
                ElevatorFactory.moveL2(),
                ArmFactory.armToScore(ArmConstants.L2_POSITION));
    }

    public static Command scoreCoralL3Command() {
        return new ParallelCommandGroup(
                ElevatorFactory.moveL3(),
                ArmFactory.armToScore(ArmConstants.L3_POSITION));
    }

    public static Command scoreCoralL4Command() {
        return new ParallelCommandGroup(
                ElevatorFactory.moveL4(),
                ArmFactory.armToScore(ArmConstants.L4_POSITION));
    }

}
