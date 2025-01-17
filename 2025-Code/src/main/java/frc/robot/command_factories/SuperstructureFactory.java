package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
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
    public static Command scoreCoralL1Command() {
        return new ParallelCommandGroup(
                ElevatorFactory.moveL1(),
                Subsystem.arm.ArmRotateCommand(ArmConstants.L1_ARM_POSITION));
    }

    public static Command scoreCoralL2Command() {
        return new ParallelCommandGroup(
                ElevatorFactory.moveL2(),
                Subsystem.arm.ArmRotateCommand(ArmConstants.L2_ARM_POSITION));
    }

    public static Command scoreCoralL3Command() {
        return new ParallelCommandGroup(
                ElevatorFactory.moveL3(),
                Subsystem.arm.ArmRotateCommand(ArmConstants.L3_ARM_POSITION));
    }

    public static Command scoreCoralL4Command() {
        return new ParallelCommandGroup(
                ElevatorFactory.moveL4(),
                Subsystem.arm.ArmRotateCommand(ArmConstants.L4_ARM_POSITION));
    }

}
