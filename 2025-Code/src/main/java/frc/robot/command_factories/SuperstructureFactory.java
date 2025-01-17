package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.util.Subsystem;

public class SuperstructureFactory {
    private static Elevator elevator = Subsystem.elevator;
    private static Claw claw = Subsystem.claw;
    private static Arm arm = Subsystem.arm;

    public static Command intakeCoral() {
        // 3 neo 550s, 3 vortexes
        Command elevatorCom = ElevatorFactory.moveToIntake();
        Command armCom = ArmFactory.armToIntake();
        Command clawCom = ClawFactory.runIntakeCoral().alongWith(ClawFactory.wristToIntake());
        return elevatorCom.alongWith(armCom, clawCom).until(() -> claw.hasGamePiece());
    }
}
