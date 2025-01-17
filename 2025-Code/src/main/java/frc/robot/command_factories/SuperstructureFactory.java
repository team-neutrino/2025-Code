package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.util.Subsystem;

public class SuperstructureFactory {
    private static Elevator elevator = Subsystem.elevator;
    private static Claw claw = Subsystem.claw;
    private static Arm arm = Subsystem.arm;

    public static Command intakeCoral() {
        // 3 neo 550s, 3 vortexes
        Command elevatorCom = ElevatorFactory.moveToIntake();
        Command armCom = arm.ArmMoveCommand(ArmConstants.CORAL_STATION_ARM_POSITION);
        Command clawCom = claw.intakeGamePiece();
        return elevatorCom.alongWith(armCom).alongWith(clawCom).until(() -> claw.hasGamePiece());
    }
}
