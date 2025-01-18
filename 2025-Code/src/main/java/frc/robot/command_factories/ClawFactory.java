package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;
import frc.robot.util.Subsystem;

public class ClawFactory {
    private static Claw claw = Subsystem.claw;

    public static Command wristToIntake() {
        return claw.rotateWrist(ClawConstants.WRIST_INTAKE_POS);
    }

    public static Command wristToScoring() {
        return claw.rotateWrist(ClawConstants.WRIST_SCORING_POS);
    }

    public static Command runIntake() {
        return claw.runIntake(ClawConstants.INTAKE_MOTOR_VOLTAGE);
    }

    public static Command runOuttake() {
        return claw.runIntake(-ClawConstants.INTAKE_MOTOR_VOLTAGE);
    }
}
