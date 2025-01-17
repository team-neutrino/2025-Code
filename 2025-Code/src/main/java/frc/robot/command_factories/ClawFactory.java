package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

public class ClawFactory {
    private static Claw claw = new Claw();

    public static Command wristToIntake() {
        return claw.rotateWrist(90);
    }

    public static Command wristToScoring() {
        return claw.rotateWrist(0);
    }

    public static Command runIntakeCoral() {
        return claw.runIntake(ClawConstants.INTAKE_MOTOR_VOLTAGE);
    }

    public static Command runOuttakeCoral() {
        return claw.runIntake(-ClawConstants.INTAKE_MOTOR_VOLTAGE);
    }
}
