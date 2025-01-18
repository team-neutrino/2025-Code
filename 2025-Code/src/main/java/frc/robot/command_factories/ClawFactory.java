package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.ClawConstants.*;
import frc.robot.subsystems.Claw;
import frc.robot.util.Subsystem;

public class ClawFactory {
    private static Claw claw = Subsystem.claw;

    public static Command wristToIntake() {
        return claw.rotateWrist(WRIST_INTAKE_POS);
    }

    public static Command wristToScoring() {
        return claw.rotateWrist(WRIST_SCORING_POS);
    }

    public static Command runIntake() {
        return claw.runIntake(INTAKE_MOTOR_VOLTAGE);
    }

    public static Command runOuttake() {
        return claw.runIntake(-INTAKE_MOTOR_VOLTAGE);
    }
}
