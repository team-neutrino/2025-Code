package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class ClawAndWristFactory {
    private static Claw claw = new Claw();

    public static Command wristToIntake() {
        return claw.rotateWristToIntake();
    }

    public static Command wristToScoring() {
        return claw.rotateWristToScore();
    }
}
