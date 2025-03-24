package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.AlgaeConstants.OUTTAKE_VOLTAGE;
import static frc.robot.Constants.CoralConstants.*;
import static frc.robot.util.Subsystem.coral;

public class CoralFactory {
    public static Command runIntake() {
        return coral.runIntake(INTAKE_VOLTAGE).until(() -> coral.hasCoral());
    }

    public static Command runOuttake() {
        return coral.runIntake(OUTTAKE_VOLTAGE);
    }

    public static Command runSlowOuttake() {
        return coral.runIntake(SLOW_SCORE_VOLTAGE);
    }

    public static Command runSlowIntake() {
        return coral.runIntake(HOLD_PIECE_AUTON_VOLTAGE);
    }
}
