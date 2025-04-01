package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.util.Subsystem;

import static frc.robot.Constants.AlgaeConstants.OUTTAKE_VOLTAGE;
import static frc.robot.Constants.CoralConstants.*;
import static frc.robot.util.Subsystem.coral;

import java.util.function.BooleanSupplier;

public class CoralFactory {
    public static Command runIntake() {
        return coral.runIntake(INTAKE_VOLTAGE).until(() -> coral.hasCoral());
    }

    public static Command runOuttake() {
        BooleanSupplier antiDriveTeamCondition = () -> Subsystem.swerve
                .getCurrentCommand() instanceof DriveToPointCommand ? Subsystem.swerve.isAtPoint() : true;
        return coral.runIntake(OUTTAKE_VOLTAGE).onlyWhile(antiDriveTeamCondition);
    }

    public static Command runSlowOuttake() {
        return coral.runIntake(SLOW_SCORE_VOLTAGE);
    }

    public static Command runSlowIntake() {
        return coral.runIntake(HOLD_PIECE_AUTON_VOLTAGE);
    }
}
