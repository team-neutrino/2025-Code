package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.CoralConstants.*;
import static frc.robot.util.Subsystem.coral;

public class CoralFactory {
    public static Command runIntake() {
        return coral.runIntake(INTAKE_MOTOR_VOLTAGE);
    }

    public static Command runOuttake() {
        return coral.runIntake(-INTAKE_MOTOR_VOLTAGE);
    }
}
