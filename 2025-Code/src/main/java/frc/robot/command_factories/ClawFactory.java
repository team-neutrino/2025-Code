package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.ClawConstants.*;
import static frc.robot.util.Subsystem.claw;

public class ClawFactory {
    public static Command runIntake() {
        return claw.runIntake(INTAKE_MOTOR_VOLTAGE);
    }

    public static Command runOuttake() {
        return claw.runIntake(-INTAKE_MOTOR_VOLTAGE);
    }
}
