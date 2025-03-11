package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.AlgaeConstants.*;
import static frc.robot.util.Subsystem.algae;

public class AlgaeFactory {
    public static Command runIntake() {
        return algae.runIntake(INTAKE_MOTOR_VOLTAGE);
    }

    public static Command runOuttake() {
        return algae.runIntake(-INTAKE_MOTOR_VOLTAGE);
    }
}
