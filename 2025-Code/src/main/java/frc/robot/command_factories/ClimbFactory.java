package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.util.Subsystem;

public class ClimbFactory {
    public static Command raiseClimbArmCommand() {
        Climb climb = Subsystem.climb;
        return climb.raiseClimbArmCommand();
    }

    public static Command lowerClimbArmCommand() {
        Climb climb = Subsystem.climb;
        return climb.lowerClimbArmCommand();
    }

    // public static Command lockCommand() {
    //     Climb climb = Subsystem.climb;
    //     return climb.lockCommand();
    // }
}
