package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;
import frc.robot.util.Subsystem;



public class ClimbFactory {
    public static Command raiseArmCommand() {
        Climb climb = Subsystem.climb;
        return climb.moveClimbArmCommand(ClimbConstants.CLIMB_ANGLE);
    }

    public static Command lowerArmCommand() {
        Climb climb = Subsystem.climb;
        return climb.moveClimbArmCommand(ClimbConstants.CLIMB_DOWN_ANGLE);
    }
}
