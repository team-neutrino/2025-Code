package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.ClimbConstants.CLIMB_DOWN_POSITION;
import static frc.robot.Constants.ClimbConstants.CLIMB_UP_POSITION;
import static frc.robot.util.Subsystem.climb;

public class ClimbFactory {
    public static Command raiseClimb() {
        return climb.moveClimbArmCommand(CLIMB_UP_POSITION);
    }

    public static Command lowerClimb() {
        return climb.moveClimbArmCommand(CLIMB_DOWN_POSITION);
    }

    public static Command lockGrabber() {
        return climb.lockCommand();
    }

    public static Command test() {
        return climb.moveClimbArmCommand(2);
    }
}
