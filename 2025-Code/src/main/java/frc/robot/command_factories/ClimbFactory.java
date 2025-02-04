package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.util.Subsystem.climb;

public class ClimbFactory {
    public static Command raiseClimb() {
        return climb.raiseClimbArmCommand();
    }

    public static Command lowerClimb() {
        return climb.lowerClimbArmCommand();
    }

    public static Command lockGrabber() {
        return climb.lockCommand();
    }

    public static Command test() {
        return climb.moveToPositionCommand(2);
    }
}
