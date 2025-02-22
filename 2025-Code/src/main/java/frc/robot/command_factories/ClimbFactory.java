package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.Constants.ClimbConstants.*;
import static frc.robot.util.Subsystem.climb;

public class ClimbFactory {
    public static Command raiseClimb() {
        return new SequentialCommandGroup(
                climb.prepareClimbCommand().withTimeout(COMMAND_WAIT_TIME),
                climb.raiseClimbArmCommand());
    }

    public static Command lowerClimb() {
        return new SequentialCommandGroup(
                climb.lowerClimbArmCommand(),
                climb.hasClimbCommand());
    }

    public static Command lockGrabber() {
        return climb.lockCommand();
    }

    /**
     * pit controller. Sets the lock position to 0
     */
    public static Command resetLock() {
        return climb.resetLockCommand();
    }

    /**
     * pit controller.
     * only use when climb arm is in a state where it is all the way up (relaxed)
     */
    public static Command resetClimb() {
        return climb.resetClimbArmCommand();
    }
}
