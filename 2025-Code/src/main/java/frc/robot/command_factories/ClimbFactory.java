package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.Constants.ClimbConstants.*;
import static frc.robot.util.Subsystem.climb;

public class ClimbFactory {
    public static Command raiseClimb() {
        return new SequentialCommandGroup(
            climb.prepareClimbCommand(),
            new WaitCommand(COMMAND_WAIT_TIME),
            climb.raiseClimbArmCommand()
        );
    }
    
    public static Command lowerClimb() {
        return climb.lowerClimbArmCommand();
    }

    public static Command lockGrabber() {
        return climb.moveLockCommand(LOCK_POSITION);
    }

    public static Command resetLock() {
        return climb.moveLockCommand(RESET_LOCK_POSITION);
    }

    /** 
     * only use when climb arm is in a state where it is all the way up (relaxed)
     * */ 
    public static Command resetClimb() {
        return climb.resetClimbArmCommand();
    }
}
