package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.Constants.ClimbConstants.*;
import static frc.robot.util.Subsystem.climb;

public class ClimbFactory {
    public static Command raiseClimb() {
        return
            // climb.disengageRatchetCommand().alongWith(
            climb.moveClimbArmCommand(CLIMB_UP_POSITION);
    }
    
    public static Command lowerClimb() {
        return 
            climb.moveClimbArmCommand(CLIMB_DOWN_POSITION);
            // .alongWith(
            // climb.engageRatchetCommand());
    }

    public static Command lockGrabber() {
        return climb.lockCommand(LOCK_POSITION);
    }

    public static Command unlockGrabber() {
        return climb.lockCommand(UNLOCK_POSITOIN);
    }

    public static Command resetLock() {
        return climb.resetLockCommand(RESET_POSITION);
    }

    /** 
     * only use when climb arm is in a state where it is all the way up (relaxed)
     * */ 
    public static Command resetClimb() {
        return new SequentialCommandGroup(
            climb.resetClimbArmCommand(RESET_CLIMB_ROTATION),
            climb.engageRatchetCommand());
    }
}
