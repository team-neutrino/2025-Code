package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.Constants.ClimbConstants.*;
import static frc.robot.util.Subsystem.climb;

public class ClimbFactory {
    public static Command raiseClimb() {
        return new SequentialCommandGroup(
            climb.motorOffFalseCommand(),
            climb.disengageRatchetCommand(),
            new WaitCommand(COMMAND_WAIT_TIME),
            new ParallelCommandGroup(climb.moveClimbArmCommand(CLIMB_UP_POSITION), climb.lockCommand(UNLOCK_POSITION))
        );
    }
    
    public static Command lowerClimb() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(climb.moveClimbArmCommand(CLIMB_DOWN_POSITION), climb.engageRatchetCommand()),
            climb.motorOffTrueCommand());
    }

    public static Command lockGrabber() {
        return climb.lockCommand(LOCK_POSITION);
    }

    public static Command resetLock() {
        return climb.resetLockCommand(RESET_LOCK_POSITION);
    }

    /** 
     * only use when climb arm is in a state where it is all the way up (relaxed)
     * */ 
    public static Command resetClimb() {
        return new ParallelCommandGroup(
            climb.resetClimbArmCommand(RESET_CLIMB_ROTATION),
            climb.engageRatchetCommand());
    }
}
