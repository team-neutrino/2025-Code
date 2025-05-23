package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.Constants.ClimbConstants.*;
import static frc.robot.util.Subsystem.climb;

public class ClimbFactory {
    public static Command raiseClimb() {
        return new SequentialCommandGroup(
                climb.prepareClimbCommand().withTimeout(PREPARE_CLIMB_WAIT_TIME),
                climb.raiseClimbCommand());
    }

    public static Command lowerClimb() {
        return new SequentialCommandGroup(
                climb.lowerClimbCommand().withTimeout(LOWER_CLIMB_TIMEOUT), climb.holdClimbLockRatchet().withTimeout(LOCK_RATCHET_WAIT_TIME),
                climb.hasClimbCommand());
    }
}
