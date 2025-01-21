package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;
import frc.robot.util.Subsystem;

public class ClimbFactory {
    public static Command raiseArmCommand() {
        Climb climb = Subsystem.climb;
        return climb.raiseClimbArmCommand(ClimbConstants.ARM_UP_TICKS);
    }

    public static Command lowerArmCommand() {
        Climb climb = Subsystem.climb;
        return climb.lowerClimbArmCommand(ClimbConstants.ARM_DOWN_TICKS);
    }

    public Command lockCommand() {
        Climb climb = Subsystem.climb;
    return new RunCommand(() -> {
        climb.lockClimb(); 
        climb.engageLockRatchet();
    }, climb);
  }
}
