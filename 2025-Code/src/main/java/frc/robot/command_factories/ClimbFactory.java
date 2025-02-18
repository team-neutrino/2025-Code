package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants.States;
import static frc.robot.util.Subsystem.LED;

import static frc.robot.Constants.ClimbConstants.*;
import static frc.robot.util.Subsystem.climb;

public class ClimbFactory {
    public static Command raiseClimb() {
        return climb.moveClimbArmCommand(CLIMB_UP_POSITION);
    }

    public static Command lowerClimb() {
        return climb.moveClimbArmCommand(CLIMB_DOWN_POSITION);
    }

    public static Command lockGrabber() {
        LED.setCommandState(States.LOCKCLIMB);
        return climb.lockCommand();
    }
}
