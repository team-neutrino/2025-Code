package frc.robot.command_factories;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Subsystem;

public class LimelightFactory {

    public static Command alignToAprilTag() {
        return Subsystem.swerve.driveForAutoAlign(Subsystem.limelight.getXboxController(),
                Subsystem.limelight.offsetToOmega(-Subsystem.limelight.getTx()));
    }

    public static Command alignToAprilTagFacingAngle() {
        return Subsystem.swerve.driveForAutoAlignFieldFacing(Subsystem.limelight.getXboxController(),
                Subsystem.limelight.getTargetYawRotation2d());
    }
}
