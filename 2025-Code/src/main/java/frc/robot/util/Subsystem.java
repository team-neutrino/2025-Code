package frc.robot.util;

import frc.robot.subsystems.*;

public class Subsystem {
    public static final boolean ENABLE_SUPERSTRUCTURE = true;
    public static final boolean ENABLE_DRIVETRAIN = false;
    public static final boolean ENABLE_LIMELIGHT = false;
    public static final boolean ENABLE_LED = ENABLE_SUPERSTRUCTURE && ENABLE_DRIVETRAIN && ENABLE_LIMELIGHT;

    public static final Arm arm = ENABLE_SUPERSTRUCTURE ? new Arm() : null;
    public static final Claw claw = ENABLE_SUPERSTRUCTURE ? new Claw() : null;
    public static final Elevator elevator = ENABLE_SUPERSTRUCTURE ? new Elevator() : null;
    public static final Swerve swerve = ENABLE_DRIVETRAIN ? new Swerve() : null;
    public static final Limelight limelight = ENABLE_LIMELIGHT ? new Limelight() : null;
    public static final LED LED = ENABLE_LED ? new LED() : null;
}