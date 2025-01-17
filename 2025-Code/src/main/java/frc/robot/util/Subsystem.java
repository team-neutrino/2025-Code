package frc.robot.util;

import frc.robot.subsystems.*;

public class Subsystem {
    public static final boolean ENABLE_ARM = true;
    public static final boolean ENABLE_CLAW = true;
    public static final boolean ENABLE_ELEVATOR = true;
    public static final boolean ENABLE_DRIVETRAIN = false;
    public static final boolean ENABLE_LIMELIGHT = true;
    public static final boolean ENABLE_LED = true;

    public static final Arm arm = ENABLE_ARM ? new Arm() : null;
    public static final Claw claw = ENABLE_CLAW ? new Claw() : null;
    public static final Elevator elevator = ENABLE_ELEVATOR ? new Elevator() : null;
    public static final Swerve swerve = ENABLE_DRIVETRAIN ? new Swerve() : null;
    public static final Limelight limelight = ENABLE_LIMELIGHT ? new Limelight() : null;
    public static final LED LED = ENABLE_LED ? new LED() : null;
}