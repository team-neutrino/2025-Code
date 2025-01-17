package frc.robot.util;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.*;

public class Subsystem {
    public static final Claw claw = new Claw();
    public static final Elevator elevator = new Elevator();
    // public static final Swerve swerve = new Swerve();
    public static final Limelight limelight = new Limelight();
    public static final LED LED = new LED();
    public static final Arm arm = new Arm();
}