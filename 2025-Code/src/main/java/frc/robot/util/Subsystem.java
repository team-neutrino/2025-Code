package frc.robot.util;

import frc.robot.subsystems.*;
import frc.robot.subsystems.NetworkTables.*;

public class Subsystem {
    public static final Claw claw = new ClawNT();
    public static final Wrist wrist = new WristNT();
    public static final Elevator elevator = new ElevatorNT();
    // public static final Swerve swerve = new Swerve();
    public static final Climb climb = new Climb();
    public static final Limelight limelight = new Limelight();
    public static final LED LED = new LED();
    public static final Arm arm = new ArmNT();
    public static final PIDChanger pidChanger = new PIDChanger();
}