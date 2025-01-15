package frc.robot.util;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;

public class Subsystem {
    public static final Claw claw = new Claw();
    public static final Elevator elevator = new Elevator();
    public static final Limelight limelight = new Limelight();
    public static final LED LED = new LED();
}