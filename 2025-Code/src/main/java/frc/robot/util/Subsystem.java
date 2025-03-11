package frc.robot.util;

import frc.robot.subsystems.*;
import frc.robot.subsystems.NetworkTables.*;

public class Subsystem {
    public Subsystem(boolean is_valkyrie) {
        swerve = new Swerve(is_valkyrie);
        limelight = new Limelight();
    }

    public static final Coral coral = new Coral();
    public static final Elevator elevator = new ElevatorNT();
    public static final Climb climb = new Climb();
    public static Swerve swerve = null;
    public static Limelight limelight = null;
    public static final LED LED = new LED();
    public static final Arm arm = new ArmNT();
    public static final Algae algae = new AlgaeNT();
}