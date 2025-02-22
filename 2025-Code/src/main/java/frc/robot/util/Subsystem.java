package frc.robot.util;

import frc.robot.NetworkTables;
import frc.robot.subsystems.*;
import frc.robot.subsystems.NetworkTables.*;

public class Subsystem {
    public Subsystem(boolean is_valkyrie) {
        swerve = new Swerve(is_valkyrie);
        limelight = new Limelight();
    }

    public static final Coral coral = new CoralNT();
    public static final Elevator elevator = new ElevatorNT();
    public static final Climb climb = new ClimbNT();
    public static Swerve swerve = null;
    public static Algae algae = null;
    public static Limelight limelight = null;
    public static final LED LED = new LED();
    public static final Arm arm = new ArmNT();
    public static final NetworkTables networkTables = new NetworkTables();
}