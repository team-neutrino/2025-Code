package frc.robot.util;

import frc.robot.subsystems.*;

public class Subsystem {
    public Subsystem(boolean is_valkyrie) {
        swerve = new Swerve(is_valkyrie);
    }

    public static Swerve swerve = null;
}