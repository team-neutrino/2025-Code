// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.util.Subsystem.wrist;
import static frc.robot.Constants.WristConstants.*;

/** Add your docs here. */
public class WristFactory {
    public static Command wristToIntake() {
        return wrist.rotateWrist(WRIST_INTAKE_POS);
    }

    public static Command wristToScoring() {
        return wrist.rotateWrist(WRIST_SCORING_POS);
    }
}
