// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.util.Subsystem;

/** Add your docs here. */
public class ArmFactory {
    private static Arm arm = Subsystem.arm;

    public static Command armToIntake() {
        return arm.ArmMoveCommand(ArmConstants.CORAL_STATION_ARM_POSITION);
    }
}
