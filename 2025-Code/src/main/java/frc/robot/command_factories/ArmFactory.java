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
        return arm.ArmRotateCommand(ArmConstants.CORAL_STATION_ARM_POSITION);
    }

    public static Command armToDescoreL2() {
        return arm.ArmRotateCommand(ArmConstants.REEF_L2_DESCORE_POSITION);
    }

    public static Command armToDescoreL3() {
        return arm.ArmRotateCommand(ArmConstants.REEF_L3_DESCORE_POSITION);
    }
}
