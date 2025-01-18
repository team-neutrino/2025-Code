// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.util.Subsystem.arm;

/** Add your docs here. */
public class ArmFactory {
    public static Command moveToL1() {
        return arm.ArmRotateCommand(ArmConstants.L1_POSITION);
    }
    private static Arm arm = Subsystem.arm;

    public static Command moveToL2() {
        return arm.ArmRotateCommand(ArmConstants.L2_POSITION);
    }

    public static Command moveToL3() {
        return arm.ArmRotateCommand(ArmConstants.L3_POSITION);
    }

    public static Command moveToL4() {
        return arm.ArmRotateCommand(ArmConstants.L4_POSITION);
    }

    public static Command moveToCoralStation() {
        return arm.ArmRotateCommand(ArmConstants.CORAL_STATION_POSITION);
    }
    public static Command armToIntake() {
        return arm.ArmRotateCommand(CORAL_STATION_ARM_POSITION);
    }

    public static Command armToDescoreL2() {
        return arm.ArmRotateCommand(REEF_L2_DESCORE_POSITION);
    }

    public static Command armToDescoreL3() {
        return arm.ArmRotateCommand(REEF_L3_DESCORE_POSITION);
    }

    public static Command armToScore(double level) {
        return arm.ArmRotateCommand(level);
    }
}
