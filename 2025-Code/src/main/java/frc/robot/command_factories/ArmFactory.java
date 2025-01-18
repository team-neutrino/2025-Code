package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.Subsystem;

public class ArmFactory {
    public static Command moveToL1() {
        return Subsystem.arm.armRotateCommand(ArmConstants.L1_POSITION);
    }

    public static Command moveToL2() {
        return Subsystem.arm.armRotateCommand(ArmConstants.L2_POSITION);
    }

    public static Command moveToL3() {
        return Subsystem.arm.armRotateCommand(ArmConstants.L3_POSITION);
    }

    public static Command moveToL4() {
        return Subsystem.arm.armRotateCommand(ArmConstants.L4_POSITION);
    }

    public static Command armToIntake() {
        return Subsystem.arm.armRotateCommand(ArmConstants.CORAL_STATION_POSITION);
    }
    public static Command armToIntake() {
        return arm.ArmRotateCommand(CORAL_STATION_ARM_POSITION);
    }

    public static Command armToDescoreL2() {
        return Subsystem.arm.armRotateCommand(ArmConstants.REEF_L2_DESCORE_POSITION);
    }

    public static Command armToDescoreL3() {
        return Subsystem.arm.armRotateCommand(ArmConstants.REEF_L3_DESCORE_POSITION);
    }

    public static Command armToScore(double level) {
        return arm.ArmRotateCommand(level);
    }
}
