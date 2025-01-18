package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.Subsystem;
import frc.robot.subsystems.Arm;

public class ArmFactory {
    public static Command moveToL1() {
        Arm arm = Subsystem.arm;
        return arm.ArmRotateCommand(ArmConstants.L1_POSITION);
    }

    public static Command moveToL2() {
        Arm arm = Subsystem.arm;
        return arm.ArmRotateCommand(ArmConstants.L2_POSITION);
    }

    public static Command moveToL3() {
        Arm arm = Subsystem.arm;
        return arm.ArmRotateCommand(ArmConstants.L3_POSITION);
    }

    public static Command moveToL4() {
        Arm arm = Subsystem.arm;
        return arm.ArmRotateCommand(ArmConstants.L4_POSITION);
    }

    public static Command armToIntake() {
        Arm arm = Subsystem.arm;
        return arm.ArmRotateCommand(ArmConstants.CORAL_STATION_POSITION);
    }
    public static Command armToIntake() {
        return arm.ArmRotateCommand(CORAL_STATION_ARM_POSITION);
    }

    public static Command armToDescoreL2() {
        Arm arm = Subsystem.arm;
        return arm.ArmRotateCommand(ArmConstants.REEF_L2_DESCORE_POSITION);
    }

    public static Command armToDescoreL3() {
        Arm arm = Subsystem.arm;
        return arm.ArmRotateCommand(ArmConstants.REEF_L3_DESCORE_POSITION);
    }

    public static Command armToScore(double level) {
        return arm.ArmRotateCommand(level);
    }
}
