package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.util.Subsystem.arm;

public class ArmFactory {
    public static Command moveToL1() {
        return arm.armRotateCommand(L1_POSITION);
    }

    public static Command moveToUnderhand() {
        return arm.armRotateCommand(L1_UNDERHAND);
    }

    public static Command moveToL2() {
        return arm.armRotateCommand(L2_POSITION);
    }

    public static Command moveToL3() {
        return arm.armRotateCommand(L3_POSITION);
    }

    public static Command moveToL4() {
        return arm.armRotateCommand(L4_POSITION);
    }

    public static Command dunkL4() {
        return arm.armRotateCommand(L4_POSITION - 20);
    }

    public static Command dunkL3() {
        return arm.armRotateCommand(L3_POSITION - 5);
    }

    public static Command dunkL2() {
        return arm.armRotateCommand(L2_POSITION - 5);
    }

    public static Command armToIntake() {
        return arm.armRotateCommand(CORAL_STATION_POSITION);
    }

    public static Command armToDescoreL2() {
        return arm.armRotateCommand(REEF_L2_DESCORE_POSITION);
    }

    public static Command armToDescoreL3() {
        return arm.armRotateCommand(REEF_L3_DESCORE_POSITION);
    }

    public static Command armToScore(double level) {
        return arm.armRotateCommand(level);
    }

}
