package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;

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

    public static Command evacuateScoreL4() {
        return arm.armRotateCommand(L4_POSITION + EVACUATE_ANGLE);
    }

    public static Command armToIntake() {
        return arm.armRotateCommand(CORAL_STATION_POSITION);
    }

    public static Command armToDescoreL2() {
        return arm.armRotateCommand(REEF_DESCORE_POSITION);
    }

    public static Command armToDescoreL3() {
        return arm.armRotateCommand(REEF_DESCORE_POSITION);
    }

    public static Command armToScoreBarge() {
        return arm.armRotateCommand(BARGE_POSITION);
    }

    public static Command armToScoreProcessor() {
        return arm.armRotateCommand(PROCESSOR_POSITION);
    }

    public static Command armToAlgaeDefault() {
        return arm.armRotateCommand(ALGAE_FRONT_SAFE_ANGLE);
    }

}
