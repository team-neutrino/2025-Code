package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.util.Subsystem.arm;

public class ArmFactory {
    private static int lastPov = -1;

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
        return arm.armRotateCommand(L4_POSITION + 15);
    }

    // public static Command modifyL4Arm(CommandXboxController controller) {
    // return new RunCommand(() -> {
    // int pov = controller.getHID().getPOV();
    // if (lastPov == -1 && pov != -1) {
    // L4_POSITION += pov == 90 ? 3 : pov == 270 ? -3 : 0;
    // }
    // lastPov = pov;
    // });
    // }

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
