package frc.robot.subsystem;

import org.junit.jupiter.api.Test;
import frc.robot.subsystems.Arm;
import frc.robot.util.Subsystem;

import static org.junit.jupiter.api.Assertions.*;

public class ArmTest {

    @Test
    void testDefaultAngle() {
        var armSubsystem = new Arm();
        var armDefault = armSubsystem.getArmPosition();
        System.out.println("armDefault " + armDefault);

    }
}
