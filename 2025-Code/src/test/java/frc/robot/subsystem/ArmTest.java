package frc.robot.subsystem;

import org.junit.jupiter.api.Test;
import frc.robot.subsystems.Arm;

import static org.junit.jupiter.api.Assertions.*;

public class ArmTest {

    @Test
    void testDefaultAngle() {
        var armSubsystem = new Arm();
        var armDefault = armSubsystem.getAngle();
        System.out.println("armDefault " + armDefault);

    }
}
