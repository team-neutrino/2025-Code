package frc.robot.subsystem;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.NetworkTables.ElevatorNT;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

public class ElevatorTest {

    Elevator elevatorSubsystem;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
    }

    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach
    void shutdown() throws Exception {
        elevatorSubsystem.close();
    }

    @Test
    void testElevatorConstructionAndPositionGetters() {
        elevatorSubsystem = new Elevator();
        var target = elevatorSubsystem.getTargetPosition();
        var actual = elevatorSubsystem.getEncoderPosition();
        System.out.println("elevator target and measured " + target + " " + actual);
    }

    @Test
    void testElevatorNTConstructionAndPositionGetters() {
        elevatorSubsystem = new ElevatorNT();
        var target = elevatorSubsystem.getTargetPosition();
        var actual = elevatorSubsystem.getEncoderPosition();
        System.out.println("elevatorNT target and measured " + target + " " + actual);
    }
}
