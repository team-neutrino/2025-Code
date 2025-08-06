package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorTest {

    private Elevator elevator;
    private RelativeEncoder mockEncoder;
    private SparkLimitSwitch mockLowLimit;

    @BeforeEach
    public void setUp() {
        mockEncoder = Mockito.mock(RelativeEncoder.class);
        mockLowLimit = Mockito.mock(SparkLimitSwitch.class);
        elevator = new Elevator(mockEncoder, mockLowLimit);
    }

    @AfterEach
    public void tearDown() {
        elevator.close();
        elevator = null;
        mockEncoder = null;
        mockLowLimit = null;
    }

    @Test
    public void testGetVelocity() {
        Mockito.when(mockEncoder.getVelocity()).thenReturn(5.0);
        assertEquals(5.0, elevator.getVelocity(), "Velocity should match the encoder's velocity.");
    }

    @Test
    public void testGetHeight() {
        Mockito.when(mockEncoder.getPosition()).thenReturn(10.0);
        assertEquals(10.0, elevator.getHeight(), "Height should match the encoder's position.");
    }

    @Test
    public void testGetTargetHeight() {
        elevator.setTargetHeight(15.0);
        assertEquals(15.0, elevator.getTargetHeight(), "Target height should match the set value.");
    }

    @Test
    public void testIsAtBottom() {
        Mockito.when(mockLowLimit.isPressed()).thenReturn(true);
        assertTrue(elevator.isAtBottom(), "Elevator should be at the bottom when the low limit switch is pressed.");

        Mockito.when(mockLowLimit.isPressed()).thenReturn(false);
        assertFalse(elevator.isAtBottom(),
                "Elevator should not be at the bottom when the low limit switch is not pressed.");
    }

    @Test
    public void testReadyToScore() {
        elevator.setTargetHeight(1.5);
        Mockito.when(mockEncoder.getPosition()).thenReturn(1.5);
        assertTrue(elevator.readyToScore(),
                "Elevator should be ready to score when at target height and not at restricted heights.");

        elevator.setTargetHeight(DEFAULT_NO_CORAL);
        assertFalse(elevator.readyToScore(), "Elevator should not be ready to score when at DEFAULT_NO_CORAL height.");

        elevator.setTargetHeight(CORAL_INTAKE);
        assertFalse(elevator.readyToScore(), "Elevator should not be ready to score when at CORAL_INTAKE height.");

        elevator.setTargetHeight(DEFAULT_WITH_CORAL);
        assertFalse(elevator.readyToScore(),
                "Elevator should not be ready to score when at DEFAULT_WITH_CORAL height.");
    }
}