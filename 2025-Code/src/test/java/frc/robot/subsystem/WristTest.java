package frc.robot.subsystem;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.lang.reflect.Method;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.Constants.WristConstants;

import frc.robot.subsystems.Wrist;
import frc.robot.util.Subsystem;

public class WristTest {
    private Wrist wrist;

    @BeforeEach
    public void beforeEach() {
        wrist = new Wrist();
    }

    @AfterEach
    public void afterEach() {
        wrist.close();
    }

    @Test
    public void testMoveToPositionBasic() {
        wrist.moveToPosition(0);
        assertTrue(getCurrentSpike(wrist) == false);
        assertTrue(getWristVoltage(wrist) == -WristConstants.VOLTAGE);
        wrist.moveToPosition(90);
        assertTrue(getCurrentSpike(wrist) == false);
        assertTrue(getWristVoltage(wrist) == WristConstants.VOLTAGE);
    }

    @Test
    public void testCurrentSpikes() {
        wrist.moveToPosition(0);
        wrist.moveToPosition(0);
        assertTrue(getCurrentSpike(wrist) == false);
        // case 2: switch directions before other position was reached
        wrist.moveToPosition(90);
        assertTrue(getCurrentSpike(wrist) == false);
        /**
         * NOTE: the cases in which the current of the motor is compared against a
         * constant are not tested here due to SparkBase's JNI usage (presumably to
         * C).
         * In short, a method could not be devised to access and modify the current
         * field of the sparkmax.
         */
    }

    private static boolean getCurrentSpike(Wrist wrist) {
        try {
            return (Boolean) getPrivateField("m_hasCurrentSpiked", wrist);
        } catch (Exception e) {
            System.out.println("misnamed field while attempting to private access" + e.getMessage());
        }
        return false;
    }

    private static double getWristVoltage(Wrist wrist) {
        try {
            return (Double) getPrivateField("m_wristVoltage", wrist);
        } catch (Exception e) {
            System.out.println();
        }
        return Integer.MIN_VALUE;
    }

    private static Object getWristMotor(Wrist wrist) {
        try {
            return getPrivateField("m_wristMotor", wrist);
        } catch (Exception e) {
            System.out.println("misnamed field while attempting to private access" + e.getMessage());
        }
        return null;
    }

    private static void muteCurrentLimit(Wrist wrist, int desiredAngle) {
        try {
            usePrivateMethod("updateCurrentSpike", wrist, desiredAngle);
        } catch (Exception e) {
            System.out.println("misnamed field while attempting to private access" + e.getMessage());
        }
    }

    /**
     * Uses BLACK SORCERY to access a private field from the wrist subsystem of your
     * choosing - just give this dark wizard the name... (don't forget to read the
     * terms and conditions of your binding contract in the 'throws' section)
     * 
     * @param fieldName The name of the field you desire.
     * @return An {@link Object} of your field, type cast it to use it.
     * @throws IllegalAccessException   dark magic liability 1
     * @throws IllegalArgumentException dark magic liability 2
     * @throws SecurityException        dark magic liability 3
     * @throws NoSuchFieldException     dark magic liability 4
     */
    private static Object getPrivateField(String fieldName, Wrist wrist)
            throws IllegalArgumentException, IllegalAccessException, NoSuchFieldException, SecurityException {
        Field accessor = null;
        accessor = wrist.getClass().getDeclaredField(fieldName);
        accessor.setAccessible(true);
        return accessor.get(wrist);
    }

    private static void usePrivateMethod(String methodName, Wrist wrist, int angleParam)
            throws IllegalArgumentException, IllegalAccessException, NoSuchFieldException, SecurityException {
        Method accessor = null;
        try {
            accessor = wrist.getClass().getDeclaredMethod(methodName, wrist.getClass());
            accessor.setAccessible(true);
            accessor.invoke(wrist, angleParam);
        } catch (Exception e) {
            System.out.println("misnamed field while attempting to private access" + e.getMessage());
        }
    }
}
