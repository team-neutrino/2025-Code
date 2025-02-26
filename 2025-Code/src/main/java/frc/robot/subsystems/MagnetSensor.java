package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MagnetSensor extends SubsystemBase {
    private static final int SENSOR_ADDRESS = 0x1E; // LSM9DS1 Magnetometer I2C Address
    private I2C magnetSensor;

    public MagnetSensor() {
        magnetSensor = new I2C(I2C.Port.kOnboard, SENSOR_ADDRESS);

        // ✅ Initialize the magnetometer (set continuous mode)
        magnetSensor.write(0x20, 0xFC); // CTRL_REG1_M: Ultra-high performance, continuous mode
    }

    private int readMagnetometerAxis(int lowReg, int highReg) {
        byte[] buffer = new byte[2];

        // Read low byte
        magnetSensor.read(lowReg, 1, buffer);
        // Read high byte
        magnetSensor.read(highReg, 1, buffer);

        // Combine high and low bytes into a 16-bit value
        int value = (buffer[1] << 8) | (buffer[0] & 0xFF);

        // Convert to signed 16-bit
        if (value > 32767) value -= 65536;

        return value;
    }

    public int getMagnetometerX() {
        return readMagnetometerAxis(0x28, 0x29);
    }

    public int getMagnetometerY() {
        return readMagnetometerAxis(0x2A, 0x2B);
    }

    public int getMagnetometerZ() {
        return readMagnetometerAxis(0x2C, 0x2D);
    }

    @Override
    public void periodic() {
        System.out.println("Magnetometer X: " + getMagnetometerX());
        System.out.println("Magnetometer Y: " + getMagnetometerY());
        System.out.println("Magnetometer Z: " + getMagnetometerZ());
    }
}