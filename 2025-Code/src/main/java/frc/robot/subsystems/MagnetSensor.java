package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MagnetSensor extends SubsystemBase{
    private static final int sensor_address = 0x1E;
    private I2C magnetSensor;

    public MagnetSensor() {
        magnetSensor = new I2C(I2C.Port.kOnboard, sensor_address);
    }

    public int readMagnetometerX() {
        byte[] buffer = new byte[2]; // Magnetometer data is 16-bit (2 bytes)
        magnetSensor.read(0x03, 2, buffer); // Read from the X-axis register (example register)
        return (buffer[0] << 8) | (buffer[1] & 0xFF);
    }

    @Override
    public void periodic() {
        System.out.println("Magnetometer X: " + readMagnetometerX());
    }
}
