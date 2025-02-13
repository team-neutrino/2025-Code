package frc.robot.subsystems.NetworkTables;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.Claw;

public class ClawNT extends Claw {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic encoderVelocity = nt.getDoubleTopic("/claw/encoder_velocity");
    DoubleTopic voltage = nt.getDoubleTopic("/claw/motor_input_voltage");
    BooleanTopic proxSensor = nt.getBooleanTopic("/claw/has_gamepiece");
    final DoublePublisher encoderVelocityPub;
    final DoublePublisher motorVoltagePub;
    final BooleanPublisher proxSensorPub;

    public ClawNT() {
        encoderVelocityPub = encoderVelocity.publish();
        encoderVelocityPub.setDefault(0.0);

        motorVoltagePub = voltage.publish();
        motorVoltagePub.setDefault(0.0);

        proxSensorPub = proxSensor.publish();
        proxSensorPub.setDefault(false);
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();
        encoderVelocityPub.set(getVelocityOfGrabber(), now);
        motorVoltagePub.set(getIntakeVoltage(), now);
        proxSensorPub.set(hasGamePiece(), now);
    }
}
