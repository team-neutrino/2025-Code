package frc.robot.subsystems.NetworkTables;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.Claw;
import frc.robot.util.PIDTuner;

public class ClawNT extends Claw {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic encoderVelocity = nt.getDoubleTopic("/claw/encoder_velocity");
    DoubleTopic voltage = nt.getDoubleTopic("/claw/motor_input_voltage");
    final DoublePublisher encoderVelocityPub;
    final DoublePublisher motorVoltagePub;
    PIDTuner m_PIDTuner;

    public ClawNT() {
        encoderVelocityPub = encoderVelocity.publish();
        encoderVelocityPub.setDefault(0.0);

        motorVoltagePub = voltage.publish();
        motorVoltagePub.setDefault(0.0);

        m_PIDTuner = new PIDTuner("claw");
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();
        encoderVelocityPub.set(getVelocityOfGrabber(), now);
        motorVoltagePub.set(getIntakeVoltage(), now);
    }
}
