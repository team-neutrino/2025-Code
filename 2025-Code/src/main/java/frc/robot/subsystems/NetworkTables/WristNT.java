package frc.robot.subsystems.NetworkTables;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.Wrist;

public class WristNT extends Wrist {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic voltage = nt.getDoubleTopic("/wrist/motor_input_voltage");
    DoubleTopic lastAngle = nt.getDoubleTopic("/wrist/last_input_angle");
    BooleanTopic isCurrentSpiked = nt.getBooleanTopic("/arm/motor_has_current_spiked");
    final DoublePublisher voltagePub;
    final DoublePublisher lastAnglePub;
    final BooleanPublisher isCurrentSpikedPub;

    public WristNT() {
        voltagePub = voltage.publish();
        voltagePub.setDefault(0.0);

        lastAnglePub = lastAngle.publish();
        lastAnglePub.setDefault(0.0);

        isCurrentSpikedPub = isCurrentSpiked.publish();
        isCurrentSpikedPub.setDefault(false);
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();
        voltagePub.set(getWristVoltage(), now);
        lastAnglePub.set(getLastAngle(), now);
        isCurrentSpikedPub.set(isCurrentSpike(), now);
    }
}
