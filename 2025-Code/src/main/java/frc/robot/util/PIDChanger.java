
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;

import frc.robot.subsystems.Arm;

public class PIDChanger extends Arm {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic encoderPosition = nt.getDoubleTopic("/arm/encoder_position");
    DoubleTopic targetPosition = nt.getDoubleTopic("/arm/target_position");
    DoubleTopic voltage = nt.getDoubleTopic("/arm/motor_input_voltage");
    final DoublePublisher encoderPositionPub;
    final DoublePublisher targetPositionPub;
    final DoublePublisher motorVoltagePub;

    public PIDChanger() {
        encoderPositionPub = encoderPosition.publish();
        encoderPositionPub.setDefault(0.0);

        targetPositionPub = targetPosition.publish();
        targetPositionPub.setDefault(0.0);

        motorVoltagePub = voltage.publish();
        motorVoltagePub.setDefault(0.0);
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();
        encoderPositionPub.set(getArmEncoderPosition(), now);
        targetPositionPub.set(getArmTargetPosition(), now);
        motorVoltagePub.set(getArmVoltage(), now);
    }

}
