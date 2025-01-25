package frc.robot.subsystems.NetworkTables;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PIDTuner;

public class ElevatorNT extends Elevator {

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic encoderPosition = nt.getDoubleTopic("/elevator/encoder_position");
    DoubleTopic targetPosition = nt.getDoubleTopic("/elevator/target_position");
    DoubleTopic voltage = nt.getDoubleTopic("/elevator/motor_input_voltage");
    final DoublePublisher encoderPositionPub;
    final DoublePublisher targetPositionPub;
    final DoublePublisher motorVoltagePub;
    PIDTuner m_PIDTuner;

    public ElevatorNT() {
        encoderPositionPub = encoderPosition.publish();
        encoderPositionPub.setDefault(0.0);

        targetPositionPub = targetPosition.publish();
        targetPositionPub.setDefault(0.0);

        motorVoltagePub = voltage.publish();
        motorVoltagePub.setDefault(0.0);

        m_PIDTuner = new PIDTuner("elevator");
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();
        encoderPositionPub.set(getEncoderPosition(), now);
        targetPositionPub.set(getTargetPosition(), now);
        motorVoltagePub.set(getInputVoltage(), now);
    }

}
