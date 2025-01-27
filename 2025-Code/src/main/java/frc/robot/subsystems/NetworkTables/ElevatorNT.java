package frc.robot.subsystems.NetworkTables;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.PIDTuner;

public class ElevatorNT extends Elevator {

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic encoderPosition = nt.getDoubleTopic("/elevator/encoder_position");
    DoubleTopic targetPosition = nt.getDoubleTopic("/elevator/target_position");
    DoubleTopic voltage = nt.getDoubleTopic("/elevator/motor_input_voltage");
    final DoublePublisher encoderPositionPub;
    final DoublePublisher targetPositionPub;
    final DoublePublisher motorVoltagePub;
    PIDTuner m_PIDTuner;
    private double m_previousP;
    private double m_previousI;
    private double m_previousD;

    public ElevatorNT() {
        encoderPositionPub = encoderPosition.publish();
        encoderPositionPub.setDefault(0.0);

        targetPositionPub = targetPosition.publish();
        targetPositionPub.setDefault(0.0);

        motorVoltagePub = voltage.publish();
        motorVoltagePub.setDefault(0.0);

        m_PIDTuner = new PIDTuner("elevator");

        m_PIDTuner.setP(ElevatorConstants.P_VAL);
        m_PIDTuner.setI(ElevatorConstants.I_VAL);
        m_PIDTuner.setD(ElevatorConstants.D_VAL);
        m_previousP = ElevatorConstants.P_VAL;
        m_previousI = ElevatorConstants.I_VAL;
        m_previousD = ElevatorConstants.D_VAL;
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();
        encoderPositionPub.set(getEncoderPosition(), now);
        targetPositionPub.set(getTargetPosition(), now);
        motorVoltagePub.set(getInputVoltage(), now);

        if (m_PIDTuner.isDifferentValues(m_previousP, m_previousI, m_previousD)) {
            changePID(m_PIDTuner.getP(), m_PIDTuner.getI(), m_PIDTuner.getD());
            m_previousP = m_PIDTuner.getP();
            m_previousI = m_PIDTuner.getI();
            m_previousD = m_PIDTuner.getD();
        }
    }

}
