package frc.robot.subsystems.NetworkTables;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.util.FFTuner;
import frc.robot.util.PIDTuner;

public class ArmNT extends Arm {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic encoderPosition = nt.getDoubleTopic("/arm/encoder_position");
    DoubleTopic targetPosition = nt.getDoubleTopic("/arm/target_position");
    DoubleTopic voltage = nt.getDoubleTopic("/arm/motor_input_voltage");
    final DoublePublisher encoderPositionPub;
    final DoublePublisher targetPositionPub;
    final DoublePublisher motorVoltagePub;
    PIDTuner m_PIDTuner;
    private double m_previousP;
    private double m_previousI;
    private double m_previousD;
    FFTuner m_FFTuner;

    public ArmNT() {
        encoderPositionPub = encoderPosition.publish();
        encoderPositionPub.setDefault(0.0);

        targetPositionPub = targetPosition.publish();
        targetPositionPub.setDefault(0.0);

        motorVoltagePub = voltage.publish();
        motorVoltagePub.setDefault(0.0);
        m_PIDTuner = new PIDTuner("arm");

        m_PIDTuner.setP(ArmConstants.kp);
        m_PIDTuner.setI(ArmConstants.ki);
        m_PIDTuner.setD(ArmConstants.kd);
        m_previousP = ArmConstants.kp;
        m_previousI = ArmConstants.ki;
        m_previousD = ArmConstants.kd;

        m_FFTuner = new FFTuner("arm");
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();
        encoderPositionPub.set(getArmEncoderPosition(), now);
        targetPositionPub.set(getArmTargetPosition(), now);
        motorVoltagePub.set(getArmVoltage(), now);

        if (m_PIDTuner.isDifferentValues(m_previousP, m_previousI, m_previousD)) {
            changePID(m_PIDTuner.getP(), m_PIDTuner.getI(), m_PIDTuner.getD());
            m_previousP = m_PIDTuner.getP();
            m_previousI = m_PIDTuner.getI();
            m_previousD = m_PIDTuner.getD();
        }
    }
}
