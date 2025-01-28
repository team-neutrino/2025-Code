package frc.robot.subsystems.NetworkTables;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.FFTuner;
import frc.robot.util.MaxMotionTuner;
import frc.robot.util.PIDTuner;

public class ElevatorNT extends Elevator {

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic encoderPosition = nt.getDoubleTopic("/elevator/encoder_position");
    DoubleTopic targetPosition = nt.getDoubleTopic("/elevator/target_position");
    DoubleTopic voltage = nt.getDoubleTopic("/elevator/motor_input_voltage");
    final DoublePublisher encoderPositionPub;
    final DoublePublisher targetPositionPub;
    final DoublePublisher motorVoltagePub;
    private PIDTuner m_PIDTuner;
    private double m_previousP = ElevatorConstants.P_VAL;
    private double m_previousI = ElevatorConstants.I_VAL;
    private double m_previousD = ElevatorConstants.D_VAL;
    private FFTuner m_FFTuner;
    private double m_previousFF;
    private MaxMotionTuner m_MaxMotionTuner;
    private double m_previousMaxVelocity = ArmConstants.MAX_VELOCITY;
    private double m_previousMaxAcceleration = ArmConstants.MAX_ACCELERATION;
    private double m_previousAllowedError = ArmConstants.ALLOWED_ERROR;

    public ElevatorNT() {
        encoderPositionPub = encoderPosition.publish();
        encoderPositionPub.setDefault(0.0);

        targetPositionPub = targetPosition.publish();
        targetPositionPub.setDefault(0.0);

        motorVoltagePub = voltage.publish();
        motorVoltagePub.setDefault(0.0);

        m_PIDTuner = new PIDTuner("elevator");

        m_PIDTuner.setP(m_previousP);
        m_PIDTuner.setI(m_previousI);
        m_PIDTuner.setD(m_previousD);

        m_FFTuner = new FFTuner("elevator");
        m_FFTuner.setFF(ElevatorConstants.FF_VAL);
        m_previousFF = ElevatorConstants.FF_VAL;

        m_MaxMotionTuner = new MaxMotionTuner("elevator");

        m_MaxMotionTuner.setMaxVelocity(m_previousMaxVelocity);
        m_MaxMotionTuner.setMaxAcceleration(m_previousMaxAcceleration);
        m_MaxMotionTuner.setAllowedError(m_previousAllowedError);
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

        if (m_FFTuner.getFF() != m_previousFF) {
            changeFF(m_FFTuner.getFF());
            m_previousFF = m_FFTuner.getFF();
        }

        if (m_MaxMotionTuner.isDifferentValues(m_previousMaxVelocity, m_previousMaxAcceleration,
                m_previousAllowedError)) {
            changeMaxMotion(m_MaxMotionTuner.getMaxVelocity(), m_MaxMotionTuner.getMaxAcceleration(),
                    m_MaxMotionTuner.getAllowedError());
            m_previousMaxVelocity = m_MaxMotionTuner.getMaxVelocity();
            m_previousMaxAcceleration = m_MaxMotionTuner.getMaxAcceleration();
            m_previousAllowedError = m_MaxMotionTuner.getAllowedError();
        }
    }

}
