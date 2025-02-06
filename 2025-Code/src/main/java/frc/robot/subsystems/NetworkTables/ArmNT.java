package frc.robot.subsystems.NetworkTables;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.util.FFTuner;
import frc.robot.util.MaxMotionTuner;
import frc.robot.util.PIDTuner;

public class ArmNT extends Arm {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic encoderPosition = nt.getDoubleTopic("/arm/encoder_position");
    DoubleTopic targetPosition = nt.getDoubleTopic("/arm/target_position");
    DoubleTopic voltage = nt.getDoubleTopic("/arm/motor_input_voltage");
    DoubleTopic encoderVelocity = nt.getDoubleTopic("/arm/encoder_velocity");
    final DoublePublisher encoderPositionPub;
    final DoublePublisher targetPositionPub;
    final DoublePublisher motorVoltagePub;
    final DoublePublisher encoderVelocityPub;
    private PIDTuner m_PIDTuner;
    private double m_previousP = ArmConstants.kp;
    private double m_previousI = ArmConstants.ki;
    private double m_previousD = ArmConstants.kd;
    private FFTuner m_FFTuner;
    private double m_previousFF = ArmConstants.FFCONSTANT;
    private MaxMotionTuner m_MaxMotionTuner;
    private double m_previousMaxAcceleration = ArmConstants.MAX_ACCELERATION;
    private double m_previousMaxVelocity = ArmConstants.MAX_VELOCITY;
    private double m_previousAllowedError = ArmConstants.ALLOWED_ERROR;

    public ArmNT() {
        encoderPositionPub = encoderPosition.publish();
        encoderPositionPub.setDefault(0.0);

        targetPositionPub = targetPosition.publish();
        targetPositionPub.setDefault(0.0);

        encoderVelocityPub = encoderVelocity.publish();
        encoderVelocityPub.setDefault(0.0);

        motorVoltagePub = voltage.publish();
        motorVoltagePub.setDefault(0.0);
        m_PIDTuner = new PIDTuner("arm/{tuning}PIDF");

        m_PIDTuner.setP(m_previousP);
        m_PIDTuner.setI(m_previousI);
        m_PIDTuner.setD(m_previousD);

        m_FFTuner = new FFTuner("arm/{tuning}PIDF");

        m_FFTuner.setFF(m_previousFF);

        m_MaxMotionTuner = new MaxMotionTuner("arm/{tuning}MaxMotion");

        m_MaxMotionTuner.setMaxVelocity(m_previousMaxVelocity);
        m_MaxMotionTuner.setMaxAcceleration(m_previousMaxAcceleration);
        m_MaxMotionTuner.setAllowedError(m_previousAllowedError);
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();
        encoderPositionPub.set(getArmEncoderPosition(), now);
        targetPositionPub.set(getArmTargetPosition(), now);
        motorVoltagePub.set(getArmVoltage(), now);
        encoderVelocityPub.set(getArmEncoderVelocity(), now);

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
