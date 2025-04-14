package frc.robot.subsystems.NetworkTables;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.FFTuner;
import frc.robot.util.MaxMotionTuner;
import frc.robot.util.PIDTuner;

public class ElevatorNT extends Elevator {

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic encoderVelocity = nt.getDoubleTopic("/elevator/encoder_velocity");
    DoubleTopic encoderPosition = nt.getDoubleTopic("/elevator/encoder_position");
    DoubleTopic targetPosition = nt.getDoubleTopic("/elevator/target_position");
    BooleanTopic at_limit = nt.getBooleanTopic("/elevator/at_limit");
    BooleanTopic scoreReady = nt.getBooleanTopic("/elevator/score_ready");
    DoubleTopic busVoltage = nt.getDoubleTopic("/elevator/bus_voltage");
    DoubleTopic outputCurrent = nt.getDoubleTopic("/elevator/output_current");
    IntegerTopic rawBits = nt.getIntegerTopic("/elevator/raw_bits_fault");
    BooleanTopic gateDriver = nt.getBooleanTopic("/elevator/gate_driver_fault");
    final DoublePublisher encoderVelocityPub;
    final DoublePublisher encoderPositionPub;
    final DoublePublisher targetPositionPub;
    final BooleanPublisher lowLimitPub;
    final BooleanPublisher scoreReadyPub;
    final DoublePublisher busVoltagePub;
    final DoublePublisher outputCurrentPub;
    final IntegerPublisher rawBitsPub;
    final BooleanPublisher gateDriverPub;
    private PIDTuner m_PIDTuner;
    private double m_previousP = ElevatorConstants.P_VAL;
    private double m_previousI = ElevatorConstants.I_VAL;
    private double m_previousD = ElevatorConstants.D_VAL;
    private FFTuner m_FFTuner1;
    private FFTuner m_FFTuner2;
    private double m_previousFF1 = ElevatorConstants.STAGE_1_FF;
    private double m_previousFF2 = ElevatorConstants.STAGE_2_FF;
    private MaxMotionTuner m_MaxMotionTuner;
    private double m_previousMaxVelocity = ElevatorConstants.MAX_VELOCITY;
    private double m_previousMaxAcceleration = ElevatorConstants.MAX_ACCELERATION;
    private double m_previousAllowedError = ElevatorConstants.ALLOWED_ERROR;

    public ElevatorNT() {
        encoderVelocityPub = encoderVelocity.publish();
        encoderVelocityPub.setDefault(0.0);

        encoderPositionPub = encoderPosition.publish();
        encoderPositionPub.setDefault(0.0);

        targetPositionPub = targetPosition.publish();
        targetPositionPub.setDefault(0.0);

        lowLimitPub = at_limit.publish();
        lowLimitPub.setDefault(false);

        scoreReadyPub = scoreReady.publish();
        scoreReadyPub.setDefault(false);

        busVoltagePub = busVoltage.publish();
        busVoltagePub.setDefault(0);

        outputCurrentPub = outputCurrent.publish();
        outputCurrentPub.setDefault(0);

        rawBitsPub = rawBits.publish();
        rawBitsPub.setDefault(0);

        gateDriverPub = gateDriver.publish();
        gateDriverPub.setDefault(false);

        m_PIDTuner = new PIDTuner("elevator/{tuning}PID");

        m_PIDTuner.setP(m_previousP);
        m_PIDTuner.setI(m_previousI);
        m_PIDTuner.setD(m_previousD);

        m_FFTuner1 = new FFTuner("elevator/{tuning}PIDF1");
        m_FFTuner1.setFF(m_previousFF1);

        m_FFTuner2 = new FFTuner("elevator/{tuning}PIDF2");
        m_FFTuner2.setFF(m_previousFF2);

        m_MaxMotionTuner = new MaxMotionTuner("elevator/{tuning}MaxMotion");

        m_MaxMotionTuner.setMaxVelocity(m_previousMaxVelocity);
        m_MaxMotionTuner.setMaxAcceleration(m_previousMaxAcceleration);
        m_MaxMotionTuner.setAllowedError(m_previousAllowedError);
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();
        encoderVelocityPub.set(getVelocity(), now);
        encoderPositionPub.set(getHeight(), now);
        targetPositionPub.set(getSafeHeight(), now);
        lowLimitPub.set(isAtBottom(), now);
        scoreReadyPub.set(readyToScore(), now);
        busVoltagePub.set(getBusVoltage(), now);
        outputCurrentPub.set(getOutputCurrent(), now);
        rawBitsPub.set(getFaultRawBits(), now);
        gateDriverPub.set(getGateDriver(), now);

        if (m_PIDTuner.isDifferentValues(m_previousP, m_previousI, m_previousD)) {
            changePID(m_PIDTuner.getP(), m_PIDTuner.getI(), m_PIDTuner.getD());
            m_previousP = m_PIDTuner.getP();
            m_previousI = m_PIDTuner.getI();
            m_previousD = m_PIDTuner.getD();
        }

        if (m_FFTuner1.getFF() != m_previousFF1) {
            changeFF1(m_FFTuner1.getFF());
            m_previousFF1 = m_FFTuner1.getFF();
        }

        if (m_FFTuner2.getFF() != m_previousFF2) {
            changeFF2(m_FFTuner2.getFF());
            m_previousFF2 = m_FFTuner2.getFF();
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
