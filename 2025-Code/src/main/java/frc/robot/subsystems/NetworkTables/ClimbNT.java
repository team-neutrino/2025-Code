package frc.robot.subsystems.NetworkTables;

import static frc.robot.Constants.ClimbConstants.*;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.Climb;
import frc.robot.util.MotionMagicTuner;
import frc.robot.util.PIDTuner;

public class ClimbNT extends Climb {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    DoubleTopic actualMotorPosition = nt.getDoubleTopic("/climb/actual_motor_position");
    DoubleTopic followerMotorPosition = nt.getDoubleTopic("/climb/follower_motor_position");
    DoubleTopic targetMotorPosition = nt.getDoubleTopic("/climb/target_motor_position");
    DoubleTopic motorVelocity = nt.getDoubleTopic("/climb/motor_velocity");
    DoubleTopic followerVelocity = nt.getDoubleTopic("/climb/follower_velocity");
    BooleanTopic motorStatus = nt.getBooleanTopic("/climb/motor_status");

    DoubleTopic lockMotorVelocity = nt.getDoubleTopic("/climb/lock_motor_velocity");
    DoubleTopic lockMotorCurrent = nt.getDoubleTopic("/climb/lock_motor_current");

    final DoublePublisher actualMotorPositionPub;
    final DoublePublisher followerMotorPositionPub;
    final DoublePublisher targetMotorPositionPub;
    final DoublePublisher motorVelocityPub;
    final DoublePublisher followerVelocityPub;
    final BooleanPublisher motorStatusPub;

    final DoublePublisher lockMotorVelocityPub;
    final DoublePublisher lockMotorCurrentPub;

    private PIDTuner m_PIDTuner;
    private double m_previousClimbP = CLIMB_kP;
    private double m_previousClimbI = CLIMB_kI;
    private double m_previousClimbD = CLIMB_kD;

    private PIDTuner m_lockPIDTuner;
    private double m_previousLockP = LOCK_kP;
    private double m_previousLockI = LOCK_kI;
    private double m_previousLockD = LOCK_kD;

    private MotionMagicTuner m_motionMagicTuner;
    private double m_previousVelocity = VELOCITY;
    private double m_previousAcceleration = ACCELERATION;
    private double m_previousJerk = JERK;

    public ClimbNT() {
        actualMotorPositionPub = actualMotorPosition.publish();
        actualMotorPositionPub.setDefault(0.0);

        followerMotorPositionPub = followerMotorPosition.publish();
        followerMotorPositionPub.setDefault(0.0);

        targetMotorPositionPub = targetMotorPosition.publish();
        targetMotorPositionPub.setDefault(0.0);

        motorVelocityPub = motorVelocity.publish();
        motorVelocityPub.setDefault(0.0);

        followerVelocityPub = followerVelocity.publish();
        followerVelocityPub.setDefault(0.0);

        motorStatusPub = motorStatus.publish();
        motorStatusPub.setDefault(false);

        lockMotorVelocityPub = lockMotorVelocity.publish();
        lockMotorVelocityPub.setDefault(0.0);

        lockMotorCurrentPub = lockMotorCurrent.publish();
        lockMotorCurrentPub.setDefault(0);

        m_PIDTuner = new PIDTuner("climb/{tuning}PID");
        m_PIDTuner.setP(m_previousClimbP);
        m_PIDTuner.setI(m_previousClimbI);
        m_PIDTuner.setD(m_previousClimbD);

        m_lockPIDTuner = new PIDTuner("climb/{tuning}lockPID");
        m_lockPIDTuner.setP(m_previousLockP);
        m_lockPIDTuner.setI(m_previousLockI);
        m_lockPIDTuner.setD(m_previousLockD);

        m_motionMagicTuner = new MotionMagicTuner("climb/{tuning}MotionMagic");
        m_motionMagicTuner.setVelocity(m_previousVelocity);
        m_motionMagicTuner.setAcceleration(m_previousAcceleration);
        m_motionMagicTuner.setJerk(m_previousJerk);
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();

        actualMotorPositionPub.set(getMotorPosition(), now);
        followerMotorPositionPub.set(getFollowerPosition(), now);

        targetMotorPositionPub.set(getTargetPosition(), now);

        motorVelocityPub.set(getMotorVelocity(), now);
        followerVelocityPub.set(getFollowerVelocity(), now);

        motorStatusPub.set(getMotorStatus(), now);

        lockMotorVelocityPub.set(getLockMotorVelocity(), now);
        lockMotorCurrentPub.set(getLockMotorCurrent(), now);

        if (m_PIDTuner.isDifferentValues(m_previousClimbP, m_previousClimbI, m_previousClimbD)) {
            changePID(m_PIDTuner.getP(), m_PIDTuner.getI(), m_PIDTuner.getD());
            m_previousClimbP = m_PIDTuner.getP();
            m_previousClimbI = m_PIDTuner.getI();
            m_previousClimbD = m_PIDTuner.getD();
        }

        if (m_lockPIDTuner.isDifferentValues(m_previousLockP, m_previousLockI, m_previousLockD)) {
            changePID(m_lockPIDTuner.getP(), m_lockPIDTuner.getI(), m_lockPIDTuner.getD());
            m_previousLockP = m_lockPIDTuner.getP();
            m_previousLockI = m_lockPIDTuner.getI();
            m_previousLockD = m_lockPIDTuner.getD();
        }

        if (m_motionMagicTuner.isDifferentValues(m_previousVelocity, m_previousAcceleration, m_previousJerk)) {
            changeMotionMagic(m_motionMagicTuner.getVelocity(), m_motionMagicTuner.getAcceleration(),
                    m_motionMagicTuner.getJerk());
            m_previousVelocity = m_motionMagicTuner.getVelocity();
            m_previousAcceleration = m_motionMagicTuner.getAcceleration();
            m_previousJerk = m_motionMagicTuner.getJerk();
        }
    }
}
