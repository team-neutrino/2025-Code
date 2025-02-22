package frc.robot.subsystems.NetworkTables;

import static frc.robot.Constants.ClimbConstants.*;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.Climb;
import frc.robot.util.PIDTuner;

public class ClimbNT extends Climb {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    DoubleTopic actualClimbPosition = nt.getDoubleTopic("/climb/actual_climb_position");
    DoubleTopic followerClimbPosition = nt.getDoubleTopic("/climb/follower_climb_position");
    DoubleTopic targetClimbPosition = nt.getDoubleTopic("/climb/target_climb_position");
    DoubleTopic climbVelocity = nt.getDoubleTopic("/climb/climb_velocity");
    DoubleTopic followerVelocity = nt.getDoubleTopic("/climb/follower_climb_velocity");
    BooleanTopic motorOff = nt.getBooleanTopic("/climb/motor_off");

    DoubleTopic grabVelocity = nt.getDoubleTopic("/climb/grab_velocity");
    DoubleTopic grabCurrent = nt.getDoubleTopic("/climb/grab_current");

    final DoublePublisher actualClimbPositionPub;
    final DoublePublisher followerClimbPositionPub;
    final DoublePublisher targetClimbPositionPub;
    final DoublePublisher climbVelocityPub;
    final DoublePublisher followerVelocityPub;
    final BooleanPublisher motorOffPub;

    final DoublePublisher grabVelocityPub;
    final DoublePublisher grabCurrentPub;

    private PIDTuner m_PIDTuner;
    private double m_previousClimbP = CLIMB_kP;
    private double m_previousClimbI = CLIMB_kI;
    private double m_previousClimbD = CLIMB_kD;

    private PIDTuner m_lockPIDTuner;
    private double m_previousLockP = LOCK_kP;
    private double m_previousLockI = LOCK_kI;
    private double m_previousLockD = LOCK_kD;

    public ClimbNT() {
        actualClimbPositionPub = actualClimbPosition.publish();
        actualClimbPositionPub.setDefault(0.0);

        followerClimbPositionPub = followerClimbPosition.publish();
        followerClimbPositionPub.setDefault(0.0);

        targetClimbPositionPub = targetClimbPosition.publish();
        targetClimbPositionPub.setDefault(0.0);

        climbVelocityPub = climbVelocity.publish();
        climbVelocityPub.setDefault(0.0);

        followerVelocityPub = followerVelocity.publish();
        followerVelocityPub.setDefault(0.0);

        motorOffPub = motorOff.publish();
        motorOffPub.setDefault(false);

        grabVelocityPub = grabVelocity.publish();
        grabVelocityPub.setDefault(0.0);

        grabCurrentPub = grabCurrent.publish();
        grabCurrentPub.setDefault(0);

        m_PIDTuner = new PIDTuner("climb/{tuning}PID");
        m_PIDTuner.setP(m_previousClimbP);
        m_PIDTuner.setI(m_previousClimbI);
        m_PIDTuner.setD(m_previousClimbD);

        m_lockPIDTuner = new PIDTuner("climb/{tuning}lockPID");
        m_lockPIDTuner.setP(m_previousLockP);
        m_lockPIDTuner.setI(m_previousLockI);
        m_lockPIDTuner.setD(m_previousLockD);
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();

        actualClimbPositionPub.set(getMotorPosition(), now);
        followerClimbPositionPub.set(getFollowerPosition(), now);

        targetClimbPositionPub.set(getTargetPosition(), now);

        climbVelocityPub.set(getMotorVelocity(), now);
        followerVelocityPub.set(getFollowerVelocity(), now);

        motorOffPub.set(getMotoroff(), now);

        grabVelocityPub.set(getLockMotorVelocity(), now);
        grabCurrentPub.set(getLockMotorCurrent(), now);

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
    }
}
