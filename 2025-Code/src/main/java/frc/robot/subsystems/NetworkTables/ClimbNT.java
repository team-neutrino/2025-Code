package frc.robot.subsystems.NetworkTables;

import static frc.robot.Constants.ClimbConstants.*;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Climb;
import frc.robot.util.MaxMotionTuner;
import frc.robot.util.PIDTuner;

public class ClimbNT extends Climb{
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    DoubleTopic actualMotorPosition = nt.getDoubleTopic("/climb/actual_motor_position");
    DoubleTopic followerMotorPosition = nt.getDoubleTopic("/climb/follower_motor_position");
    DoubleTopic targetMotorPosition = nt.getDoubleTopic("/climb/target_motor_position");
    DoubleTopic motorVelocity = nt.getDoubleTopic("/climb/motor_velocity");
    DoubleTopic followerVelocity = nt.getDoubleTopic("/climb/follower_velocity");

    DoubleTopic lockMotorVelocity = nt.getDoubleTopic("/climb/lock_motor_velocity");

    final DoublePublisher actualMotorPositionPub;
    final DoublePublisher followerMotorPositionPub;
    final DoublePublisher targetMotorPositionPub;
    final DoublePublisher motorVelocityPub;
    final DoublePublisher followerVelocityPub;

    final DoublePublisher lockMotorVelocityPub;

    private PIDTuner m_PIDTuner;
    private double m_previousP = kP;
    private double m_previousI = kI;
    private double m_previousD = kD;

    private MaxMotionTuner m_MaxMotionTuner;
    private double m_previousMaxVelocity = MAX_VELOCITY;
    private double m_previousMaxAcceleration = MAX_ACCELERATION;
    private double m_previousAllowedError = MAX_JERK;

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

        lockMotorVelocityPub = lockMotorVelocity.publish();
        lockMotorVelocityPub.setDefault(0.0);

        m_PIDTuner = new PIDTuner("climb/{tuning}PID");
        m_PIDTuner.setP(m_previousP);
        m_PIDTuner.setI(m_previousI);
        m_PIDTuner.setD(m_previousD);

        m_MaxMotionTuner = new MaxMotionTuner("climb/{tuning}MaxMotion");
        m_MaxMotionTuner.setMaxVelocity(m_previousMaxVelocity);
        m_MaxMotionTuner.setMaxAcceleration(m_previousMaxAcceleration);
        m_MaxMotionTuner.setAllowedError(m_previousAllowedError);
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

        lockMotorVelocityPub.set(getLockMotorVelocity(), now);

        if (m_PIDTuner.isDifferentValues(m_previousP, m_previousI, m_previousD)) {
            changePID(m_PIDTuner.getP(), m_PIDTuner.getI(), m_PIDTuner.getD());
            m_previousP = m_PIDTuner.getP();
            m_previousI = m_PIDTuner.getI();
            m_previousD = m_PIDTuner.getD();
        }

        if (m_MaxMotionTuner.isDifferentValues(m_previousMaxVelocity, m_previousMaxAcceleration, m_previousAllowedError)) {
            changeMaxMotion(m_MaxMotionTuner.getMaxVelocity(), m_MaxMotionTuner.getMaxAcceleration(), m_MaxMotionTuner.getAllowedError());
            m_previousMaxVelocity = m_MaxMotionTuner.getMaxVelocity();
            m_previousMaxAcceleration = m_MaxMotionTuner.getMaxAcceleration();
            m_previousAllowedError = m_MaxMotionTuner.getAllowedError();
        }
    }
}
