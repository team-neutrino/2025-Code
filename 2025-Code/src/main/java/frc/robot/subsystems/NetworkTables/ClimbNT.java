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
    DoubleTopic climbCurrent = nt.getDoubleTopic("/climb/climb_velocity");
    DoubleTopic followerCurrent = nt.getDoubleTopic("/climb/follower_climb_velocity");
    BooleanTopic motorOff = nt.getBooleanTopic("/climb/motor_off");

    final DoublePublisher actualClimbPositionPub;
    final DoublePublisher followerClimbPositionPub;
    final DoublePublisher targetClimbPositionPub;
    final DoublePublisher climbCurrentPub;
    final DoublePublisher followerCurrentPub;
    final BooleanPublisher motorOffPub;

    private PIDTuner m_PIDTuner;
    private double m_previousClimbP = CLIMB_kP;
    private double m_previousClimbI = CLIMB_kI;
    private double m_previousClimbD = CLIMB_kD;

    public ClimbNT() {
        actualClimbPositionPub = actualClimbPosition.publish();
        actualClimbPositionPub.setDefault(0.0);

        followerClimbPositionPub = followerClimbPosition.publish();
        followerClimbPositionPub.setDefault(0.0);

        targetClimbPositionPub = targetClimbPosition.publish();
        targetClimbPositionPub.setDefault(0.0);

        climbCurrentPub = climbCurrent.publish();
        climbCurrentPub.setDefault(0.0);

        followerCurrentPub = followerCurrent.publish();
        followerCurrentPub.setDefault(0.0);

        motorOffPub = motorOff.publish();
        motorOffPub.setDefault(false);

        m_PIDTuner = new PIDTuner("climb/{tuning}PID");
        m_PIDTuner.setP(m_previousClimbP);
        m_PIDTuner.setI(m_previousClimbI);
        m_PIDTuner.setD(m_previousClimbD);
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();

        actualClimbPositionPub.set(getMotorPosition(), now);
        followerClimbPositionPub.set(getFollowerPosition(), now);
        targetClimbPositionPub.set(getTargetPosition(), now);

        climbCurrentPub.set(getClimbMotorCurrent(), now);
        followerCurrentPub.set(getClimbFollowerCurrent(), now);

        motorOffPub.set(getMotoroff(), now);

        if (m_PIDTuner.isDifferentValues(m_previousClimbP, m_previousClimbI, m_previousClimbD)) {
            changePID(m_PIDTuner.getP(), m_PIDTuner.getI(), m_PIDTuner.getD());
            m_previousClimbP = m_PIDTuner.getP();
            m_previousClimbI = m_PIDTuner.getI();
            m_previousClimbD = m_PIDTuner.getD();
        }
    }
}
