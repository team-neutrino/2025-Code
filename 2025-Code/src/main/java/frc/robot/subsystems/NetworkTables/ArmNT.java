package frc.robot.subsystems.NetworkTables;

import static frc.robot.Constants.ArmConstants.kd;
import static frc.robot.Constants.ArmConstants.ki;
import static frc.robot.Constants.ArmConstants.kp;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
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
    private double previousP;
    private double previousI;
    private double previousD;

    public ArmNT() {
        encoderPositionPub = encoderPosition.publish();
        encoderPositionPub.setDefault(0.0);

        targetPositionPub = targetPosition.publish();
        targetPositionPub.setDefault(0.0);

        motorVoltagePub = voltage.publish();
        motorVoltagePub.setDefault(0.0);
        m_PIDTuner = new PIDTuner("arm");

        m_PIDTuner.setP(kp);
        m_PIDTuner.setI(ki);
        m_PIDTuner.setD(kd);
        previousP = kp;
        previousI = ki;
        previousD = kd;
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();
        encoderPositionPub.set(getArmEncoderPosition(), now);
        targetPositionPub.set(getArmTargetPosition(), now);
        motorVoltagePub.set(getArmVoltage(), now);

        if (m_PIDTuner.getP() != previousP || m_PIDTuner.getI() != previousI || m_PIDTuner.getD() != previousD) {
            changePID(m_PIDTuner.getP(), m_PIDTuner.getI(), m_PIDTuner.getD());
            previousP = m_PIDTuner.getP();
            previousI = m_PIDTuner.getI();
            previousD = m_PIDTuner.getD();
        }
    }

}
