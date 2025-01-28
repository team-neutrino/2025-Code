package frc.robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.PDH;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NetworkTables extends SubsystemBase {
    PDH m_Pdh = new PDH();
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    DoubleTopic total_current = nt.getDoubleTopic("/PDH/total_current");// battery
    DoubleTopic match_time = nt.getDoubleTopic("/FMS/match_time");

    final DoublePublisher totalCurrent;
    final DoublePublisher matchTime;

    public NetworkTables() {
        totalCurrent = total_current.publish();
        totalCurrent.setDefault(0.0);

        matchTime = match_time.publish();
        matchTime.setDefault(0.0);
    }

    public void periodic() {
        final long now = NetworkTablesJNI.now();
        totalCurrent.set(m_Pdh.getTotalCurrent(), now);
        matchTime.set(DriverStation.getMatchTime(), now);
    }

}
