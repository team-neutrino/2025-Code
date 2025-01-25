package frc.robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.util.PDH;

public class NetworkTables {
    PDH m_Pdh = new PDH();
    // fms for match time: wpilib.org -- getMatchTime()
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    DoubleTopic total_current = nt.getDoubleTopic("/PDH/total_current"); // of battery
    final DoublePublisher totalCurrent;

    public NetworkTables() {
        totalCurrent = total_current.publish();
        totalCurrent.setDefault(0.0);
    }

    public void periodic() {
        final long now = NetworkTablesJNI.now();
        totalCurrent.set(m_Pdh.getTotalCurrent(), now);
    }

}
