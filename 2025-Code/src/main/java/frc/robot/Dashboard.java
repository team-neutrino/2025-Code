package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dashboard extends SubsystemBase {
    private ShuffleboardTab m_driverstationTab;

    // nt
    // private NetworkTables nt = new NetworkTables();

    private GenericEntry m_info[] = new GenericEntry[1];

    public Dashboard() {

        m_driverstationTab = Shuffleboard.getTab("Driverstation Tab");

        driverstationTab();
    }

    public void driverstationTab() {
        m_driverstationTab = Shuffleboard.getTab("Driverstation Tab");

        m_info[0] = m_driverstationTab.add("Match Time", 0)
                .withPosition(4, 0)
                .withSize(1, 1).getEntry();
    }

    @Override
    public void periodic() {
        m_info[0].setDouble(DriverStation.getMatchTime());
    }
}