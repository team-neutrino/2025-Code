package frc.robot;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight;
import frc.robot.util.Subsystem;

public class Dashboard extends SubsystemBase {
    private ShuffleboardTab m_driverstationTab;

    private GenericEntry m_info[] = new GenericEntry[2];
    private GenericEntry m_limelightVariables[] = new GenericEntry[2];

    private Limelight m_limelight;
    private HttpCamera LLFeed;

    public Dashboard() {

        driverstationTab();
    }

    public void driverstationTab() {
        m_driverstationTab = Shuffleboard.getTab("Driverstation Tab");

        m_info[0] = m_driverstationTab
                .add("Match Time", 0)
                .withPosition(0, 0)
                .withSize(6, 4)
                .withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("min", 0, "max", 150))
                .getEntry();

        m_info[1] = m_driverstationTab
                .add("Game Piece", "No Piece")
                .withPosition(2, 2)
                .withSize(2, 2)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .getEntry();

        LLFeed = new HttpCamera("limelight1", "http://limelight.local:5800",
                HttpCameraKind.kMJPGStreamer);
        CameraServer.startAutomaticCapture(LLFeed);
        m_driverstationTab
                .add(LLFeed)
                .withPosition(6, 0)
                .withSize(4, 3)
                .withWidget(BuiltInWidgets.kCameraStream);
    }

    @Override
    public void periodic() {
        m_info[0].setDouble(DriverStation.getMatchTime());
        m_info[1].setBoolean(Subsystem.claw.hasGamePiece());
    }
}