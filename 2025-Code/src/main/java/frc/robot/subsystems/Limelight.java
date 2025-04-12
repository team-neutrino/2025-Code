// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.util.Subsystem;

import static frc.robot.Constants.DriveToPoint.ALGAE_ALIGN_COMMAND;
import static frc.robot.Constants.LimelightConstants.*;

public class Limelight extends SubsystemBase {
  LimelightHelpers m_limelightHelpers;
  double m_robotYaw;
  Swerve m_swerve;
  Rotation2d m_targetYaw;
  private double m_lastFrameReef1 = -2;
  private double m_lastFrameReef2 = -2;
  private double m_lastFrameStation = -2;
  private boolean m_has_reef1_tag;
  private boolean m_has_reef2_tag;
  private boolean m_has_station_tag;
  private boolean m_enabled = false;
  private long m_slow_count = 0;
  private boolean m_deAlgaefying = false;

  /** Creates a new ExampleSubsystem. */
  public Limelight() {
    m_swerve = Subsystem.swerve;
    m_limelightHelpers = new LimelightHelpers();
    // fake pipeline number
    // LimelightHelpers.setPipelineIndex(LIMELIGHT_1, 1);
    LimelightHelpers.setLEDMode_ForceOff(LL_REEF1);
    LimelightHelpers.setCameraPose_RobotSpace(LL_REEF1,
        CAMERA_FORWARD_OFFSET, // Forward offset (meters)
        CAMERA_SIDE_OFFSET, // Side offset (meters) left is positive
        CAMERA_HEIGHT_OFFSET, // Height offset (meters)
        CAMERA_ROLL_OFFSET, // Roll (degrfees)
        CAMERA_PITCH_OFFSET, // Pitch (degrees)
        CAMERA_YAW_OFFSET // Yaw (degrees)
    );
    LimelightHelpers.SetFiducialDownscalingOverride(LL_REEF1, 3);

    LimelightHelpers.setLEDMode_ForceOff(LL_REEF2);
    LimelightHelpers.setCameraPose_RobotSpace(LL_REEF2,
        CAMERA2_FORWARD_OFFSET, // Forward offset (meters)
        CAMERA2_SIDE_OFFSET, // Side offset (meters) left is positive
        CAMERA2_HEIGHT_OFFSET, // Height offset (meters)
        CAMERA2_ROLL_OFFSET, // Roll (degrees)
        CAMERA2_PITCH_OFFSET, // Pitch (degrees)
        CAMERA2_YAW_OFFSET // Yaw (degrees)
    );
    LimelightHelpers.SetFiducialDownscalingOverride(LL_REEF2, 3);

    LimelightHelpers.setLEDMode_ForceOff(LL_STATION);
    LimelightHelpers.setCameraPose_RobotSpace(LL_STATION,
        CAMERA_STATION_FORWARD_OFFSET, // Forward offset (meters)
        CAMERA_STATION_SIDE_OFFSET, // Side offset (meters) left is positive
        CAMERA_STATION_HEIGHT_OFFSET, // Height offset (meters)
        CAMERA_STATION_ROLL_OFFSET, // Roll (degrees)
        CAMERA_STATION_PITCH_OFFSET, // Pitch (degrees)
        CAMERA_STATION_YAW_OFFSET // Yaw (degrees)
    );

    LimelightHelpers.SetIMUMode(LL_REEF1, 1);
    LimelightHelpers.SetIMUMode(LL_REEF2, 1);
    // use external IMU yaw submitted via setRobotOrientation() and configure the
    // LL4 internal IMU's fused yaw to match the submitted yaw value
  }

  // **get valid target from camera 1*/
  public boolean getTvReef1() {
    return m_has_reef1_tag;
  }

  // **get valid target from camera 2*/
  public boolean getTvReef2() {
    return m_has_reef2_tag;
  }

  // **get valid target from camera 2*/
  public boolean getTvStation() {
    return m_has_station_tag;
  }

  private void updateOdometryReef1() {
    LimelightHelpers.PoseEstimate limePoseEstReef = LimelightHelpers
        .getBotPoseEstimate_wpiBlue_MegaTag2(LL_REEF1);
    double frame = getFrame(LL_REEF1);
    if (limePoseEstReef != null && limePoseEstReef.tagCount != 0
        && m_swerve.getState().Speeds.omegaRadiansPerSecond < 4 * Math.PI
        && frame > m_lastFrameReef1) {
      m_swerve.addVisionMeasurement(limePoseEstReef.pose, limePoseEstReef.timestampSeconds);
    }
    m_lastFrameReef1 = frame;
  }

  private void updateOdometryReef2() {
    LimelightHelpers.PoseEstimate limePoseEstReef = LimelightHelpers
        .getBotPoseEstimate_wpiBlue_MegaTag2(LL_REEF2);
    double frame = getFrame(LL_REEF2);
    if (limePoseEstReef != null && limePoseEstReef.tagCount != 0
        && m_swerve.getState().Speeds.omegaRadiansPerSecond < 4 * Math.PI
        && frame > m_lastFrameReef2) {
      m_swerve.addVisionMeasurement(limePoseEstReef.pose, limePoseEstReef.timestampSeconds);
    }
    m_lastFrameReef2 = frame;
  }

  private void updateOdometryStation() {
    LimelightHelpers.PoseEstimate limePoseEstStation = LimelightHelpers
        .getBotPoseEstimate_wpiBlue_MegaTag2(LL_STATION);
    double frame = getFrame(LL_STATION);
    if (limePoseEstStation != null && limePoseEstStation.tagCount != 0
        && m_swerve.getState().Speeds.omegaRadiansPerSecond < 4 * Math.PI
        && frame > m_lastFrameStation) {
      m_swerve.addVisionMeasurement(limePoseEstStation.pose, limePoseEstStation.timestampSeconds);
    }
    m_lastFrameStation = frame;
  }

  public void setDealgaefying(boolean dealgaefying) {
    m_deAlgaefying = dealgaefying;
  }

  public double getTargetYawFromReef1() {
    if (!getTvReef1()) {
      return 0;
    }
    return LimelightHelpers.getTargetPose_RobotSpace(LL_REEF1)[4];
  }

  public double getTargetYawFromReef2() {
    if (!getTvReef2()) {
      return 0;
    }
    return LimelightHelpers.getTargetPose_RobotSpace(LL_REEF2)[4];
  }

  public double getTargetYawFromStation() {
    return LimelightHelpers.getTargetPose_RobotSpace(LL_STATION)[4];
  }

  private void updateOdometry() {
    // if aligning to an algae position, force odometry updates from reef.
    if (getTvReef1() && (m_deAlgaefying || Subsystem.coral.hasCoral() || Subsystem.algae.hasAlgae())) {
      updateOdometryReef1();
      return;
    } else if (getTvReef2() && (m_deAlgaefying || Subsystem.coral.hasCoral() || Subsystem.algae.hasAlgae())) {
      updateOdometryReef2();
      return;
    }
    if (getTvStation()) {
      updateOdometryStation();
    }
  }

  private double getFrame(String limelight) {
    return NetworkTableInstance.getDefault().getTable(limelight).getEntry("hb").getDouble(-1);
  }

  private void ManageLimelightTemperature() {
    m_slow_count++;
    if (m_enabled && (m_slow_count % 50) != 0) {
      return;
    }
    // update at 1Hz or when disabled
    m_enabled = DriverStation.isEnabled();
    final int throttle = m_enabled ? 0 : 169;
    LimelightHelpers.SetThrottle(LL_REEF1, throttle);
    LimelightHelpers.SetThrottle(LL_REEF2, throttle);
    LimelightHelpers.SetThrottle(LL_STATION, throttle);
  }

  public Command limelightDefaultCommand() {
    return run(() -> {

    });
  }

  @Override
  public void periodic() {
    ManageLimelightTemperature();
    m_has_reef1_tag = LimelightHelpers.getTV(LL_REEF1);
    m_has_reef2_tag = LimelightHelpers.getTV(LL_REEF2);
    m_has_station_tag = LimelightHelpers.getTV(LL_STATION);

    if (m_swerve == null) {
      return;
    }

    final var yaw_degrees = Subsystem.swerve.getYawDegrees();
    // according to limelight docs, this needs to be called before using
    // .getBotPoseEstimate_wpiBlue_MegaTag2
    LimelightHelpers.SetRobotOrientation(LL_REEF1,
        yaw_degrees, 0,
        0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(LL_REEF2,
        yaw_degrees, 0,
        0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(LL_STATION,
        yaw_degrees, 0,
        0, 0, 0, 0);
    updateOdometry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
