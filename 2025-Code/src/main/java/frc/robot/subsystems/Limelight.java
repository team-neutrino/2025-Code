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
import frc.robot.util.Subsystem;

import static frc.robot.Constants.DriveToPoint.ALGAE_ALIGN_COMMAND;
import static frc.robot.Constants.LimelightConstants.*;

public class Limelight extends SubsystemBase {
  LimelightHelpers m_limelightHelpers;
  double m_robotYaw;
  Swerve m_swerve;
  Rotation2d m_targetYaw;
  private double[] pose = new double[11];
  private double[] targetPose = new double[6];
  private double[] targetPose2 = new double[6];
  private double m_lastFrameReef1 = -2;
  private double m_lastFrameReef2 = -2;
  private double m_lastFrameStation = -2;
  private boolean m_has_reef_tag;
  private boolean m_has_station_tag;
  private boolean m_enabled = false;
  private long m_slow_count = 0;

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
        CAMERA_ROLL_OFFSET, // Roll (degrees)
        CAMERA_PITCH_OFFSET, // Pitch (degrees)
        CAMERA_YAW_OFFSET // Yaw (degrees)
    );
    LimelightHelpers.SetFiducialDownscalingOverride(LL_REEF1, 3);

    // change name later
    LimelightHelpers.setLEDMode_ForceOff(LL_STATION);
    LimelightHelpers.setCameraPose_RobotSpace(LL_STATION,
        CAMERA_STATION_FORWARD_OFFSET, // Forward offset (meters)
        CAMERA_STATION_SIDE_OFFSET, // Side offset (meters) left is positive
        CAMERA_STATION_HEIGHT_OFFSET, // Height offset (meters)
        CAMERA_STATION_ROLL_OFFSET, // Roll (degrees)
        CAMERA_STATION_PITCH_OFFSET, // Pitch (degrees)
        CAMERA_STATION_YAW_OFFSET // Yaw (degrees)
    );

    LimelightHelpers.setLEDMode_ForceOff(LL_REEF2);
    LimelightHelpers.setCameraPose_RobotSpace(LL_REEF2,
        CAMERA2_FORWARD_OFFSET, // Forward offset (meters)
        CAMERA2_SIDE_OFFSET, // Side offset (meters) left is positive
        CAMERA2_HEIGHT_OFFSET, // Height offset (meters)
        CAMERA2_ROLL_OFFSET, // Roll (degrees)
        CAMERA2_PITCH_OFFSET, // Pitch (degrees)
        CAMERA2_YAW_OFFSET // Yaw (degrees)
    );

    LimelightHelpers.SetIMUMode(LL_REEF1, 1);
    // use external IMU yaw submitted via setRobotOrientation() and configure the
    // LL4 internal IMU's fused yaw to match the submitted yaw value
    LimelightHelpers.SetIMUMode(LL_STATION, 1);
  }

  // **get valid target from camera 1*/
  public boolean getTvReef() {
    return m_has_reef_tag;
  }

  // **get valid target from camera 2*/
  public boolean getTvStation() {
    return m_has_station_tag;
  }

  /**
   * get Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27
   * degrees / LL2: -29.8 to 29.8 degrees)
   */
  public double getTxReef() {
    return LimelightHelpers.getTX(LL_REEF1);
  }

  /**
   * get Horizontal Offset From Second Camera Crosshair To Target (LL1: -27
   * degrees to 27
   * degrees / LL2: -29.8 to 29.8 degrees)
   */
  public double getTxStation() {
    return LimelightHelpers.getTX(LL_STATION);
  }

  /**
   * get Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5
   * degrees / LL2: -24.85 to 24.85 degrees)
   */
  public double getTyReef() {
    return LimelightHelpers.getTY(LL_REEF1);
  }

  /**
   * get Vertical Offset From Second Camera Crosshair To Target (LL1: -20.5
   * degrees to 20.5
   * degrees / LL2: -24.85 to 24.85 degrees)
   */
  public double getTyStation() {
    return LimelightHelpers.getTY(LL_STATION);
  }

  /** get ID of the primary in-view AprilTag */
  public int getIDReef() {
    return (int) LimelightHelpers.getFiducialID(LL_REEF1);
  }

  /** get ID of the primary in-view AprilTag from the Second Camera */
  public int getIDStation() {
    return (int) LimelightHelpers.getFiducialID(LL_STATION);
  }

  public double[] getTargetPoseReef() {
    if (getTvReef()) {
      targetPose = LimelightHelpers.getTargetPose_RobotSpace(LL_REEF1);
    }
    return targetPose;
  }

  public double[] getTargetPoseStation() {
    if (getTvStation()) {
      targetPose2 = LimelightHelpers.getTargetPose_RobotSpace(LL_STATION);
    }
    return targetPose2;
  }

  public double getTargetYawReef() {
    getTargetPoseReef();
    return targetPose[5];
  }

  public double getTargetYawStation() {
    getTargetPoseStation();
    return targetPose2[5];
  }

  public Rotation2d getTargetYawRotation2dReef() {
    getTargetPoseReef();
    return Rotation2d.fromDegrees(targetPose[5]);
  }

  public Rotation2d getTargetYawRotation2dStation() {
    getTargetPoseStation();
    return Rotation2d.fromDegrees(targetPose2[5]);
  }

  public double[] getBotPose() {
    // depending on how we do want to do our vision we could have regular getBotPose
    if (getTvReef()) {
      pose = LimelightHelpers.getBotPose_wpiBlue(LL_REEF1);
    } else if (getTvStation()) {
      pose = LimelightHelpers.getBotPose_wpiBlue(LL_STATION);
    }
    return pose;
    // currently defaults to 0 if there's no pose
  }

  public double getDistanceFromPrimaryTarget() {
    return getBotPose()[9];
    // based on camera not robot
  }

  public void setPointOfInterest(double x, double y) {
    LimelightHelpers.setFiducial3DOffset("limelight", x, y, 0);
  }

  public void setPriorityIDReef(int id) {
    LimelightHelpers.setPriorityTagID(LL_REEF1, id);
  }

  public void setPriorityIDStation(int id) {
    LimelightHelpers.setPriorityTagID(LL_STATION, id);
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

  private void updateOdometry() {
    Command com = Subsystem.swerve.getCurrentCommand();
    boolean deAlgaefying = false;
    if (com != null) {
      deAlgaefying = com.getName().equals(ALGAE_ALIGN_COMMAND);
    }

    // if aligning to an algae position, force odometry updates from reef.
    if (getTvReef() && (deAlgaefying || Subsystem.coral.hasCoral())) {
      updateOdometryReef1();
      return;
    } else if (LimelightHelpers.getTV(LL_REEF2) && (deAlgaefying || Subsystem.coral.hasCoral())
        && !DriverStation.isAutonomousEnabled()) {
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
    m_has_reef_tag = LimelightHelpers.getTV(LL_REEF1);
    m_has_station_tag = LimelightHelpers.getTV(LL_STATION);

    if (m_swerve == null) {
      return;
    }

    // according to limelight docs, this needs to be called before using
    // .getBotPoseEstimate_wpiBlue_MegaTag2
    LimelightHelpers.SetRobotOrientation(LL_REEF1,
        Subsystem.swerve.getYawDegrees(), 0,
        0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(LL_STATION,
        Subsystem.swerve.getYawDegrees(), 0,
        0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(LL_REEF2,
        Subsystem.swerve.getYawDegrees(), 0,
        0, 0, 0, 0);
    updateOdometry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
