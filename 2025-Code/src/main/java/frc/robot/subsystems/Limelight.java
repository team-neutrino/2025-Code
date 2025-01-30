// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import static frc.robot.Constants.LimelightConstants.*;
import frc.robot.Constants.SwerveConstants;

public class Limelight extends SubsystemBase {
  LimelightHelpers m_limelightHelpers;
  SwerveDrivePoseEstimator m_poseEstimator;
  double m_robotYaw;
  Swerve m_swerve;
  Rotation2d m_targetYaw;
  LimelightHelpers.PoseEstimate m_limelightMeasurement;
  private double[] pose = new double[11];
  private double[] targetPose = new double[6];
  private double[] targetPose2 = new double[6];

  /** Creates a new ExampleSubsystem. */
  public Limelight() {
    // m_swerve = Subsystem.swerve;
    m_limelightHelpers = new LimelightHelpers();
    // fake pipeline number
    // LimelightHelpers.setPipelineIndex(LIMELIGHT_1, 1);
    LimelightHelpers.setLEDMode_ForceOff(LIMELIGHT_1);
    LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_1,
        CAMERA_FORWARD_OFFSET, // Forward offset (meters)
        CAMERA_SIDE_OFFSET, // Side offset (meters) left is positive
        CAMERA_HEIGHT_OFFSET, // Height offset (meters)
        CAMERA_ROLL_OFFSET, // Roll (degrees)
        CAMERA_PITCH_OFFSET, // Pitch (degrees)
        CAMERA_YAW_OFFSET // Yaw (degrees)
    );
    // change name later
    LimelightHelpers.setLEDMode_ForceOff(LIMELIGHT_2);
    LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_2,
        CAMERA2_FORWARD_OFFSET, // Forward offset (meters)
        CAMERA2_SIDE_OFFSET, // Side offset (meters) left is positive
        CAMERA2_HEIGHT_OFFSET, // Height offset (meters)
        CAMERA2_ROLL_OFFSET, // Roll (degrees)
        CAMERA2_PITCH_OFFSET, // Pitch (degrees)
        CAMERA2_YAW_OFFSET // Yaw (degrees)
    );

  }

  // **get valid target from camera 1*/
  public boolean getTv() {
    return LimelightHelpers.getTV(LIMELIGHT_1);
  }

  // **get valid target from camera 2*/
  public boolean getTvFromCamera2() {
    return LimelightHelpers.getTV(LIMELIGHT_2);
  }

  /**
   * get Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27
   * degrees / LL2: -29.8 to 29.8 degrees)
   */
  public double getTx() {
    return LimelightHelpers.getTX(LIMELIGHT_1);
  }

  /**
   * get Horizontal Offset From Second Camera Crosshair To Target (LL1: -27
   * degrees to 27
   * degrees / LL2: -29.8 to 29.8 degrees)
   */
  public double getTxFromCamera2() {
    return LimelightHelpers.getTX(LIMELIGHT_2);
  }

  /**
   * get Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5
   * degrees / LL2: -24.85 to 24.85 degrees)
   */
  public double getTy() {
    return LimelightHelpers.getTY(LIMELIGHT_1);
  }

  /**
   * get Vertical Offset From Second Camera Crosshair To Target (LL1: -20.5
   * degrees to 20.5
   * degrees / LL2: -24.85 to 24.85 degrees)
   */
  public double getTyFromCamera2() {
    return LimelightHelpers.getTY(LIMELIGHT_2);
  }

  /** get ID of the primary in-view AprilTag */
  public int getID() {
    return (int) LimelightHelpers.getFiducialID(LIMELIGHT_1);
  }

  /** get ID of the primary in-view AprilTag from the Second Camera */
  public int getIDFromCamera2() {
    return (int) LimelightHelpers.getFiducialID(LIMELIGHT_2);
  }

  public double[] getTargetPose() {
    if (getTv()) {
      targetPose = LimelightHelpers.getTargetPose_RobotSpace(LIMELIGHT_1);
    }
    return targetPose;
  }

  public double[] getTargetPoseFromCamera2() {
    if (getTv()) {
      targetPose2 = LimelightHelpers.getTargetPose_RobotSpace(LIMELIGHT_2);
    }
    return targetPose2;
  }

  public double getTargetYaw() {
    getTargetPose();
    return targetPose[5];
  }

  public double getTargetYawFromCamera2() {
    getTargetPoseFromCamera2();
    return targetPose2[5];
  }

  public Rotation2d getTargetYawRotation2d() {
    getTargetPose();
    return Rotation2d.fromDegrees(targetPose[5]);
  }

  public Rotation2d getTargetYawRotation2dFromCamera2() {
    getTargetPoseFromCamera2();
    return Rotation2d.fromDegrees(targetPose2[5]);
  }

  public double[] getBotPose() {
    // depending on how we do want to do our vision we could have regular getBotPose
    if (getTv()) {
      pose = LimelightHelpers.getBotPose_wpiBlue(LIMELIGHT_1);
    } else if (getTvFromCamera2()) {
      pose = LimelightHelpers.getBotPose_wpiBlue(LIMELIGHT_2);
    }
    return pose;
    // currently defaults to 0 if there's no pose
  }

  public double getDistanceFromPrimaryTarget() {
    return getBotPose()[9];
    // based on camera not robot
  }

  public double offsetToOmega(double offsetAngle) {
    offsetAngle /= 32; // maximum possible tx value is 29.8 in either direc

    double scaler = SwerveConstants.MAX_ROTATION_SPEED * .5;

    // Use power proportional to the offset angle
    return offsetAngle * scaler;
  }

  public void setPriorityID(int id) {
    LimelightHelpers.setPriorityTagID(LIMELIGHT_1, id);
  }

  public void setPriorityIDForCamera2(int id) {
    LimelightHelpers.setPriorityTagID(LIMELIGHT_1, id);
  }

  public void setPipelineID(int id) {
    LimelightHelpers.setPipelineIndex(LIMELIGHT_1, id);
  }

  public Command limelightDefaultCommand() {
    return run(() -> {

    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Field Localization code below
    // m_robotYaw = m_swerve.getRobotYaw
    // LimelightHelpers.PoseEstimate limelightMeasurement =
    // LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_1);
    // if (limelightMeasurement.tagCount >= 2) { // Only trust measurement if we see
    // multiple tags
    // m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7,
    // 9999999));
    // m_poseEstimator.addVisionMeasurement(
    // limelightMeasurement.pose,
    // limelightMeasurement.timestampSeconds);
    //
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
