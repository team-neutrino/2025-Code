// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.PublicKey;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {
  LimelightHelpers m_limelightHelpers;
  SwerveDrivePoseEstimator m_poseEstimator;
  double robotYaw;
  LimelightHelpers.PoseEstimate limelightMeasurement;
  private double[] pose = new double[11];
  private double[] targetPose = new double[6];

  /** Creates a new ExampleSubsystem. */
  public Limelight() {
    m_limelightHelpers = new LimelightHelpers();
    // fake pipeline number
    // LimelightHelpers.setPipelineIndex("limelight", 1);
    LimelightHelpers.setLEDMode_ForceOff("limelight");
    LimelightHelpers.setCameraPose_RobotSpace("limelight",
        LimelightConstants.CAMERA_FORWARD_OFFSET, // Forward offset (meters)
        LimelightConstants.CAMERA_SIDE_OFFSET, // Side offset (meters) left is positive
        LimelightConstants.CAMERA_HEIGHT_OFFSET, // Height offset (meters)
        LimelightConstants.CAMERA_ROLL_OFFSET, // Roll (degrees)
        LimelightConstants.CAMERA_PITCH_OFFSET, // Pitch (degrees)
        LimelightConstants.CAMERA_YAW_OFFSET // Yaw (degrees)
    );
  }

  // get valid target
  public boolean getTv() {
    return LimelightHelpers.getTV("limelight");
  }

  /**
   * get Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27
   * degrees / LL2: -29.8 to 29.8 degrees)
   */
  public double getTx() {
    return LimelightHelpers.getTX("limelight");
  }

  /**
   * get Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5
   * degrees / LL2: -24.85 to 24.85 degrees)
   */
  public double getTy() {
    return LimelightHelpers.getTY("limelight");
  }

  /** get ID of the primary in-view AprilTag */
  public int getID() {
    return (int) LimelightHelpers.getFiducialID("limelight");
  }

  public double[] getTargetPose() {
    if (getTv()) {
      targetPose = LimelightHelpers.getTargetPose_RobotSpace("limelight");
    }
    return targetPose;
    // currently defaults to 0 if there's no target
  }
  // set target yaw

  public double getTargetYaw() {
    getTargetPose();
    return targetPose[5];
  }

  public double[] getBotPose() {
    // depending on how we do want to do our vision we could have regular getBotPose
    if (getTv()) {
      pose = LimelightHelpers.getBotPose_wpiBlue("limelight");
    }
    return pose;
    // currently defaults to 0 if there's no pose
  }

  public double getDistanceFromPrimaryTarget() {
    return getBotPose()[9];
    // based on camera not robot
  }

  public void setPriorityID(int id) {
    LimelightHelpers.setPriorityTagID("limelight", id);
  }

  public void setPipelineID(int id) {
    LimelightHelpers.setPipelineIndex("limelight", id);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Field Localization code below
    // LimelightHelpers.PoseEstimate limelightMeasurement =
    // LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
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
