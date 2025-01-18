// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Subsystem;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {
  LimelightHelpers m_limelightHelpers;
  SwerveDrivePoseEstimator m_poseEstimator;
  double m_robotYaw;
  Swerve m_swerve;
  LimelightHelpers.PoseEstimate m_limelightMeasurement;
  private double[] pose = new double[11];
  private double[] targetPose = new double[6];
  private double[] targetPose2 = new double[6];
  boolean m_hasBeenConstructed = false;
  CommandXboxController m_driverController;

  /** Creates a new ExampleSubsystem. */
  public Limelight() {
    m_swerve = Subsystem.swerve;
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
    // change name later
    LimelightHelpers.setLEDMode_ForceOff("limelight-2");
    LimelightHelpers.setCameraPose_RobotSpace("limelight-2",
        LimelightConstants.CAMERA2_FORWARD_OFFSET, // Forward offset (meters)
        LimelightConstants.CAMERA2_SIDE_OFFSET, // Side offset (meters) left is positive
        LimelightConstants.CAMERA2_HEIGHT_OFFSET, // Height offset (meters)
        LimelightConstants.CAMERA2_ROLL_OFFSET, // Roll (degrees)
        LimelightConstants.CAMERA2_PITCH_OFFSET, // Pitch (degrees)
        LimelightConstants.CAMERA2_YAW_OFFSET // Yaw (degrees)
    );
    m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    if (m_hasBeenConstructed) {
      try {
        throw new IllegalAccessException("Swerve subsystem was instantiated twice");
      } catch (IllegalAccessException e) {
        System.out.println("don't instantiate a subsystem twice!");
      }
    }
    m_hasBeenConstructed = true;
  }

  // **get valid target from camera 1*/
  public boolean getTv() {
    return LimelightHelpers.getTV("limelight");
  }

  // **get valid target from camera 1*/
  public boolean getTvFromCamera2() {
    return LimelightHelpers.getTV("limelight-2");
  }

  /**
   * get Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27
   * degrees / LL2: -29.8 to 29.8 degrees)
   */
  public double getTx() {
    return LimelightHelpers.getTX("limelight");
  }

  /**
   * get Horizontal Offset From Second Camera Crosshair To Target (LL1: -27
   * degrees to 27
   * degrees / LL2: -29.8 to 29.8 degrees)
   */
  public double getTxFromCamera2() {
    return LimelightHelpers.getTX("limelight-2");
  }

  /**
   * get Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5
   * degrees / LL2: -24.85 to 24.85 degrees)
   */
  public double getTy() {
    return LimelightHelpers.getTY("limelight");
  }

  /**
   * get Vertical Offset From Second Camera Crosshair To Target (LL1: -20.5
   * degrees to 20.5
   * degrees / LL2: -24.85 to 24.85 degrees)
   */
  public double getTyFromCamera2() {
    return LimelightHelpers.getTY("limelight-2");
  }

  /** get ID of the primary in-view AprilTag */
  public int getID() {
    return (int) LimelightHelpers.getFiducialID("limelight");
  }

  /** get ID of the primary in-view AprilTag from the Second Camera */
  public int getIDFromCamera2() {
    return (int) LimelightHelpers.getFiducialID("limelight-2");
  }

  public double[] getTargetPose() {
    if (getTv()) {
      targetPose = LimelightHelpers.getTargetPose_RobotSpace("limelight");
    }
    return targetPose;
  }

  public double[] getTargetPoseFromCamera2() {
    if (getTv()) {
      targetPose2 = LimelightHelpers.getTargetPose_RobotSpace("limelight-2");
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

  public double[] getBotPose() {
    // depending on how we do want to do our vision we could have regular getBotPose
    if (getTv()) {
      pose = LimelightHelpers.getBotPose_wpiBlue("limelight");
    } else if (getTvFromCamera2()) {
      pose = LimelightHelpers.getBotPose_wpiBlue("limelight-2");
    }
    return pose;
    // currently defaults to 0 if there's no pose
  }

  public double getDistanceFromPrimaryTarget() {
    return getBotPose()[9];
    // based on camera not robot
  }

  public CommandXboxController getXboxController() {
    return m_driverController;
  }

  public double offsetToOmega(double offsetAngle) {
    offsetAngle /= 32; // maximum possible tx value is 29.8 in either direc

    double scaler = SwerveConstants.MAX_ROTATION_SPEED * .5;

    // Use power proportional to the offset angle
    return offsetAngle * scaler;
  }

  public void setPriorityID(int id) {
    LimelightHelpers.setPriorityTagID("limelight", id);
  }

  public void setPriorityIDForCamera2(int id) {
    LimelightHelpers.setPriorityTagID("limelight", id);
  }

  public void setPipelineID(int id) {
    LimelightHelpers.setPipelineIndex("limelight", id);
  }

  public Command limelightDefaultCommand() {
    return run(() -> {
  public void setPipelineIDForCamera2(int id) {
    LimelightHelpers.setPipelineIndex("limelight", id);
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
    // m_robotYaw = m_swerve.getRobotYaw
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
