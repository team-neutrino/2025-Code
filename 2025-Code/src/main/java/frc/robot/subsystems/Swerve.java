// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.jni.SwerveJNI;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModulePosition;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import static frc.robot.Constants.SwerveConstants.*;

import frc.robot.util.GeneratedSwerveCode.CommandSwerveDrivetrain;
import frc.robot.util.GeneratedSwerveCode.Telemetry;
import frc.robot.util.GeneratedSwerveCode.TunerConstants;

/**
 * Main swerve subsystem file; wraps CommandSwerveDriveTrain to avoid
 * modification of generated code.
 */
public class Swerve extends CommandSwerveDrivetrain {
  public SwerveDrivePoseEstimator m_PoseEstimator;
  private Translation2d m_frontLeftLocation = new Translation2d(TunerConstants.FrontLeft.LocationX * 0.0254,
      TunerConstants.FrontLeft.LocationY * 0.0254);
  private Translation2d m_frontRightLocation = new Translation2d(TunerConstants.FrontRight.LocationX * 0.0254,
      TunerConstants.FrontRight.LocationY * 0.0254);
  private Translation2d m_backLeftLocation = new Translation2d(TunerConstants.BackLeft.LocationX * 0.0254,
      TunerConstants.BackLeft.LocationY * 0.0254);
  private Translation2d m_backRightLocation = new Translation2d(TunerConstants.BackRight.LocationX * 0.0254,
      TunerConstants.BackRight.LocationY * 0.0254);
  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation,
      m_backLeftLocation, m_backRightLocation);
  private SwerveModulePosition m_frontLeftModulePosition = new SwerveModulePosition();
  private SwerveModulePosition m_frontRightModulePosition = new SwerveModulePosition();
  private SwerveModulePosition m_backLeftModulePosition = new SwerveModulePosition();
  private SwerveModulePosition m_backRightModulePosition = new SwerveModulePosition();
  private SwerveModulePosition[] m_modulePositions = new SwerveModulePosition[4];
  private double angle[] = new double[4];
  private double distance[] = new double[4];
  private DriveState m_driveState = m_jni.driveState;
  private boolean m_hasBeenConstructed = false;

  private Telemetry m_telemetry = new Telemetry(MAX_SPEED);

  /**
   * Constructs the drivetrain using the values found in {@link TunerConstants}.
   * <p>
   * Only call this constructor ONCE!!
   * 
   * @throws IllegalAccessException In case this constructor was called more than
   *                                once, throw an exception.
   */
  public Swerve() {
    super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight,
        TunerConstants.BackLeft, TunerConstants.BackRight);

    if (m_hasBeenConstructed) {
      try {
        throw new IllegalAccessException("Swerve subsystem was instantiated twice");
      } catch (IllegalAccessException e) {
        System.out.println("don't instantiate a subsystem twice!");
      }
    }
    configurePathPlanner();
    m_hasBeenConstructed = true;
    registerTelemetry(m_telemetry::telemeterize);
  }

  /**
   * Returns the yaw of the robot, which is the rotation of the robot around the
   * vertical axis.
   * 
   * @return The yaw of the robot in degrees.
   */
  public double getYaw() {
    return getPigeon2().getYaw().getValueAsDouble() % 360;
  }

  /**
   * Gets the current Pose of the robot.
   * 
   * @return The {@link Pose2d} representation of the robot's current position.
   */
  public Pose2d getCurrentPose() {
    return getState().Pose;
  }

  public double getRotationalRate() {
    return getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
  }

  /**
   * Resets the pigeon's headings to 0.
   */
  public void resetPigeon() {
    getPigeon2().reset();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return getState().Speeds;
  }

  private void setControlAndApplyChassis(ChassisSpeeds speeds) {
    SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
        .withDriveRequestType(DriveRequestType.Velocity);
    this.setControl(applyRobotSpeeds.withSpeeds(speeds));
  }

  private void configurePathPlanner() {
    double pTranslation = TunerConstants.FrontLeft.DriveMotorGains.kP;
    double iTranslation = TunerConstants.FrontLeft.DriveMotorGains.kI;
    double dTranslation = TunerConstants.FrontLeft.DriveMotorGains.kD;
    double pRotation = TunerConstants.FrontLeft.SteerMotorGains.kP;
    double iRotation = TunerConstants.FrontLeft.SteerMotorGains.kI;
    double dRotation = TunerConstants.FrontLeft.SteerMotorGains.kD;
    PIDConstants translationConstants = new PIDConstants(pTranslation, iTranslation, dTranslation);
    PIDConstants rotationConstants = new PIDConstants(pRotation, iRotation, dRotation);
    AutoBuilder.configure(
        this::getCurrentPose,
        this::resetPose,
        this::getChassisSpeeds,
        this::setControlAndApplyChassis,
        new PPHolonomicDriveController(
            translationConstants,
            rotationConstants),
        new RobotConfig(Pounds.of(135), KilogramSquareMeters.of(3.92),
            new ModuleConfig(Inches.of(2), TunerConstants.kSpeedAt12Volts, 1,
                new DCMotor(NOMINAL_VOLTAGE, STALL_TORQUE, STALL_CURRENT, FREE_CURRENT_AMPS, FREE_SPEED_RADS,
                    NUM_MOTORS_GEARBOX),
                DRIVE_GEAR_RATIO, Amps.of(60), NUM_MOTORS_GEARBOX),
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  /**
   * Returns the default command for the swerve - drives the robot according to
   * the stick values on the driver's controller.
   * 
   * @param controller The driver controller.
   * @return The default command.
   */
  public Command swerveDefaultCommand(CommandXboxController controller) {
    return applyRequest(() -> SwerveRequestStash.drive.withVelocityX(controller.getLeftY() * MAX_SPEED)
        .withVelocityY(controller.getLeftX() * MAX_SPEED)
        .withRotationalRate(-controller.getRightX() * MAX_ROTATION_SPEED));
  }

  /**
   * Creates and returns a slower-driving version (but not rotating) version of
   * the default command. See {@link #getDefaultCommand} for details.
   * 
   * @param controller The driver controller.
   * @return A slow-driving default command.
   */
  public Command getSlowMoveCommand(CommandXboxController controller) {
    return applyRequest(
        () -> SwerveRequestStash.drive.withVelocityX(controller.getLeftY() * (MAX_SPEED / 2))
            .withVelocityY(controller.getLeftX() * (MAX_SPEED / 2))
            .withRotationalRate(-controller.getRightX() * (MAX_ROTATION_SPEED / 2)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    angle[0] = m_driveState.ModulePositions[0].angle;
    angle[1] = m_driveState.ModulePositions[1].angle;
    angle[2] = m_driveState.ModulePositions[2].angle;
    angle[3] = m_driveState.ModulePositions[3].angle;
    distance[0] = m_driveState.ModulePositions[0].distance;
    distance[1] = m_driveState.ModulePositions[1].distance;
    distance[2] = m_driveState.ModulePositions[2].distance;
    distance[3] = m_driveState.ModulePositions[3].distance;
    // TODO
    // absolutely no clue what unit anything is in we'll have to find this out from
    // testing (the docs are useless)
    m_frontLeftModulePosition = new SwerveModulePosition(distance[0], Rotation2d.fromDegrees(angle[0] * 360));
    m_frontRightModulePosition = new SwerveModulePosition(distance[1], Rotation2d.fromDegrees(angle[1] * 360));
    m_backLeftModulePosition = new SwerveModulePosition(distance[2], Rotation2d.fromDegrees(angle[2] * 360));
    m_backRightModulePosition = new SwerveModulePosition(distance[3], Rotation2d.fromDegrees(angle[3] * 360));
    m_modulePositions[0] = m_frontLeftModulePosition;
    m_modulePositions[1] = m_frontRightModulePosition;
    m_modulePositions[2] = m_backLeftModulePosition;
    m_modulePositions[3] = m_backRightModulePosition;

    m_PoseEstimator = new SwerveDrivePoseEstimator(m_kinematics,
        Rotation2d.fromDegrees(getYaw()), m_modulePositions, new Pose2d());
  }

  /**
   * Container for SwerveRequests to be used in building swerve commands.
   */
  private class SwerveRequestStash {
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withDeadband(MAX_SPEED * 0.1)
        .withRotationalDeadband(MAX_ROTATION_SPEED * 0.06);
    public static final SwerveRequest.FieldCentric driveWithoutDeadband = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  }
}
