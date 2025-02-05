// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import static frc.robot.Constants.SwerveConstants.*;

import frc.robot.util.GeneratedSwerveCode.*;

/**
 * Main swerve subsystem file; wraps CommandSwerveDriveTrain to avoid
 * modification of generated code.
 */
public class Swerve extends CommandSwerveDrivetrain {
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
   * vertical axis, from -180 -> 180 as defined by the SWERVE, not pigeon.
   * 
   * @return The yaw of the robot in degrees.
   */
  public double getYaw() {
    return getCurrentPose().getRotation().getDegrees();
  }

  /**
   * Gets the current Pose of the robot.
   * 
   * @return The {@link Pose2d} representation of the robot's current position.
   */
  public Pose2d getCurrentPose() {
    return getState().Pose;
  }

  /**
   * Resets the yaw to 0, so the direction you're currently facing is the new
   * forwards.
   */
  public void resetYaw() {
    resetRotation(new Rotation2d(0));
    getPigeon2().reset();
    // need more research on the following
    // seedFieldCentric();
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

  public Command resetYawCommand() {
    return run(() -> resetYaw());
  }

  @Override
  public void periodic() {
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
