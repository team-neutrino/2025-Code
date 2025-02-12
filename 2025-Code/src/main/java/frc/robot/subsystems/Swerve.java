// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
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

import java.io.IOException;

import org.json.simple.parser.ParseException;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.Subsystem;
import frc.robot.util.GeneratedSwerveCode.*;

/**
 * Main swerve subsystem file; wraps CommandSwerveDriveTrain to avoid
 * modification of generated code.
 */
public class Swerve extends CommandSwerveDrivetrain {
  private boolean m_hasBeenConstructed = false;
  /**
   * set by driveAssistCom; when the command isn't being run it should be false
   * and otherwise it will represent whether or not the robot is within an
   * acceptable error of its target
   */
  private boolean isAligned = false;

  private Telemetry m_telemetry = new Telemetry(MAX_SPEED);

  private double m_speed = MAX_SPEED;

  private double m_rotationSpeed = MAX_ROTATION_SPEED;

  /**
   * Constructs the drivetrain using the values found in {@link TunerConstants}.
   * <p>
   * Only call this constructor ONCE!!
   * 
   * @throws IllegalAccessException In case this constructor was called more than
   *                                once, throw an exception.
   */
  public Swerve(boolean valkyrie) {
    super(valkyrie ? ValkyrieTunerConstants.DrivetrainConstants : TunerConstants.DrivetrainConstants,
        valkyrie ? ValkyrieTunerConstants.FrontLeft : TunerConstants.FrontLeft,
        valkyrie ? ValkyrieTunerConstants.FrontRight : TunerConstants.FrontRight,
        valkyrie ? ValkyrieTunerConstants.BackLeft : TunerConstants.BackLeft,
        valkyrie ? ValkyrieTunerConstants.BackRight : TunerConstants.BackRight);

    configureRequestPID();
    // if the robot power was never killed but code was redeployed/rebooted then the
    // swerve's yaw will zero itself but the pigeon will retain its previous value.
    resetRotation(Rotation2d.fromDegrees(getYawDegrees()));
    if (m_hasBeenConstructed) {
      try {
        throw new IllegalAccessException("Swerve subsystem was instantiated twice");
      } catch (IllegalAccessException e) {
        System.out.println("don't instantiate a subsystem twice!");
      }
    }

    if (valkyrie) {
      System.out.println("******************** VALKYRIE ********************");
      configurePathPlannerValkyrie();
    } else {
      System.out.println("******************** 2025 reefscape robot ********************");
      configurePathPlanner();
    }
    m_hasBeenConstructed = true;
    registerTelemetry(m_telemetry::telemeterize);
  }

  /**
   * Gets yaw from 0-360, going to the right
   */
  public double getYaw360() {
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

  public void setControlAndApplyChassis(ChassisSpeeds speeds) {
    setControl(
        SwerveRequestStash.autonDrive.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond));
  }

  private void configurePathPlanner() {
    double pTranslation = 1;
    double iTranslation = 0;
    double dTranslation = 0;
    double pRotation = 1;
    double iRotation = 0;
    double dRotation = 0;
    PIDConstants translationConstants = new PIDConstants(pTranslation, iTranslation, dTranslation);
    PIDConstants rotationConstants = new PIDConstants(pRotation, iRotation, dRotation);

    try {
      RobotConfig robotConfig = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getCurrentPose,
          this::resetPose,
          this::getChassisSpeeds,
          this::setControlAndApplyChassis,
          new PPHolonomicDriveController(
              translationConstants,
              rotationConstants),
          robotConfig,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this);
    } catch (IOException | ParseException e) {
      // TODO Auto-generated catch block
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }
  }

  private void configurePathPlannerValkyrie() {
    double pTranslation = ValkyrieTunerConstants.FrontLeft.DriveMotorGains.kP;
    double iTranslation = ValkyrieTunerConstants.FrontLeft.DriveMotorGains.kI;
    double dTranslation = ValkyrieTunerConstants.FrontLeft.DriveMotorGains.kD;
    double pRotation = ValkyrieTunerConstants.FrontLeft.SteerMotorGains.kP;
    double iRotation = ValkyrieTunerConstants.FrontLeft.SteerMotorGains.kI;
    double dRotation = ValkyrieTunerConstants.FrontLeft.SteerMotorGains.kD;
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
            new ModuleConfig(Inches.of(2), ValkyrieTunerConstants.kSpeedAt12Volts, 1,
                new DCMotor(NOMINAL_VOLTAGE, STALL_TORQUE, STALL_CURRENT, FREE_CURRENT_AMPS, FREE_SPEED_RADS,
                    NUM_MOTORS_GEARBOX),
                DRIVE_GEAR_RATIO, Amps.of(60), NUM_MOTORS_GEARBOX),
            new Translation2d(ValkyrieTunerConstants.FrontLeft.LocationX, ValkyrieTunerConstants.FrontLeft.LocationY),
            new Translation2d(ValkyrieTunerConstants.FrontRight.LocationX, ValkyrieTunerConstants.FrontRight.LocationY),
            new Translation2d(ValkyrieTunerConstants.BackLeft.LocationX, ValkyrieTunerConstants.BackLeft.LocationY),
            new Translation2d(ValkyrieTunerConstants.BackRight.LocationX, ValkyrieTunerConstants.BackRight.LocationY)),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  public void resetSwerveYaw() {
    resetRotation(new Rotation2d(0));
    getPigeon2().reset();
  }

  /**
   * gets yaw -180 - 180 according to swerve
   * 
   * @return yaw in degrees
   */
  public double getYawDegrees() {
    return Math.toDegrees(getYawRadians());
  }

  public double getYawRadians() {
    return MathUtil.angleModulus(Math.toRadians(getPigeon2().getYaw().getValueAsDouble()));
  }

  public boolean getIsAlinged() {
    return isAligned;
  }

  /**
   * THIS SHOULD ONLY BE USED BY DRIVEASSISTCOM
   */
  public void setIsAligned(boolean value) {
    isAligned = value;
  }

  /**
   * Returns the default command for the swerve - drives the robot according to
   * the stick values on the driver's controller.
   * 
   * @param controller The driver controller.
   * @return The default command.
   */
  public Command swerveDefaultCommand(CommandXboxController controller) {
    return applyRequest(() -> SwerveRequestStash.drive.withVelocityX(-controller.getLeftY() * m_speed)
        .withVelocityY(-controller.getLeftX() * m_speed)
        .withRotationalRate(-controller.getRightX() * m_rotationSpeed));
  }

  public Command resetYawCommand() {
    return run(() -> resetYaw());
  }

  @Override
  public void periodic() {
    if (Subsystem.elevator.getEncoderPosition() >= ElevatorConstants.L3) {
      m_speed = SLOW_SWERVE_SPEED;
      m_rotationSpeed = SLOW_ROTATION_SPEED;
    } else {
      m_speed = MAX_SPEED;
      m_rotationSpeed = MAX_ROTATION_SPEED;
    }
  }

  /**
   * Container for SwerveRequests to be used in building swerve commands.
   */
  public class SwerveRequestStash {
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withDeadband(MAX_SPEED * 0.1)
        .withRotationalDeadband(MAX_ROTATION_SPEED * 0.06);
    public static final SwerveRequest.FieldCentric driveWithoutDeadband = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public static final SwerveRequest.FieldCentricFacingAngle driveAssist = new FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withDeadband(MAX_SPEED * 0.05)
        .withRotationalDeadband(MAX_ROTATION_SPEED * .06);
    public static final SwerveRequest.RobotCentric autonDrive = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.Velocity);
  }

  public void configureRequestPID() {
    SwerveRequestStash.driveAssist.HeadingController.setPID(AUTO_ALIGN_P, 0, AUTO_ALIGN_D);
  }
}
