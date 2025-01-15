// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.GeneratedSwerveCode.CommandSwerveDrivetrain;
import frc.robot.util.GeneratedSwerveCode.TunerConstants;

/**
 * Main swerve subsystem file; wraps CommandSwerveDriveTrain to avoid
 * modification of generated code.
 */
public class Swerve extends CommandSwerveDrivetrain {
  private boolean hasBeenConstructed = false;

  /**
   * Constructs the drivetrain using the values found in {@link TunerConstants}.
   * <p>
   * Only call this constructor ONCE!!
   * 
   * @throws IllegalAccessException In case this constructor was called more than
   *                                once, throw an exception.
   */
  public Swerve(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules)
      throws IllegalAccessException {

    super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight,
        TunerConstants.BackLeft, TunerConstants.BackRight);

    if (hasBeenConstructed) {
      throw new IllegalAccessException("Swerve subsystem was instantiated twice");
    }
    hasBeenConstructed = true;
  }

  /**
   * Returns the yaw of the robot, which is the rotation of the robot around the
   * vertical axis.
   * 
   * @return The yaw of the robot in degrees.
   */
  public double getYaw() {
    return getPigeon2().getYaw().getValueAsDouble();
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
   * Resets the pigeon's headings to 0.
   */
  public void resetPigeon() {
    getPigeon2().reset();
  }

  /**
   * Returns the default command for the swerve - drives the robot according to
   * the stick values on the driver's controller.
   * 
   * @param controller The driver controller.
   * @return The default command.
   */
  public Command getDefaultCommand(CommandXboxController controller) {
    return applyRequest(() -> SwerveRequestStash.drive.withVelocityX(controller.getLeftY() * 5.7)
        .withVelocityY(controller.getLeftX() * 5.7)
        .withRotationalRate(-controller.getRightX() * 1.5 * Math.PI));
  }

  /**
   * Creates and returns a slower-driving version (but not rotating) version of
   * the default command. See {@link #getDefaultCommand} for details.
   * 
   * @param controller The driver controller.
   * @return A slow-driving default command.
   */
  public Command getSlowMoveCommand(CommandXboxController controller) {
    return applyRequest(() -> SwerveRequestStash.drive.withVelocityX(controller.getLeftY() * 2.5)
        .withVelocityY(controller.getLeftX() * 2.5)
        .withRotationalRate(-controller.getRightX() * 1.5 * Math.PI));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Container for SwerveRequests to be used in building swerve commands.
   */
  private class SwerveRequestStash {
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // .withDeadband(SwerveConstants.MaxSpeed * 0.1)
    // .withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.06)
  }
}
