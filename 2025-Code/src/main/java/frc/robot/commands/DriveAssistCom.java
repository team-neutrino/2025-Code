// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveRequestStash;
import static frc.robot.util.Subsystem.*;

public class DriveAssistCom extends Command {
  private CommandXboxController controller;

  // (swerve.getTagDx(id) + (driverController.getHID().getPOV() == 270 ?
  // LEFT_REEF_OFFSET
  // : driverController.getHID().getPOV() == 90 ? RIGHT_REEF_OFFSET : 0))
  public DriveAssistCom(CommandXboxController p_controller) {
    addRequirements(swerve);
    controller = p_controller;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    FieldCentricFacingAngle req = SwerveRequestStash.autoAlign;
    // int id = limelight.getID();
    int id = 7;
    double tagAngle = swerve.getTagAngle(id);
    double yaw = swerve.getYaw180();
    double yVel = 0;
    double xVel = 0;

    if (id == 7 || id == 10) {
      xVel = MathUtil.clamp(SwerveConstants.APRILTAG_ALIGN_KP * ((yaw - tagAngle) / 2), -.5,
          .5);
      yVel = controller.getLeftX() * SwerveConstants.MAX_SPEED;
    } else {
      yVel = MathUtil.clamp(SwerveConstants.APRILTAG_ALIGN_KP * ((yaw - tagAngle) / 2), -.5,
          .5);
      xVel = -controller.getLeftY() * SwerveConstants.MAX_SPEED;
    }

    swerve.setControl(req.withTargetDirection(Rotation2d.fromDegrees(yaw - limelight.getTx())));
  }

  /**
   * Uses limelight values to solve for the legs of the right triangle formed by
   * the robot and the reef apriltag the robot is currently facing. Still needs to
   * be converted in terms of field centric movement
   * 
   * @return A translation2d containing the legs of the right triangle according
   *         to wpilb conventions (left is positive Y, up is positive X)
   */
  private Translation2d reefRelativeDistances() {
    double distance = limelight.getTy();
    double angle = swerve.getYaw180() - limelight.getTx();

    // in terms of wpilb conventions, so positive is left
    double yDist = distance * Math.cos(Math.toRadians(angle));

    // in terms of wpilb conventions, so positive is up
    double xDist = distance * Math.sin(Math.toRadians(angle));

    return new Translation2d(xDist, yDist);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
