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

import static frc.robot.Constants.SwerveConstants.*;
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
    FieldCentricFacingAngle req = SwerveRequestStash.driveAssist;
    req.HeadingController.setPID(5, 0, .5);

    Translation2d error = getFieldRelativeDistances();
    double yVel = MathUtil.clamp(APRILTAG_ALIGN_KP * error.getY(), -APRILTAG_ALIGN_LIMIT,
        APRILTAG_ALIGN_LIMIT);
    double xVel = MathUtil.clamp(APRILTAG_ALIGN_KP * (error.getX()), -APRILTAG_ALIGN_LIMIT, APRILTAG_ALIGN_LIMIT);
    Rotation2d angle = Rotation2d.fromDegrees(swerve.getYawDegrees() - limelight.getTx());
    swerve.setControl(req.withTargetDirection(angle).withVelocityX(xVel).withVelocityY(yVel));
  }

  /**
   * Uses limelight values to solve for the appropriate leg of the right triangle
   * formed by the robot, the reef apriltag the robot is currently facing, and the
   * desired line to move the robot to. Then converts into field relative values.
   * 
   * @return A translation2d containing the shortest possible field-oriented x and
   *         y distances from the target line in the wpilb plane (left is positive
   *         y and up is positive x).
   */
  private Translation2d getFieldRelativeDistances() {
    int idMod = limelight.getID() % 7;
    // angle the reef side makes with the field-plane
    double reefSideAngle = idMod == 6 ? 300 : idMod * 60;
    reefSideAngle = 60;

    // angle of the robot-reef-target right triangle
    double triangle1angle = Math
        .toRadians((swerve.getYawDegrees() - limelight.getTx()) + reefSideAngle);
    // hypotenuse of above triangle
    double limelightTagToRobot = limelight.getDistanceFromPrimaryTarget();
    double targetError = (limelightTagToRobot) * Math.sin(triangle1angle);
    System.out.println(limelightTagToRobot);

    return new Translation2d(targetError * Math.cos(reefSideAngle), targetError * Math.sin(reefSideAngle));
  }

  @Override
  public void end(boolean interrupted) {
    SwerveRequestStash.autoAlign.withDeadband(MAX_SPEED * 0.1);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
