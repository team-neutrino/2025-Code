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
  private double POIoffset = 0;

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
    req.HeadingController.setPID(4, 0, .5);

    Translation2d error = getFieldRelativeDistances();
    double xVel = MathUtil.clamp(error.getX() * DRIVE_ASSIST_KP, -APRILTAG_ALIGN_LIMIT, APRILTAG_ALIGN_LIMIT);
    double yVel = MathUtil.clamp(error.getY() * DRIVE_ASSIST_KP, -APRILTAG_ALIGN_LIMIT, APRILTAG_ALIGN_LIMIT);
    Rotation2d angle = Rotation2d.fromDegrees(swerve.getYawDegrees() - limelight.getTx());
    req.withVelocityX(xVel).withVelocityY(yVel).withTargetDirection(angle);
  }

  /**
   * Uses limelight values to solve for the appropriate leg of the right triangle
   * formed by the robot, the reef apriltag the robot is currently facing, and the
   * desired line to move the robot to. Then converts into field relative values
   * with more trig.
   * 
   * @return A translation2d containing the shortest possible field-oriented x and
   *         y distances from the target line in the wpilb plane (left is positive
   *         y and up is positive x).
   */
  private Translation2d getFieldRelativeDistances() {
    int id = limelight.getID();
    double yaw = swerve.getYaw();
    double limelightTagToRobot = limelight.getDistanceFromPrimaryTarget();

    // int pov = controller.getHID().getPOV();
    // POIoffset = (pov == 270 ? -SwerveConstants.REEF_OFFSET : pov == 90 ?
    // REEF_OFFSET : POIoffset);
    // limelight.setPointOfInterest(0, POIoffset);

    double hexagonAngle = Integer.MAX_VALUE;
    switch (id) {
      case 10:
        hexagonAngle = 0;
        break;
      case 9:
        hexagonAngle = -60;
        break;
      case 8:
        hexagonAngle = -120;
        break;
      case 7:
        hexagonAngle = yaw > 0 ? 180 : -180;
        break;
      case 11:
        hexagonAngle = 60;
        break;
      case 6:
        hexagonAngle = 120;
        break;
    }
    double triangle1Angle = Math.toRadians(hexagonAngle - yaw);
    double error = Math.abs(Math.sin(triangle1Angle) * limelightTagToRobot);

    double audaciousTri2Angle = hexagonAngle + (triangle1Angle > 0 ? 180 : 0);
    audaciousTri2Angle = id == 7 ? -audaciousTri2Angle : audaciousTri2Angle;

    // wpilb y axis is inverted
    double yError = -(error * Math.cos(audaciousTri2Angle));
    double xError = error * Math.sin(audaciousTri2Angle);

    return new Translation2d(xError, yError);
  }

  @Override
  public void end(boolean interrupted) {
    POIoffset = 0;
    limelight.setPointOfInterest(0, POIoffset);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(controller.getRightX()) >= .75;
  }
}