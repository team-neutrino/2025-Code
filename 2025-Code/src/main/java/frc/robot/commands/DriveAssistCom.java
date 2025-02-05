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

import frc.robot.Constants.AprilTagConstants.*;
import frc.robot.subsystems.Swerve.SwerveRequestStash;
import static frc.robot.util.Subsystem.*;

public class DriveAssistCom extends Command {
  FieldCentricFacingAngle req = SwerveRequestStash.driveAssist;
  private CommandXboxController m_controller;
  private double POIoffset = 0;

  public DriveAssistCom(CommandXboxController p_controller) {
    addRequirements(swerve);
    m_controller = p_controller;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (!limelight.getTv()) {
      return;
    }
    int pov = m_controller.getHID().getPOV();
    POIoffset = pov == 270 ? -REEF_OFFSET : pov == 90 ? REEF_OFFSET : POIoffset;
    limelight.setPointOfInterest(0, POIoffset);

    Translation2d error = getFieldRelativeDistances();
    double xVel = MathUtil.clamp(error.getX() * DRIVE_ASSIST_KP, -APRILTAG_ALIGN_LIMIT, APRILTAG_ALIGN_LIMIT);
    double yVel = MathUtil.clamp(error.getY() * DRIVE_ASSIST_KP, -APRILTAG_ALIGN_LIMIT, APRILTAG_ALIGN_LIMIT);
    Rotation2d angle = Rotation2d.fromDegrees(swerve.getYawDegrees() - limelight.getTx());
    swerve.setControl(req.withVelocityX(xVel + -m_controller.getLeftY() * MAX_SPEED)
        .withVelocityY(yVel + -m_controller.getLeftX() * MAX_SPEED).withTargetDirection(angle));
  }

  /**
   * Uses limelight values to solve for the appropriate leg of the right triangle
   * formed by the robot, the reef apriltag the robot is currently facing, and the
   * desired line to move the robot to. Then converts into field relative values
   * with more trig.
   * <p>
   * KNOWN UNSOLVED EDGE CASE: for IDs 6 and 8 specifically, an unexpected robot
   * yaw sign can screw up calculation, but this could only happen if the robot is
   * at a very unusual angle relative to the tag.
   * 
   * @return A translation2d containing the shortest possible field-oriented x and
   *         y distances from the target line in the wpilb plane (left is positive
   *         y and up is positive x).
   */
  private Translation2d getFieldRelativeDistances() {
    int id = limelight.getID();
    double yaw = swerve.getYaw();
    double limelightTagToRobot = limelight.getDistanceFromPrimaryTarget();

    double hexagonAngle = id == RED_ALLIANCE_IDS.REEF_FACING_ALLIANCE ? (Math.signum(yaw) * HEXAGON_ANGLES[id])
        : HEXAGON_ANGLES[id];

    double triangle1Angle = Math.toRadians(hexagonAngle - yaw);
    double error = Math.abs(Math.sin(triangle1Angle) * limelightTagToRobot);

    double audaciousTri2Angle = Math.toRadians(hexagonAngle + (triangle1Angle > 0 ? 180 : 0));
    audaciousTri2Angle = id == RED_ALLIANCE_IDS.REEF_FACING_ALLIANCE ? -audaciousTri2Angle : audaciousTri2Angle;

    // wpilb y and x axis are switched and the y axis is inverted
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
    return Math.abs(m_controller.getRightX()) >= .75;
  }
}