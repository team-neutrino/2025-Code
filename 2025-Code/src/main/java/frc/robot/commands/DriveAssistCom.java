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

import frc.robot.Constants;
import frc.robot.Constants.AprilTagConstants.*;
import frc.robot.subsystems.Swerve.SwerveRequestStash;

import static frc.robot.util.Subsystem.*;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.LimelightConstants.*;

public class DriveAssistCom extends Command {
  private FieldCentricFacingAngle req = SwerveRequestStash.driveAssist;
  private CommandXboxController m_controller;
  private double m_POIoffset = 0;
  private int m_staticTagID;
  private Translation2d m_error;

  public DriveAssistCom(CommandXboxController p_controller) {
    addRequirements(swerve);
    m_controller = p_controller;
  }

  @Override
  public void initialize() {
    m_staticTagID = -1;
  }

  @Override
  public void execute() {
    if (!limelight.getTvReef() || exitExecute()) {
      swerve.setControl(req.withVelocityX(0)
          .withVelocityY(0));
      return;
    }
    int pov = m_controller.getHID().getPOV();
    m_POIoffset = pov == 270 ? -REEF_OFFSET : pov == 90 ? REEF_OFFSET : m_POIoffset;
    limelight.setPointOfInterest(LL_REEF1, 0, m_POIoffset);

    updateError();
    swerve.setIsAligned(isAligned());
    Rotation2d angle = Rotation2d.fromDegrees(swerve.getYawDegrees() - limelight.getTxReef());
    swerve.setControl(req.withVelocityX(m_error.getX() + -m_controller.getLeftY() * MAX_SPEED)
        .withVelocityY(m_error.getY() + -m_controller.getLeftX() * MAX_SPEED).withTargetDirection(angle));
  }

  private boolean exitExecute() {
    if (m_staticTagID == -1) {
      m_staticTagID = limelight.getIDReef();
    }
    return m_staticTagID != limelight.getIDReef();

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
    int id = m_staticTagID;
    double idealYaw = swerve.getYawDegrees() - limelight.getTxReef();
    double limelightTagToRobot = limelight.getDistanceFromPrimaryTarget();

    // ID 7/18 edge case
    double hexagonAngle = HEXAGON_ANGLES[id] * (id == RED_ALLIANCE_IDS.REEF_FACING_ALLIANCE
        || id == BLUE_ALLIANCE_IDS.REEF_FACING_ALLIANCE ? Math.signum(idealYaw) : 1);

    double triangle1Angle = Math.toRadians(hexagonAngle - idealYaw);
    double error = Math.abs(Math.sin(triangle1Angle) * limelightTagToRobot);

    double audaciousTri2Angle = Math.toRadians(hexagonAngle + (triangle1Angle > 0 ? 180 : 0));
    // ID 7/18 edge case
    audaciousTri2Angle = (id == RED_ALLIANCE_IDS.REEF_FACING_ALLIANCE || id == BLUE_ALLIANCE_IDS.REEF_FACING_ALLIANCE)
        ? -audaciousTri2Angle
        : audaciousTri2Angle;

    // wpilb y and x axis are switched and the y axis is inverted
    double yError = -(error * Math.cos(audaciousTri2Angle));
    double xError = error * Math.sin(audaciousTri2Angle);

    return new Translation2d(xError, yError);
  }

  /**
   * Does PID scaling and limiting on error values as returned by
   * {@link #getFieldRelativeDistances} and returns a suitable value.
   * 
   * @return A translation2d in the WPiLB standard coordinate system: (left is
   *         positive y and up is positive x).
   */
  private void updateError() {
    Translation2d ret = getFieldRelativeDistances();
    double xVel = MathUtil.clamp(ret.getX() * DRIVE_ASSIST_KP, -APRILTAG_ALIGN_LIMIT, APRILTAG_ALIGN_LIMIT);
    double yVel = MathUtil.clamp(ret.getY() * DRIVE_ASSIST_KP, -APRILTAG_ALIGN_LIMIT, APRILTAG_ALIGN_LIMIT);
    m_error = new Translation2d(xVel, yVel);
  }

  public boolean isAligned() {
    return Math.abs(m_error.getX()) + Math.abs(m_error.getY()) < Constants.SwerveConstants.isAlignedError;
  }

  @Override
  public void end(boolean interrupted) {
    m_POIoffset = 0;
    limelight.setPointOfInterest(LL_REEF1, 0, m_POIoffset);
    limelight.setPointOfInterest(LL_STATION, 0, m_POIoffset);
    swerve.setIsAligned(false);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_controller.getRightX()) >= .75;
  }
}