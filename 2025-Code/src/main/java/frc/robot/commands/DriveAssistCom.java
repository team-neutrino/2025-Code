// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static frc.robot.Constants.SwerveConstants.*;

import frc.robot.Constants;
import frc.robot.Constants.AprilTagConstants.*;
import frc.robot.subsystems.Swerve.SwerveRequestStash;
import static frc.robot.util.Subsystem.*;

import java.lang.annotation.Target;

import org.ejml.data.FEigenpair;

public class DriveAssistCom extends Command {
  private FieldCentricFacingAngle req = SwerveRequestStash.driveAssist;
  private CommandXboxController m_controller;
  private double m_POIoffset = 0;
  private int m_staticTagID;
  Translation2d error;

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
    if (!limelight.getTv() || exitExecute()) {
      swerve.setControl(req.withVelocityX(0)
          .withVelocityY(0));
      return;
    }
    int pov = m_controller.getHID().getPOV();
    m_POIoffset = pov == 270 ? -REEF_OFFSET : pov == 90 ? REEF_OFFSET : m_POIoffset;
    limelight.setPointOfInterest(0, m_POIoffset);

    Translation2d velocities = getVelocities();
    Translation2d updatedDriverVel = getNewDriveVelocity();
    swerve.setIsAligned(isAligned());
    Rotation2d angle = Rotation2d.fromDegrees(swerve.getYawDegrees() - limelight.getTx());
    swerve.setControl(req.withVelocityX((velocities.getX() + updatedDriverVel.getX() * MAX_SPEED) / 2)
        .withVelocityY((velocities.getY() + updatedDriverVel.getY() * MAX_SPEED) / 2).withTargetDirection(angle));
    System.out.println("ID " + limelight.getID());
    System.out.println("X " + updatedDriverVel.getX());
    System.out.println("Y " + updatedDriverVel.getY());
  }

  private boolean exitExecute() {
    if (m_staticTagID == -1) {
      setPriorityID();
    }
    return m_staticTagID != limelight.getID();

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
    double idealYaw = swerve.getYawDegrees() - limelight.getTx();
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

  private Translation2d getNewDriveVelocity() {
    double hexagonAngle = 0;
    double driverAngle = 0;
    double otherAngle = 0;
    double magnitude = 0;
    double desiredMagnitude = 0;
    int id = m_staticTagID;
    Translation2d finalVelocities = null;
    double driverX = m_controller.getLeftY();
    double driverY = m_controller.getLeftX();
    double desiredX = 0;
    double desiredY = 0;
    boolean edgeCase = id == RED_ALLIANCE_IDS.REEF_FACING_ALLIANCE || id == BLUE_ALLIANCE_IDS.REEF_FACING_ALLIANCE;
    if (m_controller.getLeftX() == 0 && m_controller.getLeftY() == 0) {
      return new Translation2d(0, 0);
    } else if (edgeCase) {
      finalVelocities = new Translation2d(-driverX, 0);
    } else if (id == 6 || id == 19) {
      hexagonAngle = HEXAGON_ANGLES[6];
      driverAngle = Math.atan2(driverX, driverY);
      magnitude = Math.sqrt(Math.pow(driverX, 2) + Math.pow(driverY, 2));
      if (driverAngle < 30) {
        // input is right of april tag (facing it)
        otherAngle = 30 - driverAngle;
      }
      otherAngle = driverAngle - 30;
      desiredMagnitude = Math.cos(otherAngle) * magnitude;
      desiredX = desiredMagnitude * (Math.sqrt(3) / 2);
      desiredY = desiredMagnitude * 0.5;
      finalVelocities = new Translation2d(-desiredX, -desiredY);
    }
    if (finalVelocities == null) {
      return new Translation2d(0, 0);
    } else {
      return finalVelocities;
    }
  }

  /**
   * Does PID scaling and limiting on error values as returned by
   * {@link #getFieldRelativeDistances} and returns a suitable value.
   * 
   * @return A translation2d in the WPiLB standard coordinate system: (left is
   *         positive y and up is positive x).
   */
  private Translation2d getVelocities() {
    error = getFieldRelativeDistances();
    double xVel = MathUtil.clamp(error.getX() * DRIVE_ASSIST_KP, -APRILTAG_ALIGN_LIMIT, APRILTAG_ALIGN_LIMIT);
    double yVel = MathUtil.clamp(error.getY() * DRIVE_ASSIST_KP, -APRILTAG_ALIGN_LIMIT, APRILTAG_ALIGN_LIMIT);
    Translation2d ret = new Translation2d(xVel, yVel);
    return ret;
  }

  private void setPriorityID() {
    m_staticTagID = limelight.getID();
  }

  public boolean isAligned() {
    return Math.abs(error.getX()) + Math.abs(error.getY()) < Constants.SwerveConstants.isAlignedError;
  }

  @Override
  public void end(boolean interrupted) {
    m_POIoffset = 0;
    limelight.setPointOfInterest(0, m_POIoffset);
    swerve.setIsAligned(false);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_controller.getRightX()) >= .75;
  }
}