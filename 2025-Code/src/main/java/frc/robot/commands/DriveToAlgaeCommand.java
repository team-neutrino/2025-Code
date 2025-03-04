// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Swerve.SwerveRequestStash;
import frc.robot.util.DriveToPointController;
import frc.robot.util.Subsystem;

import static frc.robot.Constants.DriveToPoint.*;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.GlobalConstants.*;
import static frc.robot.util.Subsystem.swerve;

import java.util.List;

public class DriveToAlgaeCommand extends Command {
  private DriveToPointController m_pointControl = new DriveToPointController();
  private List<Pose2d> m_reefPoses;

  public DriveToAlgaeCommand() {
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    swerve.setDrivingToPoint(true);
    swerve.setAtPoint(false);

    if (!redAlliance.isPresent()) {
      System.out.println("NO ALLIANCE VALUE YET");
      return;
    }
    m_reefPoses = redAlliance.get() ? RED_REEF_CENTER : BLUE_REEF_CENTER;

    obtainTarget();
  }

  @Override
  public void execute() {
    if (!redAlliance.isPresent()) {
      System.out.println("NO ALLIANCE VALUE YET");
      return;
    }
    m_reefPoses = redAlliance.get() ? RED_REEF_CENTER : BLUE_REEF_CENTER;
    drive();
    isAtPoint();
    if (swerve.isAtPoint()) {
      obtainTarget();
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setDrivingToPoint(false);
    swerve.setAtPoint(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void obtainTarget() {
    swerve.setDrivingToPoint(true);
    swerve.setAtPoint(false);
    m_pointControl.setTargetNearest(m_reefPoses);
  }

  public void isAtPoint() {
    if (Math.abs(m_pointControl.getTarget().getX() - swerve.getCurrentPose().getX()) < AT_POINT_TOLERANCE
        && (Math.abs(m_pointControl.getTarget().getY() - swerve.getCurrentPose().getY()) < AT_POINT_TOLERANCE)) {
      swerve.setDrivingToPoint(false);
      swerve.setAtPoint(true);
    }
  }

  private void drive() {
    double velx = m_pointControl.getXVelocity(), vely = m_pointControl.getYVelocity();
    double xsign = Math.signum(velx), ysign = Math.signum(vely);
    if (Subsystem.elevator.getHeight() >= ElevatorConstants.SLOW_MOVE_THRESHOLD) {
      velx = xsign * Math.min(Math.abs(velx), SLOW_SWERVE_SPEED);
      vely = ysign * Math.min(Math.abs(vely), SLOW_SWERVE_SPEED);
    }
    SwerveRequestStash.driveWithVelocity
        .withVelocityX(velx)
        .withVelocityY(vely)
        .withTargetDirection(m_pointControl.getRotation());
    swerve.setControl(SwerveRequestStash.driveWithVelocity);
  }
}
