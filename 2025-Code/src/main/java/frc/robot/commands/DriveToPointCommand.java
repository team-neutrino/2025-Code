// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Swerve.SwerveRequestStash;
import frc.robot.util.DriveToPointController;
import frc.robot.util.Subsystem;

import static frc.robot.Constants.DriveToPoint.*;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.GlobalConstants.*;
import static frc.robot.util.Subsystem.swerve;

import java.util.List;

public class DriveToPointCommand extends Command {
  private DriveToPointController m_pointControl = new DriveToPointController();
  private CommandXboxController m_xboxController;
  private List<Pose2d> m_reefPoses;
  private List<Pose2d> m_coralStationPoses;
  private boolean m_bumperWasPressed = false;
  private boolean m_hadGamePiece;
  private final boolean m_algae;

  public DriveToPointCommand(CommandXboxController xboxController, boolean algae) {
    m_algae = algae;
    m_xboxController = xboxController;
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
    m_reefPoses = m_algae ? REEF_ALGAE : (redAlliance.get() ? RED_REEF_RIGHT : BLUE_REEF_RIGHT);
    m_coralStationPoses = redAlliance.get() ? POSE_LIST.subList(0, 2) : POSE_LIST.subList(2, 4);

    obtainTarget();
    m_reefPoses = m_algae ? REEF_ALGAE : (redAlliance.get() ? RED_REEF : BLUE_REEF);
  }

  @Override
  public void execute() {
    if (!m_algae) {
      checkBumpers();
    }
    drive();
    isAtPoint();
    if (swerve.isAtPoint() && (Subsystem.coral.hasCoral() != m_hadGamePiece)) {
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
    if (m_algae) {
      m_pointControl.setTargetNearest(m_reefPoses);
      return;
    }
    boolean hasGamePiece = Subsystem.coral.hasCoral();
    m_hadGamePiece = hasGamePiece;
    if (hasGamePiece) {
      m_pointControl.setTargetNearest(m_reefPoses);
    } else {
      m_pointControl.setTargetNearest(m_coralStationPoses);
    }
  }

  public void isAtPoint() {
    if (Math.abs(m_pointControl.getTarget().getX() - swerve.getCurrentPose().getX()) < AT_POINT_TOLERANCE
        && (Math.abs(m_pointControl.getTarget().getY() - swerve.getCurrentPose().getY()) < AT_POINT_TOLERANCE)) {
      swerve.setDrivingToPoint(false);
      swerve.setAtPoint(true);
    }
  }

  private void checkBumpers() {
    boolean leftBumper = m_xboxController.getHID().getLeftBumperButton();
    boolean rightBumper = m_xboxController.getHID().getRightBumperButton();
    if (m_bumperWasPressed && (!leftBumper && !rightBumper)) {
      m_bumperWasPressed = false;
    }
    if (!m_reefPoses.contains(m_pointControl.getTarget()) || m_bumperWasPressed) {
      return;
    }
    m_bumperWasPressed = leftBumper || rightBumper;

    int id = m_reefPoses.indexOf(m_pointControl.getTarget());
    id += leftBumper ? -1 : rightBumper ? 1 : 0;
    id = id > 11 ? 0 : id < 0 ? 11 : id; // wrap value

    m_pointControl.setTarget(m_reefPoses.get(id));
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
