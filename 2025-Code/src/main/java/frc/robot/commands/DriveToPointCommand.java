// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.SwerveRequestStash;
import frc.robot.util.DriveToPointController;
import frc.robot.util.Subsystem;

import static frc.robot.Constants.DriveToPoint.*;
import static frc.robot.Constants.GlobalConstants.*;
import static frc.robot.Constants.SwerveConstants.IS_AT_POINT;
import static frc.robot.util.Subsystem.swerve;

import java.util.List;

public class DriveToPointCommand extends Command {
  private DriveToPointController m_pointControl = new DriveToPointController();
  private CommandXboxController m_xboxController;
  private List<Pose2d> m_reefPoses;
  private List<Pose2d> m_coralStationPoses;
  private boolean m_bumperWasPressed = false;

  public DriveToPointCommand(CommandXboxController xboxController) {
    m_xboxController = xboxController;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    swerve.setDrivingToPoint(true);
    if (!redAlliance.isPresent()) {
      System.out.println("NO ALLIANCE VALUE YET");
      return;
    }
    m_reefPoses = redAlliance.get() ? RED_REEF : BLUE_REEF;
    m_coralStationPoses = redAlliance.get() ? POSE_LIST.subList(0, 2) : POSE_LIST.subList(2, 4);

    obtainTarget();
  }

  @Override
  public void execute() {
    if (!redAlliance.isPresent()) {
      System.out.println("NO ALLIANCE VALUE YET");
      return;
    }
    m_reefPoses = redAlliance.get() ? RED_REEF : BLUE_REEF;
    m_coralStationPoses = redAlliance.get() ? POSE_LIST.subList(0, 2) : POSE_LIST.subList(2, 4);

    checkBumpers();
    drive();
    isAtPoint();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setDrivingToPoint(false);
    swerve.setAtPoint(false);
  }

  @Override
  public boolean isFinished() {
    boolean triedMove = Math.abs(m_xboxController.getLeftX()) > .5 || Math.abs(m_xboxController.getLeftY()) > .5;
    boolean triedTurn = Math.abs(m_xboxController.getRightX()) > .5;
    return triedMove || triedTurn;
  }

  private void obtainTarget() {
    boolean hasGamePiece = Subsystem.coral.hasCoral();
    if (hasGamePiece) {
      m_pointControl.setTargetNearest(m_reefPoses);
    } else {
      m_pointControl.setTargetNearest(m_coralStationPoses);
    }
  }

  public void isAtPoint() {
    if (Math.abs(m_pointControl.getTarget().getX() - swerve.getCurrentPose().getX()) < IS_AT_POINT) {
      if (Math.abs(m_pointControl.getTarget().getY() - swerve.getCurrentPose().getY()) < IS_AT_POINT) {
        swerve.setDrivingToPoint(false);
        swerve.setAtPoint(true);
      }
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
    SwerveRequestStash.driveWithVelocity.withVelocityX(m_pointControl.getXVelocity())
        .withVelocityY(m_pointControl.getYVelocity()).withTargetDirection(m_pointControl.getRotation());
    swerve.setControl(SwerveRequestStash.driveWithVelocity);
  }
}
