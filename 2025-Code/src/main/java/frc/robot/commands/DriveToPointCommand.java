// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve.SwerveRequestStash;
import frc.robot.util.DriveToPointController;

import static frc.robot.Constants.DriveToPoint.*;
import static frc.robot.Constants.GlobalConstants.*;
import static frc.robot.util.Subsystem.swerve;

import java.util.List;

public class DriveToPointCommand extends Command {
  private DriveToPointController m_pointControl = new DriveToPointController();
  private CommandXboxController m_xboxController;
  private List<Pose2d> m_reefPoses;
  private boolean m_bumperWasPressed = false;

  public DriveToPointCommand(CommandXboxController xboxController) {
    m_xboxController = xboxController;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    m_pointControl.setTargetNearest();
  }

  @Override
  public void execute() {
    if (!redAlliance.isPresent()) {
      System.out.println("NO ALLIANCE VALUE YET");
      return;
    }
    m_reefPoses = redAlliance.get() ? RED_REEF : BLUE_REEF;

    checkDPad();
    drive();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    boolean triedMove = Math.abs(m_xboxController.getLeftX()) > .5 || Math.abs(m_xboxController.getLeftY()) > .5;
    boolean triedTurn = Math.abs(m_xboxController.getRightX()) > .5;
    return triedMove || triedTurn;
  }

  private void checkDPad() {
    boolean leftBumper = m_xboxController.getHID().getLeftBumperButton();
    boolean rightBumper = m_xboxController.getHID().getRightBumperButton();
    if (!leftBumper && !rightBumper) {
      m_bumperWasPressed = false;
    }
    if (!m_reefPoses.contains(m_pointControl.getTarget()) || m_bumperWasPressed) {
      return;
    }

    int id = m_reefPoses.indexOf(m_pointControl.getTarget());
    // TODO: test whether blue alliance modifier needs to be switched
    // update with d-pad direction; 90 = right and 270 = left
    id += leftBumper ? -1 : rightBumper ? 1 : 0;
    id = id > 11 ? 0 : id < 0 ? 11 : id; // wrap value

    m_pointControl.setTarget(m_reefPoses.get(id));
    if (leftBumper || rightBumper) {
      m_bumperWasPressed = true;
    }
  }

  private void drive() {
    SwerveRequestStash.driveWithVelocity.withVelocityX(m_pointControl.getXVelocity())
        .withVelocityY(m_pointControl.getYVelocity()).withTargetDirection(m_pointControl.getRotation());
    swerve.setControl(SwerveRequestStash.driveWithVelocity);
  }
}
