// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveToPoint.Mode;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Swerve.SwerveRequestStash;
import frc.robot.util.DriveToPointController;
import frc.robot.util.Subsystem;

import static frc.robot.Constants.DriveToPoint.*;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.GlobalConstants.*;
import static frc.robot.util.Subsystem.swerve;
import static frc.robot.util.Subsystem.arm;

import java.util.List;

public class DriveToPointCommand extends Command {
  private DriveToPointController m_pointControl = new DriveToPointController();
  private CommandXboxController m_xboxController;
  private List<Pose2d> m_localList;
  private boolean m_bumperWasPressed = false;
  private boolean m_hadGamePiece;
  private Mode m_mode;
  private double m_distanceFromPoint;

  public DriveToPointCommand(CommandXboxController xboxController, Mode mode) {
    m_mode = mode;
    m_xboxController = xboxController;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    swerve.setDrivingToPoint(true);
    swerve.setAtPoint(false);

    if (!RED_ALLIANCE.isPresent()) {
      System.out.println("NO ALLIANCE VALUE YET");
      return;
    }
    m_hadGamePiece = Subsystem.coral.debouncedHasCoral();
    obtainTarget();
    setLocalList();
  }

  @Override
  public void execute() {
    boolean isSafePoint = PLAYER_STATION_SAFE.contains(m_pointControl.getTarget());

    if (!isSafePoint && (m_mode != Mode.ALGAE && m_mode != Mode.NET)) {
      checkBumpers();
    }
    drive();
    updateAtPoint();
    updateDistanceFromPoint();
    if ((swerve.isAtPoint() && (Subsystem.coral.debouncedHasCoral() != m_hadGamePiece))
        || (arm.isAtIntake() && isSafePoint)) {
      initialize(); // reinitialize if the state of our game piece changes
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setDrivingToPoint(false);
    swerve.setAtPoint(false);
    m_pointControl.setTarget(null);
    Subsystem.limelight.setDealgaefying(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void obtainTarget() {
    swerve.setDrivingToPoint(true);
    swerve.setAtPoint(false);
    Subsystem.limelight.setDealgaefying(m_mode == Mode.ALGAE);

    if (m_mode == Mode.ALGAE) {
      m_pointControl.setTargetNearest(REEF_ALGAE);
      return;
    }

    if (m_mode == Mode.NET) {
      m_pointControl.setTarget(RED_ALLIANCE.get() ? RED_BARGE_5 : BLUE_BARGE_14);
      return;
    }

    if (Subsystem.coral.debouncedHasCoral()) {
      switch (m_mode) {
        case LEFT:
          m_pointControl.setTargetNearest(RED_ALLIANCE.get() ? RED_REEF_LEFT : BLUE_REEF_LEFT);
          return;
        case NEAREST:
          m_pointControl.setTargetNearest(RED_ALLIANCE.get() ? RED_REEF : BLUE_REEF);
          return;
        case RIGHT:
          m_pointControl.setTargetNearest(RED_ALLIANCE.get() ? RED_REEF_RIGHT : BLUE_REEF_RIGHT);
          return;
        default:
          System.out.println("undefined behavior in obtainTarget; m_mode is null or an improper value");
          break;
      }
    }

    // by now the only case left is player station
    if (arm.isAtIntake()) {
      m_pointControl
          .setTargetNearest(RED_ALLIANCE.get() ? List.of(RED_PLAYER_STATION_1_CENTER, RED_PLAYER_STATION_2_CENTER)
              : List.of(BLUE_PLAYER_STATION_12_CENTER, BLUE_PLAYER_STATION_13_CENTER));
      return;
    }

    m_pointControl
        .setTargetNearest(PLAYER_STATION_SAFE);

  }

  /**
   * this method should only be called following a call to obtainTarget
   * <p>
   * does all logic based on set target point
   */
  private void setLocalList() {
    Pose2d target = m_pointControl.getTarget();
    if (REEF_ALGAE.contains(target)) {
      m_localList = REEF_ALGAE;
      return;
    }
    if (RED_REEF.contains(target)) {
      m_localList = RED_REEF;
      return;
    }
    if (BLUE_REEF.contains(target)) {
      m_localList = BLUE_REEF;
      return;
    }
    // at this point it's for sure a player station point
    if (RED_ALLIANCE.get()) {
      m_localList = target == RED_PLAYER_STATION_1_CENTER ? POSE_LIST.subList(0, 3) : POSE_LIST.subList(3, 6);
    } else {
      m_localList = target == BLUE_PLAYER_STATION_12_CENTER ? POSE_LIST.subList(6, 9)
          : POSE_LIST.subList(9, 12);
    }
  }

  public void updateAtPoint() {
    if ((m_mode == Mode.NET || appropriateLLHasTv())
        && Math.abs(m_pointControl.getStraightLineDist()) < AT_POINT_TOLERANCE) {
      swerve.setDrivingToPoint(false);
      swerve.setAtPoint(true);
    } else {
      swerve.setDrivingToPoint(true);
      swerve.setAtPoint(false);
    }
  }

  private void updateDistanceFromPoint() {
    m_distanceFromPoint = m_pointControl.getStraightLineDist();
  }

  public double getDistanceFromPoint() {
    return m_distanceFromPoint;
  }

  private boolean appropriateLLHasTv() {
    Pose2d target = m_pointControl.getTarget();
    if (RED_REEF_RIGHT.contains(target) || BLUE_REEF_RIGHT.contains(target) || REEF_ALGAE.contains(target)) {
      return Subsystem.limelight.getTvReef1();
    } else if (RED_REEF_LEFT.contains(target) || BLUE_REEF_LEFT.contains(target)) {
      return Subsystem.limelight.getTvReef2();
    } else {
      return Subsystem.limelight.getTvStation();
    }
  }

  private void checkBumpers() {
    boolean leftBumper = m_xboxController.getHID().getLeftBumperButton();
    boolean rightBumper = m_xboxController.getHID().getRightBumperButton();
    if (m_bumperWasPressed && (!leftBumper && !rightBumper)) {
      m_bumperWasPressed = false;
    }
    if (m_bumperWasPressed) {
      return;
    }
    m_bumperWasPressed = leftBumper || rightBumper;

    int id = m_localList.indexOf(m_pointControl.getTarget());
    id += leftBumper ? -1 : rightBumper ? 1 : 0;
    id = id >= m_localList.size() ? 0 : id < 0 ? m_localList.size() - 1 : id;

    m_pointControl.setTarget(m_localList.get(id));
  }

  private void drive() {
    double velx = m_pointControl.getXVelocity(), vely = m_pointControl.getYVelocity();
    double xsign = Math.signum(velx), ysign = Math.signum(vely);
    if (Subsystem.elevator.getHeight() >= ElevatorConstants.SLOW_MOVE_THRESHOLD) {
      velx = xsign * Math.min(Math.abs(velx), SLOW_SWERVE_SPEED);
      vely = ysign * Math.min(Math.abs(vely), SLOW_SWERVE_SPEED);
    }

    velx = MathUtil.clamp(velx, -MAX_DRIVETOPOINT_SPEED, MAX_DRIVETOPOINT_SPEED);
    vely = MathUtil.clamp(vely, -MAX_DRIVETOPOINT_SPEED, MAX_DRIVETOPOINT_SPEED);

    SwerveRequestStash.driveWithVelocity
        .withVelocityX(velx)
        .withVelocityY(vely)
        .withTargetDirection(m_pointControl.getRotation());
    swerve.setControl(SwerveRequestStash.driveWithVelocity);
  }
}
