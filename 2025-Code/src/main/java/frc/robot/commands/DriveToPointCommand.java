// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.Constants.DriveToPoint.*;
import frc.robot.subsystems.Swerve.SwerveRequestStash;
import frc.robot.util.DriveToPoint;
import static frc.robot.util.Subsystem.swerve;

public class DriveToPointCommand extends Command {
  private Pose2d m_poseTarget = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  private DriveToPoint m_driveController = new DriveToPoint();
  private CommandXboxController m_xboxController = new CommandXboxController(0);
  private int m_poseIndex = -1;
  private Timer m_timer = new Timer();

  public DriveToPointCommand(DriveToPoint driveController, CommandXboxController xboxController) {
    m_driveController = driveController;
    m_xboxController = xboxController;

    addRequirements(swerve);
  }

  public void changeTargetAndIndex(int index) {
    m_poseIndex %= RED_REEF.size();
    m_poseTarget = RED_REEF.get(m_poseIndex);
    DriveToPoint.setTarget(m_poseTarget);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_poseTarget = swerve.getCurrentPose().nearest(POSE_LIST);
    DriveToPoint.setTarget(m_poseTarget);
    m_poseIndex = RED_REEF.indexOf(m_poseTarget);
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() >= 0.5 && RED_REEF.contains(m_poseTarget)) {
      int pov = m_xboxController.getHID().getPOV();
      if (pov == 270) {
        m_poseIndex--;
        m_timer.reset();
      } else if (pov == 90) {
        m_poseIndex++;
        m_timer.reset();
      }
      changeTargetAndIndex(m_poseIndex);
    }

    SwerveRequestStash.driveWithVelocity.withVelocityX(m_driveController.getXVelocity())
        .withVelocityY(m_driveController.getYVelocity()).withTargetDirection(m_driveController.getRotation());
    swerve.setControl(SwerveRequestStash.driveWithVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
