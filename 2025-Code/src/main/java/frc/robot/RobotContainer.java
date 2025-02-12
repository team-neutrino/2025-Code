// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.command_factories.*;
import frc.robot.commands.DriveAssistCom;
import frc.robot.util.DriveToPoint;
import frc.robot.util.Subsystem;

import static frc.robot.util.Subsystem.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // @SuppressWarnings("unused")
  private Subsystem subsystemContainer;
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_buttonsController = new CommandXboxController(
      OperatorConstants.kButtonsControllerPort);
  private final DriveToPoint m_driveToPoint = new DriveToPoint();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DigitalInput m_robot_select = new DigitalInput(9);
    subsystemContainer = new Subsystem(!m_robot_select.get());
    configureBindings();
    configureDefaultCommands();
    configureNamedCommands();
  }

  private void configureBindings() {
    // driver controller
    m_driverController.y().onTrue(ClimbFactory.raiseClimb());
    m_driverController.a().onTrue(ClimbFactory.lowerClimb());
    m_driverController.x().onTrue(ClimbFactory.lockGrabber());

    m_driverController.b()
        .whileTrue(Subsystem.swerve.swerveDriveToPoint(m_driveToPoint));

    m_driverController.leftStick().toggleOnTrue(new DriveAssistCom(m_driverController));
    m_driverController.back().whileTrue(swerve.resetYawCommand());

    // buttons controller
    m_buttonsController.y().whileTrue(SuperstructureFactory.scoreCoralL1Command());
    m_buttonsController.x().whileTrue(SuperstructureFactory.scoreCoralL2Command());
    m_buttonsController.b().whileTrue(SuperstructureFactory.scoreCoralL3Command());
    m_buttonsController.a().whileTrue(SuperstructureFactory.scoreCoralL4Command());

    m_buttonsController.leftBumper().whileTrue(SuperstructureFactory.intakeCoral());
    m_buttonsController.rightBumper().whileTrue(SuperstructureFactory.outtake());
    m_buttonsController.rightTrigger().whileTrue(SuperstructureFactory.dunkL4Command());
  }

  private void configureDefaultCommands() {
    claw.setDefaultCommand(claw.clawDefaultCommand());
    wrist.setDefaultCommand(wrist.wristDefaultCommand());
    arm.setDefaultCommand(arm.armDefaultCommand());
    elevator.setDefaultCommand(elevator.elevatorDefaultCommand());
    LED.setDefaultCommand(LED.LEDefaultCommand());
    limelight.setDefaultCommand(limelight.limelightDefaultCommand());
    climb.setDefaultCommand(climb.climbDefaultCommand());
    if (swerve == null) {
      return;
    }
    swerve.setDefaultCommand(swerve.swerveDefaultCommand(m_driverController));
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("MoveToScoringL4", SuperstructureFactory.moveToScoreL4Command());
    NamedCommands.registerCommand("MoveToScoringL3", SuperstructureFactory.moveToScoreL3Command());
    NamedCommands.registerCommand("MoveToScoringL2", SuperstructureFactory.moveToScoreL2Command());
    NamedCommands.registerCommand("MoveToScoringL1", SuperstructureFactory.moveToScoreL1Command());
    NamedCommands.registerCommand("MoveToIntake", SuperstructureFactory.moveToIntake());
    NamedCommands.registerCommand("ScoreL4", SuperstructureFactory.scoreCoralL4AutonCommand());
    NamedCommands.registerCommand("ScoreL3", SuperstructureFactory.scoreCoralL3AutonCommand());
    NamedCommands.registerCommand("ScoreL2", SuperstructureFactory.scoreCoralL2AutonCommand());
    NamedCommands.registerCommand("ScoreL1", SuperstructureFactory.scoreCoralL1AutonCommand());
    NamedCommands.registerCommand("Intake", SuperstructureFactory.intakeCoralAutonCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command auto;

    if (Subsystem.swerve == null) {
      return new InstantCommand();
    }
    try {
      auto = new PathPlannerAuto("test");
    } catch (Exception e) {
      auto = new PathPlannerAuto("Nothing");
    }

    return auto;
  }
}
