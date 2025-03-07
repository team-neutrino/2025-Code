// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import static frc.robot.Constants.DriveToPoint.*;
import frc.robot.command_factories.*;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.util.Subsystem;

import static frc.robot.util.Subsystem.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // "layout" wouldn't initialize until the b button was pressed, causing a 2-3
    // second delay. this forces it to initialize on startup.
    AprilTagFieldLayout tmp = Constants.DriveToPoint.layout;
    DigitalInput m_robot_select = new DigitalInput(9);
    subsystemContainer = new Subsystem(!m_robot_select.get());
    configureBindings();
    configureDefaultCommands();
    configureNamedCommands();
  }

  private void configureBindings() {
    // driver controller
    // ONLY RUN CLIMB IN ORDER AS LISTED BELOW vvv
    m_driverController.y().onTrue(ClimbFactory.raiseClimb());
    m_driverController.x().onTrue(ClimbFactory.lockGrabber());
    m_driverController.a().onTrue(ClimbFactory.lowerClimb());

    m_driverController.back().whileTrue(swerve.resetYawCommand());
    m_driverController.b().whileTrue(new DriveToPointCommand(m_driverController, false));

    Command deAlgae = new DriveToPointCommand(m_driverController, true);
    deAlgae.setName(ALGAE_ALIGN_COMMAND);
    m_driverController.rightTrigger().whileTrue(deAlgae);

    // buttons controller
    m_buttonsController.x().whileTrue(SuperstructureFactory.scoreL1(m_buttonsController));
    m_buttonsController.y().whileTrue(SuperstructureFactory.scoreL2(m_buttonsController));
    m_buttonsController.b().whileTrue(SuperstructureFactory.scoreL3(m_buttonsController));
    m_buttonsController.a().whileTrue(SuperstructureFactory.scoreL4(m_buttonsController));

    m_buttonsController.rightTrigger().whileTrue(SuperstructureFactory.descoreAlgaeL3());
    m_buttonsController.leftTrigger().whileTrue(SuperstructureFactory.descoreAlgaeL2());

    m_buttonsController.leftBumper().whileTrue(SuperstructureFactory.intakeCoral());
  }

  private void configureDefaultCommands() {
    coral.setDefaultCommand(coral.coralDefaultCommand());
    arm.setDefaultCommand(arm.armDefaultCommand());
    elevator.setDefaultCommand(elevator.elevatorDefaultCommand());
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
    NamedCommands.registerCommand("KeepCoralIn", CoralFactory.runSlowIntake());
    NamedCommands.registerCommand("DriveToPoint",
        new DriveToPointCommand(m_driverController, false).until(() -> swerve.isAtPointDebounced()));
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
      auto = new PathPlannerAuto("2 CORAL TOP");
    } catch (Exception e) {
      auto = new PathPlannerAuto("Nothing");
    }

    return auto;
  }
}
