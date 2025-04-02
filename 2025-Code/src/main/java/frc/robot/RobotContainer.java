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
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // @SuppressWarnings("unused")
  private Subsystem subsystemContainer;
  private Command m_autonPath;
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
    DataLogManager.start();
    DataLogManager.logNetworkTables(true);
    m_autonPath = new PathPlannerAuto("3 CORAL PROCESSOR");

  }

  private void configureBindings() {
    // driver controller
    // ONLY RUN CLIMB IN ORDER AS LISTED BELOW vvv
    m_driverController.y().onTrue(ClimbFactory.raiseClimb());
    m_driverController.a().onTrue(ClimbFactory.lowerClimb());

    m_driverController.back().whileTrue(swerve.resetYawCommand());
    m_driverController.b().whileTrue(new DriveToPointCommand(m_driverController, Mode.NEAREST));

    Command deAlgae = new DriveToPointCommand(m_driverController, Mode.ALGAE);
    deAlgae.setName(ALGAE_ALIGN_COMMAND);
    m_driverController.rightTrigger().whileTrue(deAlgae);
    m_driverController.leftTrigger().whileTrue(swerve.slowDefaultCommand(m_driverController));

    // buttons controller
    m_buttonsController.x().whileTrue(SuperstructureFactory.scoreL1(m_buttonsController));
    m_buttonsController.y().whileTrue(SuperstructureFactory.scoreL2(m_buttonsController));
    m_buttonsController.b().whileTrue(SuperstructureFactory.scoreL3(m_buttonsController));
    m_buttonsController.a().whileTrue(SuperstructureFactory.scoreL4(m_buttonsController));

    m_buttonsController.start().whileTrue(AlgaeFactory.runOuttake());

    m_buttonsController.povUp().whileTrue(SuperstructureFactory.scoreBargeCommand(m_buttonsController));
    m_buttonsController.povDown().whileTrue(SuperstructureFactory.scoreProcessorCommand(m_buttonsController));

    m_buttonsController.povLeft().whileTrue(SuperstructureFactory.descoreAlgaeL2());
    m_buttonsController.povRight().whileTrue(SuperstructureFactory.descoreAlgaeL3());

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
    algae.setDefaultCommand(algae.algaeDefaultCommand());
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
    NamedCommands.registerCommand("ScoreL4Teleop", SuperstructureFactory.scoreL4(m_buttonsController));
    NamedCommands.registerCommand("ScoreL3Teleop", SuperstructureFactory.scoreL3(m_buttonsController));
    NamedCommands.registerCommand("ScoreL2Teleop", SuperstructureFactory.scoreL2(m_buttonsController));
    NamedCommands.registerCommand("ScoreL1Teleop", SuperstructureFactory.scoreL1(m_buttonsController));
    NamedCommands.registerCommand("Intake", SuperstructureFactory.intakeCoral());
    NamedCommands.registerCommand("Outtake", SuperstructureFactory.outtake());
    NamedCommands.registerCommand("DescoreAlgaeL3", SuperstructureFactory.descoreAlgaeL3());
    NamedCommands.registerCommand("DescoreAlgaeL2", SuperstructureFactory.descoreAlgaeL2());
    NamedCommands.registerCommand("ScoreProcessor", SuperstructureFactory.scoreProcessorCommand(m_buttonsController));
    NamedCommands.registerCommand("ScoreBarge", SuperstructureFactory.scoreBargeCommand(m_buttonsController));
    NamedCommands.registerCommand("KeepCoralIn", CoralFactory.runSlowIntake());
    NamedCommands.registerCommand("DriveToPoint",
        new DriveToPointCommand(m_driverController, Mode.NEAREST).until(() -> swerve.isAtPointStable()));
    NamedCommands.registerCommand("DriveToPointLeft",
        new DriveToPointCommand(m_driverController, Mode.LEFT).until(() -> swerve.isAtPointStable()));
    NamedCommands.registerCommand("DriveToPointRight",
        new DriveToPointCommand(m_driverController, Mode.RIGHT).until(() -> swerve.isAtPointStable()));
    NamedCommands.registerCommand("DriveToPointForever",
        new DriveToPointCommand(m_driverController, Mode.NEAREST));
    NamedCommands.registerCommand("SwerveDefault", swerve.getDefaultCommand());
    NamedCommands.registerCommand("IntakeOnly", CoralFactory.runIntake());
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
      auto = m_autonPath;
    } catch (Exception e) {
      auto = new PathPlannerAuto("Nothing");
    }

    return auto;
  }
}
