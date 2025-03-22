// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import static frc.robot.Constants.DriveToPoint.*;
import frc.robot.util.Subsystem;

import static frc.robot.util.Subsystem.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DataLogManager;
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
    subsystemContainer = new Subsystem(true);
    configureBindings();
    configureDefaultCommands();
    DataLogManager.start();
    m_autonPath = new PathPlannerAuto("3 CORAL PROCESSOR");

  }

  private void configureBindings() {
    // driver controller
    // ONLY RUN CLIMB IN ORDER AS LISTED BELOW vvv

    m_driverController.back().whileTrue(swerve.resetYawCommand());

    m_driverController.leftTrigger().whileTrue(swerve.slowDefaultCommand(m_driverController));

    // buttons controller
  }

  private void configureDefaultCommands() {
    if (swerve == null) {
      return;
    }
    swerve.setDefaultCommand(swerve.swerveDefaultCommand(m_driverController));

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
