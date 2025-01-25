// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.command_factories.ArmFactory;
import frc.robot.command_factories.ClimbFactory;
import frc.robot.command_factories.ClawFactory;
import frc.robot.command_factories.ElevatorFactory;
import frc.robot.command_factories.WristFactory;
import frc.robot.subsystems.Climb;
import frc.robot.util.Subsystem;

import static frc.robot.util.Subsystem.*;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // @SuppressWarnings("unused")
  private Subsystem subsystemContainer = new Subsystem();
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_buttonsController = new CommandXboxController(
      OperatorConstants.kButtonsControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    configureDefaultCommands();

    m_driverController.x().whileTrue(ElevatorFactory.moveL1());
    m_driverController.y().whileTrue(ElevatorFactory.moveL2());
    m_driverController.b().whileTrue(ElevatorFactory.moveL3());
    m_driverController.a().whileTrue(ElevatorFactory.moveL4());

    m_driverController.leftBumper().whileTrue(ArmFactory.moveToL1());

    m_buttonsController.y().toggleOnTrue(ClimbFactory.raiseClimb());
    m_buttonsController.x().toggleOnTrue(ClimbFactory.lowerClimb());
    m_buttonsController.a().toggleOnTrue(ClimbFactory.lockGrabber());
  }

  private void configureDefaultCommands() {
    claw.setDefaultCommand(claw.clawDefaultCommand());
    wrist.setDefaultCommand(wrist.wristDefaultCommand());
    arm.setDefaultCommand(arm.armDefaultCommand());
    elevator.setDefaultCommand(elevator.elevatorDefaultCommand());
    LED.setDefaultCommand(LED.LEDefaultCommand());
    limelight.setDefaultCommand(limelight.limelightDefaultCommand());
    climb.setDefaultCommand(climb.climbDefaultCommand());
    // swerve.setDefaultCommand(swerve.swerveDefaultCommand(m_driverController));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command auto;
    try {
      auto = new PathPlannerAuto("test");
    } catch (Exception e) {
      auto = new PathPlannerAuto("Nothing");
    }

    return auto;
  }
}
