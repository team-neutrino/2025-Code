// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.Constants;
import frc.robot.util.Subsystem;
import edu.wpi.first.wpilibj.XboxController;

public class LED extends SubsystemBase {

  private NetworkTableInstance m_nt = NetworkTableInstance.getDefault();
  private StringTopic m_color_topic = m_nt.getStringTopic("/LED/color");
  private StringTopic m_state_topic = m_nt.getStringTopic("/LED/state");
  private Coral m_coral = Subsystem.coral;

  private XboxController m_buttonsxboxController;
  private XboxController m_driverxboxController = new XboxController(0);

  private final StringPublisher m_color_pub;
  private final StringPublisher m_state_pub;

  private boolean m_previousCoralIntake = false;

  public LED() {
    m_color_pub = m_color_topic.publish();
    m_state_pub = m_state_topic.publish();
  }

  private void setToGamePieceColor() {
    if (m_coral.hasCoral()) {
      System.out.println("working");
      if (m_previousCoralIntake) {
        m_driverxboxController.setRumble(GenericHID.RumbleType.kLeftRumble, Constants.OperatorConstants.RUMBLE_SPEED);
        // m_buttonsxboxController.setRumble(GenericHID.RumbleType.kBothRumble,
        // Constants.OperatorConstants.RUMBLE_SPEED);
      }
      m_state_pub.set("blinktwice");
      m_color_pub.set("white");

      m_previousCoralIntake = true;
    } else {
      m_previousCoralIntake = false;
    }
  }

  private void setColor() {
    if (DriverStation.isAutonomousEnabled()) {
      m_color_pub.set("cyan");
    } else if (DriverStation.isTeleopEnabled()) {
      if (m_coral.debouncedHasCoral()) {
        setToGamePieceColor();
      }
      setActionColor();
    } else {
      m_color_pub.set("orange");
      m_state_pub.set("solid");
    }
  }

  public void setActionColor() {
    if (Subsystem.swerve.isAtPoint()) {
      m_color_pub.set("green");
      m_state_pub.set("solid");
    } else if (Subsystem.swerve.isDrivingToPoint()) {
      m_color_pub.set("red");
    } else if (Subsystem.arm.readyToScore()) {
      m_color_pub.set("yellow");
    } else if (!Subsystem.arm.atTargetAngle()) {
      m_color_pub.set("purple");
    } else {
      m_color_pub.set("orange");
    }
  }

  @Override
  public void periodic() {
    setColor();
  }

  public Command LEDDefaultCommand(CommandXboxController p_buttonsController,
      CommandXboxController p_driverController) {
    return run(() -> m_driverxboxController = p_driverController.getHID());
  }
}
