// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.util.Subsystem;

public class LED extends SubsystemBase {

  private NetworkTableInstance m_nt = NetworkTableInstance.getDefault();
  private StringTopic m_color_topic = m_nt.getStringTopic("/LED/color");
  private StringTopic m_state_topic = m_nt.getStringTopic("/LED/state");
  private Coral m_coral = Subsystem.coral;

  private final StringPublisher m_color_pub;
  private final StringPublisher m_state_pub;

  public LED() {
    m_color_pub = m_color_topic.publish();
    m_state_pub = m_state_topic.publish();
  }

  private void setToGamePieceColor() {
    if (m_coral.hasCoral()) {
      m_state_pub.set("blinktwice");
      m_color_pub.set("white");
    }
  }

  private void setColor() {
    if (DriverStation.isAutonomousEnabled()) {
      m_color_pub.set("cyan");
    } else if (DriverStation.isTeleopEnabled()) {
      setActionColor();
      if (coral.debouncedHasCoral()) {
        setToGamePieceColor();
      }
    } else {
      m_color_pub.set("orange");
      m_state_pub.set("blink");
    }
  }

  public void setActionColor() {
    if (Subsystem.arm.isAtTarget()) {
      color_pub.set("pink");
    } else if (Subsystem.arm.goingToTarget()) {
      color_pub.set("purple");
    } else if (Subsystem.swerve.isDrivingToPoint()) {
      color_pub.set("red");
    } else if (Subsystem.swerve.isAtPoint()) {
      m_color_pub.set("green");
    } else {
      m_color_pub.set("orange");
    }
    m_state_pub.set("solid");
  }

  @Override
  public void periodic() {
  }
}
