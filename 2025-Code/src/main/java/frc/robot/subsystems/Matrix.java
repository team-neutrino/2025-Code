// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.StringTopic;

import static frc.robot.util.Subsystem.limelight;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.util.Subsystem;

public class LED extends SubsystemBase {

  private NetworkTableInstance m_nt = NetworkTableInstance.getDefault();
  private StringTopic m_color_topic = m_nt.getStringTopic("/LED/color");
  private StringTopic m_state_topic = m_nt.getStringTopic("/LED/state");
  private Coral m_coral = Subsystem.coral;
  private Algae m_algae = Subsystem.algae;

  private final StringPublisher m_color_pub;
  private final StringPublisher m_state_pub;

  public LED() {
    m_color_pub = m_color_topic.publish();
    m_state_pub = m_state_topic.publish();
  }

  @Override
  public void periodic() {

    if (Subsystem.swerve.isDrivingToPoint() && !limelight.getTvReef1() && !limelight.getTvReef2()) {
      m_color_pub.set("pink");
      m_state_pub.set("solid");
      return;
    }

    if (Subsystem.swerve.isAtPoint()) {
      m_color_pub.set("green");
      m_state_pub.set("solid");
      return;
    }

    if (m_coral.debouncedHasCoral()) {
      m_state_pub.set("blinktwice");
      m_color_pub.set("white");
      return;
    }

    if (m_algae.debouncedHasAlgae()) {
      m_state_pub.set("blinktwice");
      m_color_pub.set("turquoise");
      return;
    }

    // default to orange
    m_color_pub.set("orange");
    m_state_pub.set("solid");
  }
}
