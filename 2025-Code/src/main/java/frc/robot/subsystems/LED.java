// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class LED extends SubsystemBase {
  private int m_counter = 0;
  private int m_counter2 = 0;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  StringTopic color_topic = inst.getStringTopic("/LED/color");

  final StringPublisher color_pub;

  public LED() {
    color_pub = color_topic.publish();
    color_pub.setDefault("orange");
  }

  private boolean slowDown(int rate) {
    m_counter2++;
    return m_counter2 % rate == 0;
  }

  public Command LEDefaultCommand() {
    return run(() -> {

    });
  }

  @Override
  public void periodic() {
    if (slowDown(50)) {
      if (m_counter == 0) {
        color_pub.set("white");
        m_counter++;
      } else if (m_counter == 1) {
        color_pub.set("orange");
        m_counter++;
      } else if (m_counter == 2) {
        color_pub.set("blue");
        m_counter++;
      } else if (m_counter == 3) {
        color_pub.set("indigo");
        m_counter = 0;
      }
    }
  }
}