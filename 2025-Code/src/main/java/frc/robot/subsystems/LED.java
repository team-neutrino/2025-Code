// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class LED extends SubsystemBase {
  private int counter = 0;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  StringTopic color_topic = inst.getStringTopic("/LED/color");

  final StringPublisher color_pub;

  public LED() {
    color_pub = color_topic.publish();
    color_pub.setDefault("white");
  }

  @Override
  public void periodic() {
    counter++;
    if (counter % 50 == 0) {
      if (counter % 100 == 0) {
        color_pub.set("white");
        return;
      }
      color_pub.set("orange");

    }

  }
}