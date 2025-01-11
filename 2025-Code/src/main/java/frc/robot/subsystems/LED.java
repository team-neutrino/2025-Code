// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class LED extends SubsystemBase {

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  StringTopic Color_topic = inst.getStringTopic("/LED/color");

  final StringPublisher color_pub;

  // Creates a new LED Subsystem
  public LED() {
    color_pub = Color_topic.publish();
    color_pub.setDefault(getName());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
