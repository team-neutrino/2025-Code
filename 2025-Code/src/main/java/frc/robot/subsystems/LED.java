// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.StringTopic;

import static frc.robot.util.Subsystem.claw;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.subsystems.*;
import frc.robot.util.Subsystem;

public class LED extends SubsystemBase {
  private int m_counter = 0;
  private int m_counter2 = 0;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  StringTopic color_topic = inst.getStringTopic("/LED/color");
  Claw claw = Subsystem.claw;

  final StringPublisher color_pub;

  public LED() {
    color_pub = color_topic.publish();

  }

  private boolean slowDown(int rate) {
    m_counter2++;
    return m_counter2 % rate == 0;
  }

  public Command LEDefaultCommand() {
    return run(() -> color_pub.setDefault("orange"));
  }

  // Command or void? What to return(forgot)
  public void setToGamePieceColor() {
    if (claw.isAlgae()) {
      color_pub.set("turquoise");
    } else if (claw.isCoral()) {
      color_pub.set("white");
    }
  }

  @Override
  public void periodic() {
    if (claw.hasGamePiece()) {
      setToGamePieceColor();
      return;
    }

    // if (slowdown(50)) {
    // if (m_counter == 0) {
    // color_pub.set("white");
    // m_counter++;
    // } else if (m_counter == 1) {
    // color_pub.set("orange");
    // m_counter++;
    // } else if (m_counter == 2) {
    // color_pub.set("blue");
    // m_counter = 0;
  }
}
